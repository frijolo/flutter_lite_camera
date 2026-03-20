#include "include/Camera.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <csetjmp>
#include <cstring>
#include <jpeglib.h>

// ---------------------------------------------------------------------------
// YUYV → RGB conversion
//
// YUV formulas (ITU-R BT.601):
//   R = Y + 1.402   * (V − 128)
//   G = Y − 0.34414 * (U − 128) − 0.71414 * (V − 128)
//   B = Y + 1.772   * (U − 128)
// ---------------------------------------------------------------------------

static unsigned char clampU8(double value)
{
    if (value < 0.0) return 0;
    if (value > 255.0) return 255;
    return static_cast<unsigned char>(value);
}

// Converts a YUYV (YUY2) buffer to RGB888.
// Output order: R, G, B (one byte each per pixel).
static void ConvertYUY2ToRGB(const unsigned char *yuy2Data,
                               unsigned char *rgbData,
                               int width, int height)
{
    int rgbIndex = 0;
    for (int i = 0; i < width * height * 2; i += 4)
    {
        unsigned char y1 = yuy2Data[i];
        unsigned char u  = yuy2Data[i + 1];
        unsigned char y2 = yuy2Data[i + 2];
        unsigned char v  = yuy2Data[i + 3];

        // First pixel
        rgbData[rgbIndex++] = clampU8(y1 + 1.402   * (v - 128));                        // R
        rgbData[rgbIndex++] = clampU8(y1 - 0.34414 * (u - 128) - 0.71414 * (v - 128)); // G
        rgbData[rgbIndex++] = clampU8(y1 + 1.772   * (u - 128));                        // B

        // Second pixel
        rgbData[rgbIndex++] = clampU8(y2 + 1.402   * (v - 128));                        // R
        rgbData[rgbIndex++] = clampU8(y2 - 0.34414 * (u - 128) - 0.71414 * (v - 128)); // G
        rgbData[rgbIndex++] = clampU8(y2 + 1.772   * (u - 128));                        // B
    }
}

// ---------------------------------------------------------------------------
// MJPEG → RGB888 decoding via libjpeg
// ---------------------------------------------------------------------------

struct JpegErrorMgr
{
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

static void jpegErrorExit(j_common_ptr cinfo)
{
    JpegErrorMgr *myerr = reinterpret_cast<JpegErrorMgr *>(cinfo->err);
    longjmp(myerr->setjmp_buffer, 1);
}

// Decodes a JPEG-compressed frame into an RGB888 buffer (pre-allocated,
// width*height*3 bytes).  Returns true on success.
static bool DecodeJPEGToRGB(const unsigned char *jpegData, size_t jpegSize,
                              unsigned char *rgbData, int width, int height)
{
    struct jpeg_decompress_struct cinfo;
    JpegErrorMgr jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpegErrorExit;

    if (setjmp(jerr.setjmp_buffer))
    {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo,
                 const_cast<unsigned char *>(jpegData),
                 static_cast<unsigned long>(jpegSize));

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK)
    {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    const int rowStride = static_cast<int>(cinfo.output_width) * 3;
    while (cinfo.output_scanline < cinfo.output_height)
    {
        // Write directly into rgbData, capped to the declared buffer size.
        if (static_cast<int>(cinfo.output_scanline) >= height) break;
        unsigned char *row = rgbData + cinfo.output_scanline * rowStride;
        jpeg_read_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

// ---------------------------------------------------------------------------
// Camera::Open
// ---------------------------------------------------------------------------

bool Camera::Open(int cameraIndex)
{
    std::string devicePath = "/dev/video" + std::to_string(cameraIndex);
    fd = open(devicePath.c_str(), O_RDWR);
    if (fd < 0)
    {
        perror("Error opening video device");
        return false;
    }

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        perror("Error querying device capabilities");
        close(fd);
        return false;
    }

    std::cout << "Driver: " << cap.driver << "\nCard: " << cap.card << std::endl;

    // Request YUYV format. The driver may silently negotiate a different format
    // (e.g. MJPEG if the device doesn't support YUYV).  Read back the actual
    // negotiated format after the ioctl so CaptureFrame() can decode correctly.
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = frameWidth;
    fmt.fmt.pix.height      = frameHeight;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        perror("Error setting format");
        close(fd);
        return false;
    }

    // Accept the resolution and format the driver actually settled on.
    frameWidth  = fmt.fmt.pix.width;
    frameHeight = fmt.fmt.pix.height;
    pixelFormat = fmt.fmt.pix.pixelformat;

    if (pixelFormat == V4L2_PIX_FMT_YUYV)
        std::cout << "Negotiated format: YUYV " << frameWidth << "x" << frameHeight << std::endl;
    else if (pixelFormat == V4L2_PIX_FMT_MJPEG)
        std::cout << "Negotiated format: MJPEG " << frameWidth << "x" << frameHeight << std::endl;
    else
    {
        // Try to force MJPEG as a fallback when the requested format is unknown.
        memset(&fmt, 0, sizeof(fmt));
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = frameWidth;
        fmt.fmt.pix.height      = frameHeight;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) == 0)
        {
            frameWidth  = fmt.fmt.pix.width;
            frameHeight = fmt.fmt.pix.height;
            pixelFormat = fmt.fmt.pix.pixelformat;
            std::cout << "Negotiated format: MJPEG (fallback) " << frameWidth << "x" << frameHeight << std::endl;
        }
        else
        {
            std::cerr << "Unknown pixel format 0x" << std::hex << pixelFormat
                      << std::dec << " — frames may appear corrupted." << std::endl;
        }
    }

    if (!InitDevice() || !StartCapture())
    {
        Release();
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Camera::Release
// ---------------------------------------------------------------------------

void Camera::Release()
{
    StopCapture();
    UninitDevice();
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
}

// ---------------------------------------------------------------------------
// Camera::SetResolution
// ---------------------------------------------------------------------------

bool Camera::SetResolution(int width, int height)
{
    if (fd == -1)
    {
        std::cerr << "Device not opened." << std::endl;
        return false;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = pixelFormat; // keep the negotiated format

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        perror("Error setting resolution");
        return false;
    }

    frameWidth  = fmt.fmt.pix.width;
    frameHeight = fmt.fmt.pix.height;
    pixelFormat = fmt.fmt.pix.pixelformat;

    return true;
}

// ---------------------------------------------------------------------------
// Camera::CaptureFrame
// ---------------------------------------------------------------------------

FrameData Camera::CaptureFrame()
{
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    {
        perror("Failed to dequeue buffer");
        return {};
    }

    FrameData frame;
    frame.width   = frameWidth;
    frame.height  = frameHeight;
    frame.size    = frameWidth * frameHeight * 3; // 3 bytes per pixel (RGB888)
    frame.rgbData = new unsigned char[frame.size];

    const unsigned char *rawData =
        reinterpret_cast<unsigned char *>(buffers[buf.index].start);

    // Detect the actual content: JPEG starts with SOI marker 0xFF 0xD8.
    const bool isJpeg =
        (pixelFormat == V4L2_PIX_FMT_MJPEG) ||
        (buf.bytesused >= 2 && rawData[0] == 0xFF && rawData[1] == 0xD8);

    if (isJpeg)
    {
        if (!DecodeJPEGToRGB(rawData, buf.bytesused,
                              frame.rgbData, frameWidth, frameHeight))
        {
            std::cerr << "MJPEG decode failed — blank frame." << std::endl;
            memset(frame.rgbData, 0, frame.size);
        }
    }
    else
    {
        ConvertYUY2ToRGB(rawData, frame.rgbData, frameWidth, frameHeight);
    }

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
    {
        perror("Failed to requeue buffer");
    }

    return frame;
}

// ---------------------------------------------------------------------------
// Camera::InitDevice / UninitDevice / StartCapture / StopCapture
// ---------------------------------------------------------------------------

bool Camera::InitDevice()
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0)
    {
        perror("Failed to request buffers");
        return false;
    }

    buffers     = new Buffer[req.count];
    bufferCount = req.count;

    for (unsigned int i = 0; i < bufferCount; ++i)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            perror("Failed to query buffer");
            return false;
        }

        buffers[i].length = buf.length;
        buffers[i].start  = mmap(nullptr, buf.length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED)
        {
            perror("Failed to map buffer");
            return false;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
        {
            perror("Failed to queue buffer");
            return false;
        }
    }
    return true;
}

void Camera::UninitDevice()
{
    if (buffers)
    {
        for (unsigned int i = 0; i < bufferCount; ++i)
            munmap(buffers[i].start, buffers[i].length);
        delete[] buffers;
        buffers = nullptr;
    }
}

bool Camera::StartCapture()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(fd, VIDIOC_STREAMON, &type) >= 0;
}

void Camera::StopCapture()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
}

// ---------------------------------------------------------------------------
// Camera::ListSupportedMediaTypes
// ---------------------------------------------------------------------------

std::vector<MediaTypeInfo> Camera::ListSupportedMediaTypes()
{
    std::vector<MediaTypeInfo> mediaTypes;

    if (fd == -1)
    {
        std::cerr << "Device not opened.\n";
        return mediaTypes;
    }

    struct v4l2_fmtdesc fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0)
    {
        struct v4l2_frmsizeenum frmsize;
        memset(&frmsize, 0, sizeof(frmsize));
        frmsize.pixel_format = fmt.pixelformat;

        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0)
        {
            MediaTypeInfo info = {};
            info.width  = 0;
            info.height = 0;

            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
            {
                info.width  = frmsize.discrete.width;
                info.height = frmsize.discrete.height;
            }
            else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
            {
                info.width  = frmsize.stepwise.min_width;
                info.height = frmsize.stepwise.min_height;
            }

            strncpy(info.subtypeName,
                    reinterpret_cast<const char *>(fmt.description),
                    sizeof(info.subtypeName) - 1);
            info.subtypeName[sizeof(info.subtypeName) - 1] = '\0';

            mediaTypes.push_back(info);
            frmsize.index++;
        }
        fmt.index++;
    }

    if (mediaTypes.empty())
        std::cerr << "No supported media types found.\n";

    return mediaTypes;
}

// ---------------------------------------------------------------------------
// ListCaptureDevices (free function)
// ---------------------------------------------------------------------------

std::vector<CaptureDeviceInfo> ListCaptureDevices()
{
    std::vector<CaptureDeviceInfo> devices;

    for (int i = 0; i < 10; ++i)
    {
        std::string devicePath = "/dev/video" + std::to_string(i);
        int devFd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (devFd == -1) continue;

        struct v4l2_capability cap;
        if (ioctl(devFd, VIDIOC_QUERYCAP, &cap) == 0)
        {
            if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
            {
                CaptureDeviceInfo deviceInfo = {};
                strncpy(deviceInfo.friendlyName,
                        reinterpret_cast<const char *>(cap.card),
                        sizeof(deviceInfo.friendlyName) - 1);
                deviceInfo.friendlyName[sizeof(deviceInfo.friendlyName) - 1] = '\0';
                devices.push_back(deviceInfo);
            }
        }

        close(devFd);
    }

    return devices;
}

// ---------------------------------------------------------------------------
// ReleaseFrame / saveFrameAsJPEG
// ---------------------------------------------------------------------------

void ReleaseFrame(FrameData &frame)
{
    if (frame.rgbData)
    {
        delete[] frame.rgbData;
        frame.rgbData = nullptr;
        frame.size    = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Save a frame as a JPEG image using the STB library
// https://github.com/nothings/stb/blob/master/stb_image_write.h
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"
void saveFrameAsJPEG(const unsigned char *data, int width, int height,
                      const std::string &filename)
{
    if (stbi_write_jpg(filename.c_str(), width, height, 3, data, 90))
        std::cout << "Saved frame to " << filename << std::endl;
    else
        std::cerr << "Error saving frame as JPEG." << std::endl;
}
///////////////////////////////////////////////////////////////////////////////
