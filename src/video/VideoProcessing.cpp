#include "VideoProcessing.h"
#include <iostream>
#include <stdexcept>

VideoFrameBuffer AlignIndicesOnVideo(const VideoFrameBuffer &p_frames)
{
    VideoFrameBuffer alignedFrames;
    alignedFrames.reserve(p_frames.size());

    for (const auto &frame : p_frames)
    {
        StaticArray<SRGBColor, g_LED_COUNT> alignedFrame{};
        for (uint32_t col = 0; col < g_VIDEO_WIDTH; ++col)
        {
            for (uint32_t row = 0; row < g_VIDEO_HEIGHT; ++row)
            {
                uint32_t srcIndex = row * g_VIDEO_WIDTH + col;
                uint32_t physicalRow = (col % 2 == 1) ? row : (g_VIDEO_HEIGHT - 1 - row);
                uint32_t dstIndex = col * g_VIDEO_HEIGHT + physicalRow;
                alignedFrame[dstIndex] = frame[srcIndex];
            }
        }
        alignedFrames.push_back(alignedFrame);
    }
    return alignedFrames;
}

VideoFrameBuffer ProcessGreyscaleVideo(String p_path, size_t p_maxFileSize)
{
    IFStream file(p_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open video file: " + p_path);
    }
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    if (fileSize == 0 || fileSize > p_maxFileSize)
    {
        throw std::runtime_error("Invalid video file size: " + std::to_string(fileSize));
    }
    uint32_t frameCount = fileSize / g_LED_COUNT;
    if (fileSize % g_LED_COUNT != 0)
    {
        std::cerr << "Warning: video file size (" << fileSize << " bytes) is not a multiple of LED count (" << g_LED_COUNT << " bytes). The last " << (fileSize % g_LED_COUNT) << " bytes will be discarded." << std::endl;
    }
    file.seekg(0, std::ios::beg);

    VideoFrameBuffer frames(frameCount);

    for (uint32_t i = 0; i < frameCount; i++)
    {
        StaticByteArray<g_LED_COUNT> frameData{};
        file.read(reinterpret_cast<char *>(frameData.data()), frameData.size());
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        for (size_t j = 0; j < g_LED_COUNT; ++j)
        {
            uint8_t intensity = frameData[j];
            frame[j] = {intensity, intensity, intensity};
        }
        if (file.gcount() == static_cast<std::streamsize>(frameData.size()))
        {
            frames.push_back(frame);
        }
        else
        {
            if (!file.eof())
            {
                throw std::runtime_error("Error reading video file: " + p_path);
            }
            break;
        }
    }
    file.close();
    return AlignIndicesOnVideo(frames);
}

VideoFrameBuffer ProcessRGBVideo(String p_path, size_t p_maxFileSize)
{
    IFStream file(p_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open video file: " + p_path);
    }
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    if (fileSize == 0 || fileSize > p_maxFileSize)
    {
        throw std::runtime_error("Invalid video file size: " + std::to_string(fileSize));
    }
    if (fileSize % g_RGB_FRAME_SIZE != 0)
    {
        std::cerr << "Warning: RGB video file size (" << fileSize << " bytes) is not a multiple of frame size ("
                  << g_RGB_FRAME_SIZE << " bytes). The last " << (fileSize % g_RGB_FRAME_SIZE) << " bytes will be discarded." << std::endl;
    }
    uint32_t frameCount = static_cast<uint32_t>(fileSize / g_RGB_FRAME_SIZE);
    file.seekg(0, std::ios::beg);

    VideoFrameBuffer frames;
    frames.reserve(frameCount);

    for (uint32_t i = 0; i < frameCount; ++i)
    {
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        file.read(reinterpret_cast<char *>(frame.data()), g_RGB_FRAME_SIZE);

        if (file.gcount() == static_cast<std::streamsize>(g_RGB_FRAME_SIZE))
        {
            frames.push_back(frame);
        }
        else
        {
            if (!file.eof())
            {
                throw std::runtime_error("Error reading RGB video file: " + p_path);
            }
            break;
        }
    }
    file.close();
    return AlignIndicesOnVideo(frames);
}

VideoFrameBuffer CreateDummyVideoSlide(uint32_t p_frameCount)
{
    VideoFrameBuffer frames;
    frames.reserve(p_frameCount);
    for (uint32_t i = 0; i < p_frameCount; ++i)
    {
        uint32_t activeColumn = (i * g_VIDEO_WIDTH) / p_frameCount;

        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        frame.fill({0, 0, 0});

        for (uint32_t row = 0; row < g_VIDEO_HEIGHT; ++row)
        {
            uint32_t ledIndex = activeColumn * g_VIDEO_HEIGHT + row;
            frame[ledIndex] = {0xff, 0xff, 0xff};
        }

        frames.push_back(frame);
    }
    return AlignIndicesOnVideo(frames);
}

VideoFrameBuffer CreateDummyVideoPixelScan(uint32_t p_frameCount)
{
    VideoFrameBuffer frames;
    frames.reserve(p_frameCount);
    for (uint32_t i = 0; i < p_frameCount; ++i)
    {
        uint32_t pixelIndex = (i * g_LED_COUNT) / p_frameCount;
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        frame.fill({0, 0, 0});
        frame[pixelIndex] = {0xff, 0xff, 0xff};

        frames.push_back(frame);
    }
    return AlignIndicesOnVideo(frames);
}

VideoFrameBuffer LoadVideoBuffer(const String &p_path, EVideoFormat p_format, size_t p_maxFileSize)
{
    switch (p_format)
    {
    case EVideoFormat::Rgb:
        return ProcessRGBVideo(p_path, p_maxFileSize);
    case EVideoFormat::Greyscale:
        return ProcessGreyscaleVideo(p_path, p_maxFileSize);
    default:
        throw std::invalid_argument("Unknown video format");
    }
}
