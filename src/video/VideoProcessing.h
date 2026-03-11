#pragma once

#include "VideoConstants.h"

// Re-map row-major frame data to column-major with snake column indexing
VideoFrameBuffer AlignIndicesOnVideo(const VideoFrameBuffer &p_frames);

// Load raw video files
VideoFrameBuffer ProcessGreyscaleVideo(String p_path, size_t p_maxFileSize);
VideoFrameBuffer ProcessRGBVideo(String p_path, size_t p_maxFileSize);

// Dummy/test video generators
VideoFrameBuffer CreateDummyVideoSlide(uint32_t p_frameCount);
VideoFrameBuffer CreateDummyVideoPixelScan(uint32_t p_frameCount);

// High-level loader dispatching on format
VideoFrameBuffer LoadVideoBuffer(const String &p_path, EVideoFormat p_format,
                                 size_t p_maxFileSize = g_VIDEO_MAX_FILE_SIZE_DEFAULT);
