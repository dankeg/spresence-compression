#ifndef CAMERA_YUV_H
#define CAMERA_YUV_H

#include <stdint.h>
#include <stddef.h>

// Forward declarations
class CameraClass;
class SDClass;
class IFileSystem;

// Convert YUV422 interleaved data to separate Y, U, V channel files
// Uses IFileSystem interface for file operations
int convertYuv422ToSeparateChannels(
    const uint8_t* yuv422_data,
    size_t width,
    size_t height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    IFileSystem* filesystem
);

class CamImage;

// Convert JPEG image to separate Y, U, V channel files
// Uses IFileSystem interface for file operations
int convertJpegToSeparateChannels(
    CamImage& jpeg_img,
    size_t* out_width,
    size_t* out_height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    IFileSystem* filesystem
);

// Backward compatibility: Wrapper functions that accept SDClass*
// These create a temporary IFileSystem wrapper and call the main functions
// For new code, prefer using IFileSystem* directly
int convertYuv422ToSeparateChannels(
    const uint8_t* yuv422_data,
    size_t width,
    size_t height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    SDClass* sd_card
);

int convertJpegToSeparateChannels(
    CamImage& jpeg_img,
    size_t* out_width,
    size_t* out_height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    SDClass* sd_card
);

#endif // CAMERA_YUV_H

