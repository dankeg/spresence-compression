#ifndef FLASH_ICER_COMPRESSION_H
#define FLASH_ICER_COMPRESSION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "icer_compression.h"

// Forward declarations
class SDClass;
class IFileSystem;

// Flash-based ICER compression for large images (e.g., 720p)
// This function handles the complete pipeline with minimal RAM usage:
// 1. Streaming wavelet transform (if channels not pre-transformed)
// 2. Flash-based partition compression
// 3. Output to flash
//
// Input: Flash file paths for Y, U, V channels (uint16_t, row-major)
// Output: ICER compressed data in flash file
//
// This maintains 100% compatibility with standard ICER output
//
// Parameters:
// - filesystem: File system interface instance
// - y_flash_file: Flash file path for Y channel
// - u_flash_file: Flash file path for U channel
// - v_flash_file: Flash file path for V channel
// - width, height: Image dimensions
// - stages: Wavelet decomposition stages
// - filter_type: ICER filter type
// - segments: Error containment segments
// - target_size: Target compressed size (0 for lossless)
// - output_flash_file: Output flash file path for compressed data
// - channels_pre_transformed: If true, skip wavelet transform (channels already transformed)
//
// Returns: IcerCompressionResult with success status
//
// RAM Usage: ~50-100 KB (segment buffers + ICER buffers in GNSS RAM)
//
// Note: setGnssRamAvailable_flash() must be called before this function to enable GNSS RAM usage
// This is separate from icer_compression.cpp's setGnssRamAvailable() due to separate compilation units
void setGnssRamAvailable_flash(bool available);

IcerCompressionResult compressYuvWithIcerFlash(
    IFileSystem* filesystem,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type,
    uint8_t segments,
    size_t target_size,
    const char* output_flash_file,
    bool channels_pre_transformed
);

// Backward compatibility: Wrapper function that accepts SDClass*
// This creates a temporary IFileSystem wrapper and calls the main function
// For new code, prefer using IFileSystem* directly
IcerCompressionResult compressYuvWithIcerFlash(
    SDClass* sd_card,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type,
    uint8_t segments,
    size_t target_size,
    const char* output_flash_file,
    bool channels_pre_transformed
);

#endif // FLASH_ICER_COMPRESSION_H

