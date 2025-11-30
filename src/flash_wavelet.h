#ifndef FLASH_WAVELET_H
#define FLASH_WAVELET_H

#include <stdint.h>
#include <stddef.h>

// Forward declarations
class IFile;
class SDClass;
class IFileSystem;

// Apply wavelet transform to an image stored in flash, using standard ICER algorithms
// This processes the image in a streaming fashion to minimize RAM usage
// Input: Flash file containing uint16_t image data (row-major, width * height * sizeof(uint16_t))
// Output: Flash file containing transformed image (same size, same format)
// The output is IDENTICAL to what standard ICER wavelet transform would produce
// 
// Algorithm:
// 1. Row-wise transform: Read rows from flash, transform, write to intermediate file
// 2. Column-wise transform: Read columns from intermediate file, transform, write to output
// 3. Repeat for multiple stages (each stage processes the low-pass subband)
//
// RAM usage: ~width * sizeof(uint16_t) for row buffer + ~height * sizeof(uint16_t) for column buffer
// For 720p: ~2.5 KB (row) + ~1.4 KB (column) = ~4 KB total
//
// Returns: 0 on success, negative error code on failure
int streamingWaveletTransform(
    IFileSystem* filesystem,
    const char* input_flash_file,
    const char* output_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type
);

// Set GNSS RAM availability for wavelet transform buffers
// This allows column buffering to use GNSS RAM instead of main RAM
// Must be called before streamingWaveletTransform if GNSS RAM is available
// Note: This is separate from other setGnssRamAvailable functions due to separate compilation units
void setGnssRamAvailable_wavelet(bool available);

// Backward compatibility: Wrapper function that accepts SDClass*
// This creates a temporary IFileSystem wrapper and calls the main function
// For new code, prefer using IFileSystem* directly
int streamingWaveletTransform(
    SDClass* sd_card,
    const char* input_flash_file,
    const char* output_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type
);

#endif // FLASH_WAVELET_H
