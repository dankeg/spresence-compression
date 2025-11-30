#include "camera_yuv.h"
#include "filesystem_interface.h"
#include "spresence_sd_filesystem.h"
#include <Camera.h>
#include <SDHCI.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <Arduino.h>  // For Serial progress reporting

// Streaming JPEG decoder using Tiny JPEG Decompressor (tjpgd)
// This decoder reads JPEG from flash and writes RGB directly to flash, row by row
// Memory usage: Only a small working buffer (~3-4 KB) instead of full image buffer
#include "../lib/tjpgd/tjpgd.h"

// Context for streaming JPEG decode
static struct {
    IFile* jpeg_file;          // Input: JPEG file on flash
    IFile* rgb_file;           // Output: RGB file on flash
    int width;                 // Image width
    int height;                // Image height
    size_t row_size_bytes;     // Size of one RGB row (width * 3)
    int mcu_blocks_processed;  // Counter for progress reporting
    unsigned long last_progress_time;  // For periodic progress updates
} stream_decode_ctx = {NULL, NULL, 0, 0, 0, 0, 0};

// Input function for tjpgd - reads JPEG data from flash
// This is called by tjpgd whenever it needs more data to fill its internal buffer
// tjpgd maintains its own buffer and position, so we just read sequentially from the file
static size_t jpeg_input_func(JDEC* jd, uint8_t* buff, size_t nbyte) {
    (void)jd;  // Unused parameter - tjpgd passes the JDEC pointer but we use global context
    
    if (!stream_decode_ctx.jpeg_file || !stream_decode_ctx.jpeg_file->isOpen()) {
        return 0;  // Error: file not available
    }
    
    // Read from current file position (tjpgd calls this sequentially)
    return stream_decode_ctx.jpeg_file->read(buff, nbyte);
}

// Output function for tjpgd - writes RGB data directly to flash
// This is called for each MCU (Minimum Coded Unit) block decoded
// bitmap contains RGB888 data for the rectangle in row-major order
// The bitmap data is stored in jd->workbuf and contains exactly rect_width * rect_height * 3 bytes
// Data layout: Row 0 (pixels 0 to rect_width-1), Row 1, Row 2, etc.
static int jpeg_output_func(JDEC* jd, void* bitmap, JRECT* rect) {
    (void)jd;  // Unused but provided by tjpgd
    
    if (!stream_decode_ctx.rgb_file || !stream_decode_ctx.rgb_file->isOpen()) {
        return 0;  // Error: file not available
    }
    
    // bitmap contains RGB888 data for the rectangle (rect->left to rect->right, rect->top to rect->bottom)
    // The data is stored row by row, with each pixel as 3 bytes (R, G, B)
    // Total size: rect_width * rect_height * 3 bytes
    uint8_t* rgb_data = (uint8_t*)bitmap;
    int rect_width = rect->right - rect->left + 1;
    int rect_height = rect->bottom - rect->top + 1;
    
    // Validate rectangle bounds
    // Rectangle coordinates are inclusive: [left, right] and [top, bottom]
    // Valid pixel coordinates: [0, width-1] and [0, height-1]
    if (rect_width <= 0 || rect_height <= 0 || 
        rect->left < 0 || rect->top < 0 ||
        rect->right >= stream_decode_ctx.width || 
        rect->bottom >= stream_decode_ctx.height ||
        rect->left > rect->right || rect->top > rect->bottom) {
        return 0;  // Invalid rectangle
    }
    
    size_t rect_row_bytes = (size_t)rect_width * 3;  // RGB888 = 3 bytes per pixel
    
    // Write each row of the rectangle to flash
    for (int y = 0; y < rect_height; y++) {
        int row = rect->top + y;
        if (row >= stream_decode_ctx.height) {
            return 0;  // Row out of bounds (shouldn't happen with valid rect)
        }
        
        // Calculate offset in RGB file for this row segment
        size_t row_offset = (size_t)row * stream_decode_ctx.row_size_bytes;
        size_t pixel_offset = (size_t)rect->left * 3;  // 3 bytes per pixel (RGB)
        size_t file_offset = row_offset + pixel_offset;
        
        // Seek to the start of this row segment in the RGB file
        // Note: If file_offset is beyond current file size, the file system will extend the file
        // This is much faster than pre-allocating the entire file with zeros
        if (!stream_decode_ctx.rgb_file->seek(file_offset)) {
            return 0;  // Seek error (shouldn't happen with proper file system)
        }
        
        // Write the RGB data for this row segment
        // rgb_data is organized row by row: row y starts at rgb_data + y * rect_row_bytes
        size_t written = stream_decode_ctx.rgb_file->write(
            rgb_data + y * rect_row_bytes, 
            rect_row_bytes
        );
        if (written != rect_row_bytes) {
            return 0;  // Write error
        }
    }
    
    // Update progress counter
    stream_decode_ctx.mcu_blocks_processed++;
    
    // Report progress every 100 MCU blocks or every 2 seconds, whichever comes first
    unsigned long current_time = millis();
    if (stream_decode_ctx.mcu_blocks_processed % 100 == 0 || 
        (current_time - stream_decode_ctx.last_progress_time) > 2000) {
        // Estimate progress based on bottom row processed (MCU blocks are typically 8x8 or 16x16)
        // Use rect->bottom as a proxy for progress
        int progress_percent = 0;
        if (stream_decode_ctx.height > 0) {
            progress_percent = (int)((rect->bottom * 100) / stream_decode_ctx.height);
            if (progress_percent > 100) progress_percent = 100;
        }
        // Use minimal Serial output to avoid stack issues
        Serial.print("  JPEG decode: ~");
        Serial.print(progress_percent);
        Serial.print("% (");
        Serial.print(rect->bottom);
        Serial.print("/");
        Serial.print(stream_decode_ctx.height);
        Serial.println(")");
        stream_decode_ctx.last_progress_time = current_time;
    }
    
    // Flush periodically to reduce flash wear (every ~50 rows)
    // Use rect->bottom to determine when to flush
    if (rect->bottom > 0 && (rect->bottom % 50 == 0)) {
        stream_decode_ctx.rgb_file->flush();
    }
    
    return 1;  // Continue decoding
}

// Convert YUV422 interleaved to separate Y, U, V channels (scanline-by-scanline)
// YUV422 format: Y, U, Y, V, Y, U, Y, V... (2 bytes per pixel)
// ICER needs: separate Y, U, V channels as uint16_t (full resolution for Y, half for U/V)
// Note: YUV422 has chroma subsampling - U and V are at half horizontal resolution
int convertYuv422ToSeparateChannels(
    const uint8_t* yuv422_data,
    size_t width,
    size_t height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    IFileSystem* filesystem) {
    
    if (!yuv422_data || !filesystem || width == 0 || height == 0) {
        return -1;
    }
    
    // Remove existing files
    filesystem->remove(y_flash_file);
    filesystem->remove(u_flash_file);
    filesystem->remove(v_flash_file);
    
    // Open flash files for Y, U, V channels
    IFile* y_file = filesystem->open(y_flash_file, FILE_WRITE);
    IFile* u_file = filesystem->open(u_flash_file, FILE_WRITE);
    IFile* v_file = filesystem->open(v_flash_file, FILE_WRITE);
    
    if (!y_file || !u_file || !v_file) {
        if (y_file) { y_file->close(); delete y_file; }
        if (u_file) { u_file->close(); delete u_file; }
        if (v_file) { v_file->close(); delete v_file; }
        filesystem->remove(y_flash_file);
        filesystem->remove(u_flash_file);
        filesystem->remove(v_flash_file);
        return -2;
    }
    
    // Process scanline-by-scanline to minimize RAM
    // YUV422: 2 bytes per pixel (Y, U, Y, V pattern)
    // For ICER: we need full-resolution Y, and full-resolution U/V (upsampled from YUV422)
    size_t yuv422_scanline_size = width * 2;  // 2 bytes per pixel in YUV422
    
    // Allocate scanline buffers
    uint8_t* yuv422_scanline = (uint8_t*)malloc(yuv422_scanline_size);
    uint16_t* y_scanline = (uint16_t*)malloc(width * sizeof(uint16_t));
    uint16_t* u_scanline = (uint16_t*)malloc(width * sizeof(uint16_t));
    uint16_t* v_scanline = (uint16_t*)malloc(width * sizeof(uint16_t));
    
    if (!yuv422_scanline || !y_scanline || !u_scanline || !v_scanline) {
        if (yuv422_scanline) free(yuv422_scanline);
        if (y_scanline) free(y_scanline);
        if (u_scanline) free(u_scanline);
        if (v_scanline) free(v_scanline);
        y_file->close(); delete y_file;
        u_file->close(); delete u_file;
        v_file->close(); delete v_file;
        filesystem->remove(y_flash_file);
        filesystem->remove(u_flash_file);
        filesystem->remove(v_flash_file);
        return -3;
    }
    
    // Process each scanline
    // YUV422 format: Spresense uses YUYV format (Y0, U0, Y1, V0, Y2, U1, Y3, V1, ...)
    // 2 bytes per pixel, U and V are subsampled horizontally (shared between adjacent pixels)
    // Pattern: [Y0, U0], [Y1, V0], [Y2, U1], [Y3, V1], ...
    // Pixels: 0=(Y0,U0,V0), 1=(Y1,U0,V0), 2=(Y2,U1,V1), 3=(Y3,U1,V1), ...
    // Note: If Spresense uses UYVY format instead, swap U and V extraction logic
    // We need to upsample U and V to full width for ICER (duplicate for odd columns)
    for (size_t row = 0; row < height; row++) {
        // Read YUV422 scanline
        memcpy(yuv422_scanline, yuv422_data + row * yuv422_scanline_size, yuv422_scanline_size);
        
        // Convert YUV422 interleaved to separate Y, U, V
        for (size_t col = 0; col < width; col++) {
            size_t byte_idx = col * 2;
            
            // Y is at every even byte (full resolution)
            y_scanline[col] = (uint16_t)yuv422_scanline[byte_idx];
            
            // U and V are shared between pairs of pixels
            if (col % 2 == 0) {
                // Even columns (0, 2, 4...): U is at byte_idx+1, V is at byte_idx+3
                if (byte_idx + 1 < yuv422_scanline_size) {
                    u_scanline[col] = (uint16_t)yuv422_scanline[byte_idx + 1];  // U
                } else {
                    u_scanline[col] = 128;  // Default neutral U
                }
                if (byte_idx + 3 < yuv422_scanline_size) {
                    v_scanline[col] = (uint16_t)yuv422_scanline[byte_idx + 3];  // V
                } else {
                    // Last pixel in row, use previous V or default
                    v_scanline[col] = (col > 0) ? v_scanline[col - 1] : 128;
                }
            } else {
                // Odd columns (1, 3, 5...): share U and V from previous even pixel
                u_scanline[col] = u_scanline[col - 1];
                v_scanline[col] = v_scanline[col - 1];
            }
        }
        
        // Write scanlines to flash
        size_t scanline_yuv_size = width * sizeof(uint16_t);
        size_t y_written = y_file->write((uint8_t*)y_scanline, scanline_yuv_size);
        size_t u_written = u_file->write((uint8_t*)u_scanline, scanline_yuv_size);
        size_t v_written = v_file->write((uint8_t*)v_scanline, scanline_yuv_size);
        
        if (y_written != scanline_yuv_size || u_written != scanline_yuv_size || v_written != scanline_yuv_size) {
            free(yuv422_scanline);
            free(y_scanline);
            free(u_scanline);
            free(v_scanline);
            y_file->close(); delete y_file;
            u_file->close(); delete u_file;
            v_file->close(); delete v_file;
            filesystem->remove(y_flash_file);
            filesystem->remove(u_flash_file);
            filesystem->remove(v_flash_file);
            return -4;
        }
    }
    
    // Cleanup
    free(yuv422_scanline);
    free(y_scanline);
    free(u_scanline);
    free(v_scanline);
    
    y_file->close(); delete y_file;
    u_file->close(); delete u_file;
    v_file->close(); delete v_file;
    
    return 0;
}


// Convert RGB to YUV using ITU-R BT.601 standard formulas
// Y = 0.299*R + 0.587*G + 0.114*B
// U (Cb) = -0.168736*R - 0.331264*G + 0.5*B + 128
// V (Cr) = 0.5*R - 0.418688*G - 0.081312*B + 128
// Using integer arithmetic: multiply coefficients by 1000000, then divide by 1000000 for maximum accuracy
// Output: Y, U, V values in range [0, 255] stored as uint16_t (compatible with ICER)
static inline void rgb_to_yuv(uint8_t r, uint8_t g, uint8_t b, uint16_t* y, uint16_t* u, uint16_t* v) {
    // Y calculation: 0.299*R + 0.587*G + 0.114*B
    // Exact: (299000*R + 587000*G + 114000*B) / 1000000
    int32_t y_val = (299000L * (int32_t)r + 587000L * (int32_t)g + 114000L * (int32_t)b) / 1000000L;
    *y = (uint16_t)(y_val < 0 ? 0 : (y_val > 255 ? 255 : y_val));
    
    // U (Cb) calculation: -0.168736*R - 0.331264*G + 0.5*B + 128
    // Exact: (-168736*R - 331264*G + 500000*B) / 1000000 + 128
    int32_t u_val = (-168736L * (int32_t)r - 331264L * (int32_t)g + 500000L * (int32_t)b) / 1000000L + 128;
    *u = (uint16_t)(u_val < 0 ? 0 : (u_val > 255 ? 255 : u_val));
    
    // V (Cr) calculation: 0.5*R - 0.418688*G - 0.081312*B + 128
    // Exact: (500000*R - 418688*G - 81312*B) / 1000000 + 128
    int32_t v_val = (500000L * (int32_t)r - 418688L * (int32_t)g - 81312L * (int32_t)b) / 1000000L + 128;
    *v = (uint16_t)(v_val < 0 ? 0 : (v_val > 255 ? 255 : v_val));
}

// Convert JPEG image to separate Y, U, V channel files in flash
// This function uses streaming JPEG decoding (tjpgd) to minimize RAM usage
// 
// Output format (ICER-compatible):
//   - Each channel: row-major order, uint16_t per pixel
//   - Y, U, V values: [0, 255] range stored as uint16_t
//   - File size per channel: width * height * sizeof(uint16_t) bytes
//   - Format matches flash_icer_compression.cpp expectations exactly
//
// Peak memory utilization (for 720p = 1280x720):
//   - Step 2 (JPEG decode): ~3.7 KB (WORK_BUF_SIZE + JDEC struct)
//   - Step 4 (RGB to YUV): ~11.5 KB (scanline buffers)
//   - Overall peak: ~11.5 KB (during Step 4)
//
// This is a massive improvement over loading full image in RAM:
//   - Full 720p RGB: 1280 * 720 * 3 = 2,764,800 bytes ≈ 2.76 MB
//   - Our approach: ~11.5 KB (99.6% reduction!)
int convertJpegToSeparateChannels(
    CamImage& jpeg_img,
    size_t* out_width,
    size_t* out_height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    IFileSystem* filesystem
) {
    if (!jpeg_img.isAvailable() || !filesystem) {
        return -1;
    }

    // Get JPEG data from CamImage
    // CRITICAL: After Step 1, we no longer need the CamImage object or camera
    // The JPEG data is copied to flash, so the caller can free CamImage and end camera
    // to free up memory during the conversion process
    const uint8_t* jpeg_data = jpeg_img.getImgBuff();
    size_t jpeg_size = jpeg_img.getImgSize();
    if (!jpeg_data || jpeg_size == 0) {
        return -2;
    }

    // Step 1: Save JPEG to flash first (compressed, so small)
    Serial.println("  Step 1: Saving JPEG to flash...");
    // After this step, the JPEG data is in flash and we no longer need:
    // - The CamImage object (can be freed by caller)
    // - The camera (can be ended by caller to free memory)
    // All subsequent operations read from flash files only
    const char* temp_jpeg_file = "_temp_jpeg.tmp";
    filesystem->remove(temp_jpeg_file);
    IFile* jpeg_flash_file = filesystem->open(temp_jpeg_file, FILE_WRITE);
    if (!jpeg_flash_file) {
        Serial.println("  ERROR: Failed to open JPEG temp file for writing");
        return -3;
    }
    
    size_t jpeg_written = jpeg_flash_file->write(jpeg_data, jpeg_size);
    jpeg_flash_file->close();
    delete jpeg_flash_file;
    
    if (jpeg_written != jpeg_size) {
        Serial.print("  ERROR: Failed to write JPEG data (");
        Serial.print(jpeg_written);
        Serial.print(" of ");
        Serial.print(jpeg_size);
        Serial.println(" bytes)");
        filesystem->remove(temp_jpeg_file);
        return -4;
    }
    Serial.print("  Step 1 complete: Saved ");
    Serial.print(jpeg_size);
    Serial.println(" bytes to flash");
    
    // JPEG data is now safely in flash - caller can free CamImage and end camera here

    // Step 2: Decode JPEG directly to flash using streaming decoder (tjpgd)
    // This uses minimal RAM - only a small working buffer (~3-4 KB) instead of full image buffer
    const char* temp_rgb_file = "_temp_rgb.tmp";
    filesystem->remove(temp_rgb_file);
    IFile* rgb_flash_file = filesystem->open(temp_rgb_file, FILE_WRITE);
    if (!rgb_flash_file) {
        filesystem->remove(temp_jpeg_file);
        return -5;
    }
    
    // Open JPEG file for reading
    IFile* jpeg_file = filesystem->open(temp_jpeg_file, FILE_READ);
    if (!jpeg_file) {
        rgb_flash_file->close();
        delete rgb_flash_file;
        filesystem->remove(temp_jpeg_file);
        filesystem->remove(temp_rgb_file);
        return -6;
    }
    
    // Initialize streaming decode context
    stream_decode_ctx.jpeg_file = jpeg_file;
    stream_decode_ctx.rgb_file = rgb_flash_file;
    stream_decode_ctx.width = 0;
    stream_decode_ctx.height = 0;
    stream_decode_ctx.row_size_bytes = 0;
    stream_decode_ctx.mcu_blocks_processed = 0;
    stream_decode_ctx.last_progress_time = millis();
    
    // Allocate working buffer for tjpgd
    // This is the ONLY significant RAM allocation during JPEG decode - much smaller than full image buffer
    // tjpgd needs: input buffer (512 bytes) + huffman tables + quantization tables + MCU buffer + IDCT work buffer
    // For baseline JPEG (8-bit, 3 components): minimum ~3100 bytes, recommended ~3500 bytes for safety
    // Using 3500 bytes to ensure we have enough headroom for various JPEG configurations
    // Peak memory during Step 2: 3500 bytes (WORK_BUF_SIZE) + ~200 bytes (JDEC struct on stack) ≈ 3.7 KB
    const size_t WORK_BUF_SIZE = 3500;  // Safe size for baseline JPEG with 3 components
    void* work_buf = malloc(WORK_BUF_SIZE);
    if (!work_buf) {
        jpeg_file->close();
        delete jpeg_file;
        rgb_flash_file->close();
        delete rgb_flash_file;
        stream_decode_ctx.jpeg_file = NULL;
        stream_decode_ctx.rgb_file = NULL;
        filesystem->remove(temp_jpeg_file);
        filesystem->remove(temp_rgb_file);
        return -7;  // Memory allocation failed
    }
    
    // Prepare JPEG decoder
    Serial.println("  Step 2: Preparing JPEG decoder...");
    JDEC jdec;
    JRESULT jres = jd_prepare(&jdec, jpeg_input_func, work_buf, WORK_BUF_SIZE, NULL);
    if (jres != JDR_OK) {
        Serial.print("  ERROR: JPEG prepare failed with code ");
        Serial.println((int)jres);
        free(work_buf);
        jpeg_file->close();
        delete jpeg_file;
        rgb_flash_file->close();
        delete rgb_flash_file;
        stream_decode_ctx.jpeg_file = NULL;
        stream_decode_ctx.rgb_file = NULL;
        filesystem->remove(temp_jpeg_file);
        filesystem->remove(temp_rgb_file);
        return -8;  // JPEG prepare failed
    }
    
    // Get image dimensions
    int width = (int)jdec.width;
    int height = (int)jdec.height;
    Serial.print("  Image dimensions: ");
    Serial.print(width);
    Serial.print("x");
    Serial.println(height);
    
    if (width <= 0 || height <= 0) {
        free(work_buf);
        jpeg_file->close();
        delete jpeg_file;
        rgb_flash_file->close();
        delete rgb_flash_file;
        stream_decode_ctx.jpeg_file = NULL;
        stream_decode_ctx.rgb_file = NULL;
        filesystem->remove(temp_jpeg_file);
        filesystem->remove(temp_rgb_file);
        return -9;
    }
    
    // Set output dimensions
    if (out_width) {
        *out_width = (size_t)width;
    }
    if (out_height) {
        *out_height = (size_t)height;
    }
    
    // Update context
    stream_decode_ctx.width = width;
    stream_decode_ctx.height = height;
    stream_decode_ctx.row_size_bytes = (size_t)width * 3;  // RGB888 = 3 bytes per pixel
    
    // CRITICAL FIX: Skip pre-allocation for RGB file
    // Pre-allocating 2.76 MB by writing zeros is extremely slow (2,705 write operations)
    // Instead, we'll let the file grow naturally as MCU blocks are written
    // The file system will handle extending the file automatically
    // This is much faster and more reliable
    // Note: tjpgd writes MCU blocks in decode order (top-to-bottom, left-to-right),
    // so the file will grow sequentially, making this approach safe
    
    // CRITICAL: After jd_prepare, the file position is at the start of image data (after SOS marker)
    // jd_decomp will continue reading from this position via the input function
    // We must NOT reset the file position - jd_prepare already positioned us correctly
    // The input function will be called by jd_decomp to read sequentially from this point
    
    // Decompress JPEG - this will call jpeg_output_func for each MCU block
    // RGB data is written directly to flash, no full buffer in RAM!
    Serial.println("  Step 2: Decompressing JPEG to RGB (this may take a while)...");
    jres = jd_decomp(&jdec, jpeg_output_func, 0);  // scale = 0 means no scaling
    if (jres != JDR_OK) {
        Serial.print("  ERROR: JPEG decompress failed with code ");
        Serial.println((int)jres);
        free(work_buf);
        jpeg_file->close();
        delete jpeg_file;
        rgb_flash_file->close();
        delete rgb_flash_file;
        stream_decode_ctx.jpeg_file = NULL;
        stream_decode_ctx.rgb_file = NULL;
        filesystem->remove(temp_jpeg_file);
        filesystem->remove(temp_rgb_file);
        return -11;  // JPEG decompress failed
    }
    Serial.print("  Step 2 complete: Processed ");
    Serial.print(stream_decode_ctx.mcu_blocks_processed);
    Serial.println(" MCU blocks");
    
    // Flush any remaining data
    rgb_flash_file->flush();
    
    // CRITICAL: Clear static context pointers BEFORE closing files
    // This prevents any potential use-after-close issues
    stream_decode_ctx.jpeg_file = NULL;
    stream_decode_ctx.rgb_file = NULL;
    
    // Cleanup
    free(work_buf);
    jpeg_file->close();
    delete jpeg_file;
    rgb_flash_file->close();
    delete rgb_flash_file;
    
    // Small delay to ensure file system operations complete
    delay(50);
    
    filesystem->remove(temp_jpeg_file);
    
    // Additional delay before opening new files
    delay(50);
    
    // RGB data is now in temp_rgb_file in flash

    // Step 3: Open flash files for Y, U, V channels
    Serial.println("  Step 3: Opening Y, U, V channel files...");
    filesystem->remove(y_flash_file);
    filesystem->remove(u_flash_file);
    filesystem->remove(v_flash_file);

    // Step 4: Read RGB from flash and convert to YUV scanline-by-scanline
    Serial.println("  Step 4: Converting RGB to YUV (this may take a while)...");
    // All operations now use flash - minimal RAM usage
    // Memory: RGB scanline (width * 3) + Y scanline (width * 2) + U scanline (width * 2) + V scanline (width * 2)
    // For 720p: (1280 * 3) + (1280 * 2) + (1280 * 2) + (1280 * 2) = 3,840 + 2,560 + 2,560 + 2,560 = 11,520 bytes ≈ 11.25 KB
    // Peak memory during this step: 11,520 bytes (all scanline buffers allocated simultaneously)
    
    // CRITICAL REWRITE: Open all files at function scope, then destroy them one at a time
    // in separate scopes. The hard fault occurs when File destructors run during function return,
    // so we need to ensure they run one at a time in isolated scopes BEFORE the function returns.
    IFile* y_file = filesystem->open(y_flash_file, FILE_WRITE);
    IFile* u_file = filesystem->open(u_flash_file, FILE_WRITE);
    IFile* v_file = filesystem->open(v_flash_file, FILE_WRITE);

    {
        // Check if files opened successfully

        if (!y_file || !u_file || !v_file) {
        if (y_file) { y_file->close(); delete y_file; }
        if (u_file) { u_file->close(); delete u_file; }
        if (v_file) { v_file->close(); delete v_file; }
            filesystem->remove(temp_rgb_file);
        filesystem->remove(y_flash_file);
        filesystem->remove(u_flash_file);
        filesystem->remove(v_flash_file);
            return -12;
        }
    }  // End of check scope
    
    // Open RGB read file in its own scope
    IFile* rgb_read_file = nullptr;
    {
        rgb_read_file = filesystem->open(temp_rgb_file, FILE_READ);
        if (!rgb_read_file) {
        y_file->close(); delete y_file;
        u_file->close(); delete u_file;
        v_file->close(); delete v_file;
            filesystem->remove(temp_rgb_file);
        filesystem->remove(y_flash_file);
        filesystem->remove(u_flash_file);
        filesystem->remove(v_flash_file);
            return -13;
        }
    }  // End of rgb_read_file opening scope
    
    // All file operations happen in this scope
    {
        uint8_t* rgb_scanline = (uint8_t*)malloc((size_t)width * 3);
        uint16_t* y_scanline = (uint16_t*)malloc((size_t)width * sizeof(uint16_t));
        uint16_t* u_scanline = (uint16_t*)malloc((size_t)width * sizeof(uint16_t));
        uint16_t* v_scanline = (uint16_t*)malloc((size_t)width * sizeof(uint16_t));

        if (!rgb_scanline || !y_scanline || !u_scanline || !v_scanline) {
            if (rgb_scanline) free(rgb_scanline);
            if (y_scanline) free(y_scanline);
            if (u_scanline) free(u_scanline);
            if (v_scanline) free(v_scanline);
            rgb_read_file->close(); delete rgb_read_file;
                y_file->close(); delete y_file;
                u_file->close(); delete u_file;
                v_file->close(); delete v_file;
            filesystem->remove(temp_rgb_file);
                filesystem->remove(y_flash_file);
                filesystem->remove(u_flash_file);
                filesystem->remove(v_flash_file);
            return -14;
        }

        // Process RGB from flash, convert to YUV, write to flash
        // All operations use flash - peak RAM is only scanline buffers
        unsigned long rgb_convert_start_time = millis();
        for (int row = 0; row < height; row++) {
            // Report progress every 50 rows or every 2 seconds
            if (row % 50 == 0 || (millis() - rgb_convert_start_time) > 2000) {
                int progress_percent = (int)((row * 100) / height);
                Serial.print("  RGB->YUV progress: ");
                Serial.print(progress_percent);
                Serial.print("% (row ");
                Serial.print(row);
                Serial.print(" of ");
                Serial.print(height);
                Serial.println(")");
                rgb_convert_start_time = millis();  // Reset timer
            }
            // Read RGB scanline from flash
            size_t rgb_scanline_bytes = (size_t)width * 3;
            size_t rgb_read = rgb_read_file->read(rgb_scanline, rgb_scanline_bytes);
            if (rgb_read != rgb_scanline_bytes) {
                free(rgb_scanline);
                free(y_scanline);
                free(u_scanline);
                free(v_scanline);
                rgb_read_file->close(); delete rgb_read_file;
                y_file->close(); delete y_file;
                u_file->close(); delete u_file;
                v_file->close(); delete v_file;
                filesystem->remove(temp_rgb_file);
                filesystem->remove(y_flash_file);
                filesystem->remove(u_flash_file);
                filesystem->remove(v_flash_file);
                return -15;
            }
            
            // Convert RGB to YUV
            for (int col = 0; col < width; col++) {
                uint8_t r = rgb_scanline[col * 3 + 0];
                uint8_t g = rgb_scanline[col * 3 + 1];
                uint8_t b = rgb_scanline[col * 3 + 2];
                
                rgb_to_yuv(r, g, b, &y_scanline[col], &u_scanline[col], &v_scanline[col]);
            }

            // Write YUV scanlines to flash files
            size_t scanline_bytes = (size_t)width * sizeof(uint16_t);
            size_t y_written = y_file->write((uint8_t*)y_scanline, scanline_bytes);
            size_t u_written = u_file->write((uint8_t*)u_scanline, scanline_bytes);
            size_t v_written = v_file->write((uint8_t*)v_scanline, scanline_bytes);
            
            // Periodic flush every 100 rows to ensure data is written
            if (row > 0 && row % 100 == 0) {
                y_file->flush();
                u_file->flush();
                v_file->flush();
            }

            if (y_written != scanline_bytes || u_written != scanline_bytes || v_written != scanline_bytes) {
                free(rgb_scanline);
                free(y_scanline);
                free(u_scanline);
                free(v_scanline);
                rgb_read_file->close(); delete rgb_read_file;
                y_file->close(); delete y_file;
                u_file->close(); delete u_file;
                v_file->close(); delete v_file;
                filesystem->remove(temp_rgb_file);
                filesystem->remove(y_flash_file);
                filesystem->remove(u_flash_file);
                filesystem->remove(v_flash_file);
                return -16;
            }
        }
        
        // CRITICAL: Free buffers BEFORE closing files
        // This ensures buffers are freed while files are still valid
        free(rgb_scanline);
                    free(y_scanline);
                    free(u_scanline);
                    free(v_scanline);
    }  // End of file operations scope - buffers are freed
    
    // CRITICAL REWRITE: Close all files explicitly and wait for operations to complete
    // The hard fault occurs when File destructors run during function return.
    // By closing all files explicitly and adding delays, we ensure file system
    // operations complete before destructors run.
    
    // Close read file first
    rgb_read_file->close();
    delete rgb_read_file;
    delay(300);
    
    // Close write files one at a time with delays
    v_file->close();
    delete v_file;
    delay(300);
    
    u_file->close();
    delete u_file;
    delay(300);
    
    y_file->close();
    delete y_file;
    delay(300);
    
    // CRITICAL: All files are now explicitly closed
    // Wait for all file system operations to fully complete
    // This gives the file system time to finish all pending operations
    delay(500);
    
    Serial.println("  Step 4 complete: RGB to YUV conversion finished");
    
    // CRITICAL: Final delay before return to ensure all File destructors
    // can run safely. The destructors will be called when the function returns,
    // but since files are already closed, they should be no-ops.
    delay(300);
    
    // All IFile objects are now deleted
    // Do NOT remove temp_rgb_file here - it will be removed by the caller

    return 0;
}

// Backward compatibility wrappers that accept SDClass*
// These create a temporary IFileSystem wrapper and call the interface-based functions
int convertYuv422ToSeparateChannels(
    const uint8_t* yuv422_data,
    size_t width,
    size_t height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    SDClass* sd_card
) {
    // Create temporary file system wrapper (doesn't take ownership)
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) {
        return -1;
    }
    
    // Call interface-based function
    int result = convertYuv422ToSeparateChannels(yuv422_data, width, height,
                                                  y_flash_file, u_flash_file, v_flash_file,
                                                  fs);
    
    // Clean up wrapper
    delete fs;
    
    return result;
}

int convertJpegToSeparateChannels(
    CamImage& jpeg_img,
    size_t* out_width,
    size_t* out_height,
    const char* y_flash_file,
    const char* u_flash_file,
    const char* v_flash_file,
    SDClass* sd_card
) {
    // Create temporary file system wrapper (doesn't take ownership)
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) {
        return -1;
    }
    
    // Call interface-based function
    int result = convertJpegToSeparateChannels(jpeg_img, out_width, out_height,
                                                y_flash_file, u_flash_file, v_flash_file,
                                                fs);
    
    // Clean up wrapper
    delete fs;
    
    return result;
}

