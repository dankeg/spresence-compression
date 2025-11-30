#include "flash_icer_compression.h"
#include "flash_wavelet.h"
#include "flash_partition.h"
#include "icer_compression.h"
#include "memory_monitor.h"
#include "filesystem_interface.h"
#include "spresence_sd_filesystem.h"
#include <SDHCI.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <stdio.h>
#include <Arduino.h>  // For Serial progress reporting

extern "C" {
#include "icer.h"
}

// Packet comparison function (same as in icer_color.c)
static inline int comp_packet(const void *a, const void *b) {
    if (((icer_packet_context *)a)->priority == ((icer_packet_context *)b)->priority) {
        // prioritize based on subband type given same priority
        return (((icer_packet_context *)a)->subband_type > ((icer_packet_context *)b)->subband_type) ? 1 :
               (((icer_packet_context *)a)->subband_type < ((icer_packet_context *)b)->subband_type) ? -1 : 0;
    }
    return (((icer_packet_context *)a)->priority > ((icer_packet_context *)b)->priority) ? -1 : 1;
}

// Flash write callback implementation (same as in icer_compression.cpp)
static size_t icer_flash_write_callback_impl(void* context, const void* data, size_t size) {
    IFile* flash_file = static_cast<IFile*>(context);
    if (!flash_file || !flash_file->isOpen()) {
        return 0;
    }
    size_t written = flash_file->write(static_cast<const uint8_t*>(data), size);
    return written;
}

// GNSS RAM allocation wrappers (consistent with icer_compression.cpp)
#ifdef __arm__
#include <arch/chip/gnssram.h>
static bool gnss_ram_available = false;

// Set GNSS RAM availability (called from main.cpp after initialization)
// This must be called separately for flash_icer_compression.cpp
// Note: This is separate from icer_compression.cpp's setGnssRamAvailable
// because they are in different compilation units with separate static variables
void setGnssRamAvailable_flash(bool available) {
    gnss_ram_available = available;
}

static void* gnss_malloc(size_t size) {
    if (gnss_ram_available) {
        void* ptr = up_gnssram_malloc(size);
        if (ptr) return ptr;
        // Fallback to main RAM if GNSS RAM allocation fails
    }
    return malloc(size);
}

static void gnss_free(void* ptr) {
    if (ptr) {
        if (gnss_ram_available) {
            up_gnssram_free(ptr);
        } else {
            free(ptr);
        }
    }
}
#else
#define gnss_malloc malloc
#define gnss_free free
void setGnssRamAvailable_flash(bool available) { (void)available; } // No-op for non-ARM
#endif

// Flash-based ICER compression for large images (e.g., 720p)
// Complete pipeline with minimal RAM usage, maintaining 100% ICER compatibility
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
    bool channels_pre_transformed) {
    
    IcerCompressionResult result = {NULL, 0, false, 0, NULL};
    
    Serial.println("  ICER Flash Compression: Starting...");
    
    if (!filesystem || !y_flash_file || !u_flash_file || !v_flash_file || !output_flash_file) {
        Serial.println("  ICER Flash Compression: ERROR - Invalid parameters");
        result.error_code = -200;
        return result;
    }
    
    // Allocate ICER buffers (in GNSS RAM if available)
    Serial.println("  ICER Flash Compression: Allocating buffers...");
    int alloc_result = allocateIcerBuffers();
    if (alloc_result != 0) {
        Serial.print("  ICER Flash Compression: ERROR - Buffer allocation failed: ");
        Serial.println(alloc_result);
        result.error_code = -120 - alloc_result;
        return result;
    }
    
    // Initialize ICER
    Serial.println("  ICER Flash Compression: Initializing ICER...");
    static bool icer_initialized = false;
    if (!icer_initialized) {
        int init_result = icer_init();
        if (init_result != 0) {
            Serial.print("  ICER Flash Compression: ERROR - ICER init failed: ");
            Serial.println(init_result);
            freeIcerBuffers();
            result.error_code = init_result;
            return result;
        }
        icer_initialized = true;
    }
    
    // Temporary files for transformed channels
    const char* y_transformed_file = "_y_transformed.tmp";
    const char* u_transformed_file = "_u_transformed.tmp";
    const char* v_transformed_file = "_v_transformed.tmp";
    
    // Step 1: Apply wavelet transform to each channel (if not pre-transformed)
    if (!channels_pre_transformed) {
        Serial.println("  ICER Flash Compression: Step 1 - Wavelet transform...");
        // Transform Y channel
        Serial.println("    Transforming Y channel...");
        int transform_result = streamingWaveletTransform(
            filesystem, y_flash_file, y_transformed_file,
            width, height, stages, filter_type
        );
        if (transform_result != 0) {
            Serial.print("    ERROR: Y channel transform failed: ");
            Serial.println(transform_result);
            freeIcerBuffers();
            result.error_code = -201 - transform_result;
            return result;
        }
        
        // Transform U channel
        Serial.println("    Transforming U channel...");
        transform_result = streamingWaveletTransform(
            filesystem, u_flash_file, u_transformed_file,
            width, height, stages, filter_type
        );
        if (transform_result != 0) {
            Serial.print("    ERROR: U channel transform failed: ");
            Serial.println(transform_result);
            freeIcerBuffers();
            filesystem->remove(y_transformed_file);
            result.error_code = -201 - transform_result;
            return result;
        }
        
        // Transform V channel
        Serial.println("    Transforming V channel...");
        transform_result = streamingWaveletTransform(
            filesystem, v_flash_file, v_transformed_file,
            width, height, stages, filter_type
        );
        if (transform_result != 0) {
            Serial.print("    ERROR: V channel transform failed: ");
            Serial.println(transform_result);
            freeIcerBuffers();
            filesystem->remove(y_transformed_file);
            filesystem->remove(u_transformed_file);
            result.error_code = -201 - transform_result;
            return result;
        }
        Serial.println("  Step 1 complete: Wavelet transform finished");
    } else {
        // Channels are already transformed, use input files directly
        Serial.println("  ICER Flash Compression: Channels pre-transformed, skipping Step 1");
        y_transformed_file = y_flash_file;
        u_transformed_file = u_flash_file;
        v_transformed_file = v_flash_file;
    }
    
    // Step 2: Calculate LL mean values (needed for ICER)
    Serial.println("  ICER Flash Compression: Step 2 - Calculating LL mean values...");
    // We need to read the LL subband from the transformed image
    size_t ll_w = icer_get_dim_n_low_stages(width, stages);
    size_t ll_h = icer_get_dim_n_low_stages(height, stages);
    
    // Allocate buffer for LL subband (small, fits in RAM)
    size_t ll_size = ll_w * ll_h * sizeof(uint16_t);
    uint16_t* ll_buffer = (uint16_t*)malloc(ll_size);
    if (!ll_buffer) {
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
        result.error_code = -202;
        return result;
    }
    
    uint16_t ll_mean[ICER_CHANNEL_MAX + 1];
    
    // Read LL subband and calculate mean for each channel
    for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
        const char* channel_name = (chan == ICER_CHANNEL_Y) ? "Y" : (chan == ICER_CHANNEL_U) ? "U" : "V";
        Serial.print("    Calculating LL mean for channel ");
        Serial.print(channel_name);
        Serial.println("...");
        
        const char* channel_file = (chan == ICER_CHANNEL_Y) ? y_transformed_file :
                                   (chan == ICER_CHANNEL_U) ? u_transformed_file : v_transformed_file;
        
        IFile* chan_file = filesystem->open(channel_file, FILE_READ);
        if (!chan_file) {
            free(ll_buffer);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -203;
            return result;
        }
        
        // Read LL subband (top-left region of transformed image)
        // LL subband is at position (0, 0) with dimensions ll_w x ll_h
        uint64_t sum = 0;
        for (size_t row = 0; row < ll_h; row++) {
            size_t file_pos = row * width * sizeof(uint16_t);
            chan_file->seek(file_pos);
            size_t bytes_read = chan_file->read((uint8_t*)ll_buffer + row * ll_w * sizeof(uint16_t),
                                               ll_w * sizeof(uint16_t));
            if (bytes_read != ll_w * sizeof(uint16_t)) {
                chan_file->close();
                delete chan_file;
                free(ll_buffer);
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -204;
                return result;
            }
        }
        chan_file->close();
        delete chan_file;
        
        // Calculate mean
        for (size_t i = 0; i < ll_w * ll_h; i++) {
            sum += ll_buffer[i];
        }
        ll_mean[chan] = sum / (ll_w * ll_h);
        Serial.print("      Channel ");
        Serial.print(channel_name);
        Serial.print(" LL mean: ");
        Serial.println(ll_mean[chan]);
        
        if (ll_mean[chan] > INT16_MAX) {
            free(ll_buffer);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = ICER_INTEGER_OVERFLOW;
            return result;
        }
    }
    
    free(ll_buffer);
    Serial.println("  Step 2 complete: LL mean values calculated");
    
    // Step 2.5: Subtract LL mean from LL subband and convert to sign-magnitude
    Serial.println("  ICER Flash Compression: Step 2.5 - Subtracting LL mean and converting to sign-magnitude...");
    // This must be done before compression
    // We need to:
    // 1. Subtract mean from LL subband (in-place in flash)
    // 2. Convert entire image to sign-magnitude (in-place in flash)
    
    // Process each channel
    for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
        const char* channel_name = (chan == ICER_CHANNEL_Y) ? "Y" : (chan == ICER_CHANNEL_U) ? "U" : "V";
        Serial.print("    Processing channel ");
        Serial.print(channel_name);
        Serial.println("...");
        
        const char* channel_file = (chan == ICER_CHANNEL_Y) ? y_transformed_file :
                                   (chan == ICER_CHANNEL_U) ? u_transformed_file : v_transformed_file;
        
        IFile* chan_file = filesystem->open(channel_file, FILE_READ);
        if (!chan_file) {
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -203;
            return result;
        }
        
        // Read entire image into buffer for processing
        // This is necessary for sign-magnitude conversion
        // For 720p: 1280 * 720 * 2 = 1,843,200 bytes (~1.76 MB) - TOO LARGE!
        // We need to process in chunks or do it during partition read
        
        // ALTERNATIVE: Do sign-magnitude conversion during partition read
        // For now, we'll process the LL subband subtraction, and do sign-magnitude during partition
        
        // Subtract mean from LL subband (read, modify, write back)
        size_t ll_buffer_size = ll_w * ll_h * sizeof(uint16_t);
        uint16_t* ll_buffer = (uint16_t*)malloc(ll_buffer_size);
        if (!ll_buffer) {
            chan_file->close();
            delete chan_file;
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -202;
            return result;
        }
        
        // Read LL subband
        for (size_t row = 0; row < ll_h; row++) {
            size_t file_pos = row * width * sizeof(uint16_t);
            chan_file->seek(file_pos);
            size_t bytes_read = chan_file->read((uint8_t*)ll_buffer + row * ll_w * sizeof(uint16_t),
                                               ll_w * sizeof(uint16_t));
            if (bytes_read != ll_w * sizeof(uint16_t)) {
                free(ll_buffer);
                chan_file->close();
                delete chan_file;
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -204;
                return result;
            }
        }
        chan_file->close();
        delete chan_file;
        
        // Subtract mean from LL subband
        int16_t* signed_pixel = (int16_t*)ll_buffer;
        for (size_t i = 0; i < ll_w * ll_h; i++) {
            signed_pixel[i] = (int16_t)(signed_pixel[i] - (int16_t)ll_mean[chan]);
        }
        
        // Write LL subband back
        IFile* chan_file_write_ll = filesystem->open(channel_file, FILE_WRITE);
        if (!chan_file_write_ll) {
            free(ll_buffer);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -205;
            return result;
        }
        
        for (size_t row = 0; row < ll_h; row++) {
            size_t file_pos = row * width * sizeof(uint16_t);
            chan_file_write_ll->seek(file_pos);
            size_t bytes_written = chan_file_write_ll->write(
                (uint8_t*)ll_buffer + row * ll_w * sizeof(uint16_t),
                ll_w * sizeof(uint16_t)
            );
            if (bytes_written != ll_w * sizeof(uint16_t)) {
                free(ll_buffer);
                chan_file_write_ll->close();
                delete chan_file_write_ll;
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -206;
                return result;
            }
        }
        chan_file_write_ll->close();
        delete chan_file_write_ll;
        free(ll_buffer);
        
        // Convert entire image to sign-magnitude format
        // We need to do this in-place in flash
        // Process row-by-row to minimize RAM
        // Use a temporary file approach: read from original, convert, write to temp, then replace
        const char* temp_convert_file = "_temp_convert.tmp";
        filesystem->remove(temp_convert_file);
        
        IFile* chan_file_read = filesystem->open(channel_file, FILE_READ);
        IFile* chan_file_write = filesystem->open(temp_convert_file, FILE_WRITE);
        
        if (!chan_file_read || !chan_file_write) {
            if (chan_file_read) { chan_file_read->close(); delete chan_file_read; }
            if (chan_file_write) { chan_file_write->close(); delete chan_file_write; }
            filesystem->remove(temp_convert_file);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -207;
            return result;
        }
        
        size_t row_size = width * sizeof(uint16_t);
        uint16_t* row_buffer = (uint16_t*)malloc(row_size);
        if (!row_buffer) {
            chan_file_read->close();
            delete chan_file_read;
            chan_file_write->close();
            delete chan_file_write;
            filesystem->remove(temp_convert_file);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -208;
            return result;
        }
        
        // Read, convert, write row-by-row
        unsigned long convert_start_time = millis();
        for (size_t row = 0; row < height; row++) {
            // Report progress every 50 rows or every 2 seconds
            if (row % 50 == 0 || (millis() - convert_start_time) > 2000) {
                int progress_percent = (int)((row * 100) / height);
                Serial.print("      Sign-magnitude conversion: ");
                Serial.print(progress_percent);
                Serial.print("% (row ");
                Serial.print(row);
                Serial.print(" of ");
                Serial.print(height);
                Serial.println(")");
                convert_start_time = millis();
            }
            
            size_t bytes_read = chan_file_read->read((uint8_t*)row_buffer, row_size);
            if (bytes_read != row_size) {
                free(row_buffer);
                chan_file_read->close();
                delete chan_file_read;
                chan_file_write->close();
                delete chan_file_write;
                filesystem->remove(temp_convert_file);
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -209;
                return result;
            }
            
            // Convert row to sign-magnitude using exact ICER function
            icer_to_sign_magnitude_int16(row_buffer, width);
            
            // Write converted row to temp file
            size_t bytes_written = chan_file_write->write((uint8_t*)row_buffer, row_size);
            if (bytes_written != row_size) {
                free(row_buffer);
                chan_file_read->close();
                delete chan_file_read;
                chan_file_write->close();
                delete chan_file_write;
                filesystem->remove(temp_convert_file);
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -210;
                return result;
            }
        }
        
        free(row_buffer);
        chan_file_read->close();
        delete chan_file_read;
        chan_file_write->close();
        delete chan_file_write;
        
        // Replace original file with converted file
        // Copy temp file back to original location (SD card doesn't support rename)
        filesystem->remove(channel_file);
        IFile* temp_read = filesystem->open(temp_convert_file, FILE_READ);
        IFile* orig_write = filesystem->open(channel_file, FILE_WRITE);
        if (!temp_read || !orig_write) {
            if (temp_read) { temp_read->close(); delete temp_read; }
            if (orig_write) { orig_write->close(); delete orig_write; }
            filesystem->remove(temp_convert_file);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -211;
            return result;
        }
        
        // Copy temp file back to original
        size_t total_size = width * height * sizeof(uint16_t);
        size_t copy_buffer_size = 4096;  // 4 KB chunks
        uint8_t* copy_buffer = (uint8_t*)malloc(copy_buffer_size);
        if (!copy_buffer) {
            temp_read->close();
            delete temp_read;
            orig_write->close();
            delete orig_write;
            filesystem->remove(temp_convert_file);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -212;
            return result;
        }
        
        size_t remaining = total_size;
        while (remaining > 0) {
            size_t to_read = (remaining > copy_buffer_size) ? copy_buffer_size : remaining;
            size_t bytes_read = temp_read->read(copy_buffer, to_read);
            if (bytes_read != to_read) {
                free(copy_buffer);
                temp_read->close();
                delete temp_read;
                orig_write->close();
                delete orig_write;
                filesystem->remove(temp_convert_file);
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -213;
                return result;
            }
            size_t bytes_written = orig_write->write(copy_buffer, bytes_read);
            if (bytes_written != bytes_read) {
                free(copy_buffer);
                temp_read->close();
                delete temp_read;
                orig_write->close();
                delete orig_write;
                filesystem->remove(temp_convert_file);
                freeIcerBuffers();
                if (!channels_pre_transformed) {
                    filesystem->remove(y_transformed_file);
                    filesystem->remove(u_transformed_file);
                    filesystem->remove(v_transformed_file);
                }
                result.error_code = -214;
                return result;
            }
            remaining -= bytes_read;
        }
        
        free(copy_buffer);
        temp_read->close();
        delete temp_read;
        orig_write->close();
        delete orig_write;
        filesystem->remove(temp_convert_file);
    }
    Serial.println("  Step 2.5 complete: LL mean subtraction and sign-magnitude conversion finished");
    
    // Step 3: Prepare ICER output structure
    Serial.println("  ICER Flash Compression: Step 3 - Preparing ICER output structure...");
    size_t pixel_count = width * height;
    size_t byte_quota = target_size;
    if (byte_quota == 0) {
        // Lossless compression
        if (pixel_count > SIZE_MAX / 6) {
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -102;
            return result;
        }
        byte_quota = pixel_count * 6;  // 3 channels * 2 bytes (uint16_t)
    }
    
    // Allocate datastream buffer (in GNSS RAM if available)
    // CRITICAL ISSUE: All segments must remain in the buffer simultaneously because
    // icer_rearrange_segments_16 stores pointers into the buffer. During rearrange,
    // segments are read from these buffer locations and written to flash.
    //
    // Buffer requirements:
    // 1. Segment headers: ICER_MAX_PACKETS_16 * sizeof(icer_image_segment_typedef) = 800 * 32 = 25.6 KB
    // 2. All segment compressed data: Must fit in buffer simultaneously
    //    - For lossless compression, total compressed size can be large
    //    - With 800 segments, even 500 bytes/segment = 400 KB total
    //    - Worst case: segments could be several KB each = several MB total
    //
    // PROBLEM: The original code tried to allocate byte_quota (7.3 MB for lossless),
    // which exceeds available memory. We need a buffer large enough for all segments
    // but within memory constraints.
    //
    // SOLUTION: Use a large buffer (512 KB - 1 MB) and hope it's sufficient.
    // If not, compression will fail with BYTE_QUOTA_EXCEEDED, which is better than
    // allocation failure. For very large images, lossless compression may not be feasible.
    //
    // CRITICAL: icer_init_output_struct requires byte_quota <= buf_len for flash streaming.
    // We use effective_byte_quota = min(byte_quota, buffer_size) to pass this check.
    // size_allocated is set to effective_byte_quota, which limits how much can be stored.
    const size_t MAX_DATASTREAM_BUFFER_SIZE = 400 * 1024;  // 400 KB - increased to handle full compression (matches target_size)
    size_t buffer_size = MAX_DATASTREAM_BUFFER_SIZE;
    
    // If byte_quota is smaller than our buffer, use byte_quota + safety margin
    // Otherwise, cap at MAX_DATASTREAM_BUFFER_SIZE
    if (byte_quota < MAX_DATASTREAM_BUFFER_SIZE - 512) {
        buffer_size = byte_quota + 512;
    }
    
    // CRITICAL: For flash streaming with large byte_quota, we need to ensure
    // byte_quota <= buffer_size for icer_init_output_struct check to pass.
    // However, size_allocated limits how much can be stored in the buffer.
    // If the total compressed size exceeds size_allocated, compression will fail.
    // This is a fundamental limitation: we cannot compress images that require
    // more buffer space than available.
    size_t effective_byte_quota = (byte_quota > buffer_size) ? buffer_size : byte_quota;
    
    uint8_t* datastream = (uint8_t*)gnss_malloc(buffer_size);
    if (!datastream) {
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
        result.error_code = -105;
        return result;
    }
    
    icer_output_data_buf_typedef output;
    output.rearrange_flash_write = NULL;
    output.rearrange_flash_context = NULL;
    output.rearrange_flash_offset = 0;
    output.channels_pre_transformed = 0;
    
    // Open output file for rearrange phase (flash streaming)
    filesystem->remove(output_flash_file);
    IFile* output_file = filesystem->open(output_flash_file, FILE_WRITE);
    if (!output_file) {
        gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
        result.error_code = -206;
        return result;
    }
    
    // Set up flash write callback for rearrange phase before calling icer_init_output_struct.
    // output_file is already a pointer, so we can use it directly
    IFile* output_file_ptr = output_file;
    output.rearrange_flash_write = icer_flash_write_callback_impl;
    output.rearrange_flash_context = output_file_ptr;
    output.rearrange_flash_offset = 0;
    
    // Initialize output structure (will check rearrange_flash_write to allow smaller buffer)
    // Use effective_byte_quota to pass the buffer size check, but the actual quota
    // is tracked separately for size reporting
    int init_result = icer_init_output_struct(&output, datastream, buffer_size, effective_byte_quota);
    if (init_result != ICER_RESULT_OK) {
        output_file->close();
        delete output_file;
        gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
        result.error_code = init_result;
        return result;
    }
    
    // Step 4: Process each subband using flash-based partition
    // This replicates the logic from icer_compress_image_yuv_uint16 but uses flash-based partition
    // Note: Sign-magnitude conversion was already done in Step 2.5
    
    // Create packet list (same as standard ICER)
    uint32_t priority = 0;
    uint32_t ind = 0;
    for (uint8_t curr_stage = 1; curr_stage <= stages; curr_stage++) {
        priority = icer_pow_uint(2, curr_stage);
        for (uint8_t lsb = 0; lsb < ICER_BITPLANES_TO_COMPRESS_16; lsb++) {
            for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
                if (chan == ICER_CHANNEL_Y) priority *= 2;
                
                // HL subband
                icer_packets_16[ind].subband_type = ICER_SUBBAND_HL;
                icer_packets_16[ind].decomp_level = curr_stage;
                icer_packets_16[ind].ll_mean_val = ll_mean[chan];
                icer_packets_16[ind].lsb = lsb;
                icer_packets_16[ind].priority = priority << lsb;
                icer_packets_16[ind].image_w = width;
                icer_packets_16[ind].image_h = height;
                icer_packets_16[ind].channel = chan;
                ind++;
                if (ind >= ICER_MAX_PACKETS_16) {
                    output_file->close();
                    delete output_file;
                    gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                    result.error_code = ICER_PACKET_COUNT_EXCEEDED;
                    return result;
                }
                
                // LH subband
                icer_packets_16[ind].subband_type = ICER_SUBBAND_LH;
                icer_packets_16[ind].decomp_level = curr_stage;
                icer_packets_16[ind].ll_mean_val = ll_mean[chan];
                icer_packets_16[ind].lsb = lsb;
                icer_packets_16[ind].priority = priority << lsb;
                icer_packets_16[ind].image_w = width;
                icer_packets_16[ind].image_h = height;
                icer_packets_16[ind].channel = chan;
                ind++;
                if (ind >= ICER_MAX_PACKETS_16) {
                    output_file->close();
                    delete output_file;
                    gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                    result.error_code = ICER_PACKET_COUNT_EXCEEDED;
                    return result;
                }
                
                // HH subband
                icer_packets_16[ind].subband_type = ICER_SUBBAND_HH;
                icer_packets_16[ind].decomp_level = curr_stage;
                icer_packets_16[ind].ll_mean_val = ll_mean[chan];
                icer_packets_16[ind].lsb = lsb;
                icer_packets_16[ind].priority = ((priority / 2) << lsb) + 1;
                icer_packets_16[ind].image_w = width;
                icer_packets_16[ind].image_h = height;
            icer_packets_16[ind].channel = chan;
            ind++;
            if (ind >= ICER_MAX_PACKETS_16) {
                    output_file->close();
                    delete output_file;
                    gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                    result.error_code = ICER_PACKET_COUNT_EXCEEDED;
                    return result;
                }
            }
        }
    }
    
    // LL subband (final stage)
    priority = icer_pow_uint(2, stages);
    for (uint8_t lsb = 0; lsb < ICER_BITPLANES_TO_COMPRESS_16; lsb++) {
        for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
            if (chan == ICER_CHANNEL_Y) priority *= 2;
            
            icer_packets_16[ind].subband_type = ICER_SUBBAND_LL;
            icer_packets_16[ind].decomp_level = stages;
            icer_packets_16[ind].ll_mean_val = ll_mean[chan];
            icer_packets_16[ind].lsb = lsb;
            icer_packets_16[ind].priority = (2 * priority) << lsb;
            icer_packets_16[ind].image_w = width;
            icer_packets_16[ind].image_h = height;
            icer_packets_16[ind].channel = chan;
            ind++;
            if (ind >= ICER_MAX_PACKETS_16) {
                    output_file->close();
                    delete output_file;
                    gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                    result.error_code = ICER_PACKET_COUNT_EXCEEDED;
                    return result;
                }
            }
        }
    
    // Sort packets by priority (same as standard ICER)
    Serial.print("    Sorting ");
    Serial.print(ind);
    Serial.println(" packets by priority...");
    qsort(icer_packets_16, ind, sizeof(icer_packet_context), comp_packet);
    
    // Initialize rearrange segments array
    Serial.println("    Initializing rearrange segments array...");
    for (int i = 0; i <= ICER_MAX_DECOMP_STAGES; i++) {
        for (int j = 0; j <= ICER_SUBBAND_MAX; j++) {
            for (int k = 0; k <= ICER_MAX_SEGMENTS; k++) {
                for (int lsb = 0; lsb < ICER_BITPLANES_TO_COMPRESS_16; lsb++) {
                    for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
                        icer_rearrange_segments_16[chan][i][j][lsb][k] = NULL;
                    }
                }
            }
        }
    }
    
    // Process each packet using flash-based partition
    Serial.println("  ICER Flash Compression: Step 4 - Processing partitions...");
    partition_param_typdef partition_params;
    size_t ll_w_sub, ll_h_sub;
    size_t file_offset;
    
    unsigned long partition_start_time = millis();
    for (size_t it = 0; it < ind; it++) {
        // Report progress every 10 packets or every 2 seconds
        if (it % 10 == 0 || (millis() - partition_start_time) > 2000) {
            int progress_percent = (int)((it * 100) / ind);
            Serial.print("    Partition progress: ");
            Serial.print(progress_percent);
            Serial.print("% (packet ");
            Serial.print(it);
            Serial.print(" of ");
            Serial.print(ind);
            Serial.println(")");
            partition_start_time = millis();
        }
        // Calculate subband dimensions and file offset
        if (icer_packets_16[it].subband_type == ICER_SUBBAND_LL) {
            ll_w_sub = icer_get_dim_n_low_stages(width, icer_packets_16[it].decomp_level);
            ll_h_sub = icer_get_dim_n_low_stages(height, icer_packets_16[it].decomp_level);
            file_offset = 0;  // LL subband starts at (0, 0)
        } else if (icer_packets_16[it].subband_type == ICER_SUBBAND_HL) {
            ll_w_sub = icer_get_dim_n_high_stages(width, icer_packets_16[it].decomp_level);
            ll_h_sub = icer_get_dim_n_low_stages(height, icer_packets_16[it].decomp_level);
            file_offset = icer_get_dim_n_low_stages(width, icer_packets_16[it].decomp_level) * sizeof(uint16_t);
        } else if (icer_packets_16[it].subband_type == ICER_SUBBAND_LH) {
            ll_w_sub = icer_get_dim_n_low_stages(width, icer_packets_16[it].decomp_level);
            ll_h_sub = icer_get_dim_n_high_stages(height, icer_packets_16[it].decomp_level);
            file_offset = icer_get_dim_n_low_stages(height, icer_packets_16[it].decomp_level) * width * sizeof(uint16_t);
        } else if (icer_packets_16[it].subband_type == ICER_SUBBAND_HH) {
            ll_w_sub = icer_get_dim_n_high_stages(width, icer_packets_16[it].decomp_level);
            ll_h_sub = icer_get_dim_n_high_stages(height, icer_packets_16[it].decomp_level);
            file_offset = (icer_get_dim_n_low_stages(height, icer_packets_16[it].decomp_level) * width +
                          icer_get_dim_n_low_stages(width, icer_packets_16[it].decomp_level)) * sizeof(uint16_t);
        } else {
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = ICER_FATAL_ERROR;
            return result;
        }
        
        // Select channel file
        const char* channel_file = (icer_packets_16[it].channel == ICER_CHANNEL_Y) ? y_transformed_file :
                                   (icer_packets_16[it].channel == ICER_CHANNEL_U) ? u_transformed_file : v_transformed_file;
        
        // Open channel file for reading
        IFile* channel_file_handle = filesystem->open(channel_file, FILE_READ);
        if (!channel_file_handle) {
            output_file->close();
            delete output_file;
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -207;
            return result;
        }
        
        // Generate partition parameters
        int res = icer_generate_partition_parameters(&partition_params, ll_w_sub, ll_h_sub, segments);
        if (res != ICER_RESULT_OK) {
            channel_file_handle->close();
            delete channel_file_handle;
            output_file->close();
            delete output_file;
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = res;
            return result;
        }
        
        // Use flash-based partition compression
        res = icer_compress_partition_uint16_flash(
            channel_file_handle,
            file_offset,
            &partition_params,
            width,  // rowstride (full image width)
            &(icer_packets_16[it]),
            &output,
            (const icer_image_segment_typedef **) icer_rearrange_segments_16[icer_packets_16[it].channel][icer_packets_16[it].decomp_level][icer_packets_16[it].subband_type][icer_packets_16[it].lsb]
        );
        
        channel_file_handle->close();
        delete channel_file_handle;
        
        if (res != ICER_RESULT_OK) {
            output_file->close();
            delete output_file;
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = res;
            return result;
        }
    }
    Serial.println("  Step 4 complete: All partitions processed");
    
    // Step 5: Rearrange segments (same as standard ICER)
    Serial.println("  ICER Flash Compression: Step 5 - Rearranging segments...");
    // This must happen AFTER all partitions are processed
    // The rearrange phase writes all segments in the correct order to the output file
    // This is critical for correct ICER output format
    // Segments are stored in the datastream buffer during partition compression
    // The rearrange phase writes them sequentially to the output file
    
    // Ensure output file is positioned at the beginning for sequential write
    IFile* output_file_for_rearrange = static_cast<IFile*>(output.rearrange_flash_context);
    if (output_file_for_rearrange && output_file_for_rearrange->isOpen()) {
        output_file_for_rearrange->seek(0);
    }
    
    size_t rearrange_offset = 0;
    size_t len;
    int use_flash = (output.rearrange_flash_write != NULL);
    
    // The rearrange phase iterates through all segments and writes them in order
    size_t segments_written = 0;
    unsigned long rearrange_start_time = millis();
    for (int k = 0; k <= ICER_MAX_SEGMENTS; k++) {
        for (int j = ICER_SUBBAND_MAX; j >= 0; j--) {
            for (int i = ICER_MAX_DECOMP_STAGES; i >= 0; i--) {
                for (int lsb = ICER_BITPLANES_TO_COMPRESS_16 - 1; lsb >= 0; lsb--) {
                    for (int chan = ICER_CHANNEL_MIN; chan <= ICER_CHANNEL_MAX; chan++) {
                        if (icer_rearrange_segments_16[chan][i][j][lsb][k] != NULL) {
                            segments_written++;
                            // Report progress every 50 segments or every 2 seconds
                            if (segments_written % 50 == 0 || (millis() - rearrange_start_time) > 2000) {
                                Serial.print("    Rearrange progress: ");
                                Serial.print(segments_written);
                                Serial.println(" segments written");
                                rearrange_start_time = millis();
                            }
                            len = icer_ceil_div_uint32(icer_rearrange_segments_16[chan][i][j][lsb][k]->data_length, 8) +
                                  sizeof(icer_image_segment_typedef);
                            icer_rearrange_segments_16[chan][i][j][lsb][k]->lsb_chan |= ICER_SET_CHANNEL_MACRO(chan);
                            
                            if (use_flash) {
                                // Write directly to flash via callback
                                size_t written = output.rearrange_flash_write(
                                    output.rearrange_flash_context,
                                    icer_rearrange_segments_16[chan][i][j][lsb][k],
                                    len
                                );
                                if (written != len) {
                                    // Flash write failed
                                    output_file->close();
                                    delete output_file;
                                    gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                                    result.error_code = ICER_FATAL_ERROR;
                                    return result;
                                }
                                rearrange_offset += len;
                            } else {
                                // RAM-based path (shouldn't happen with flash callback set)
                                // This is an error condition
                                output_file->close();
                                delete output_file;
                                gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
                                result.error_code = -220;
                                return result;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Update output size
    output.size_used = rearrange_offset;
    if (use_flash) {
        output.rearrange_flash_offset = rearrange_offset;
    }
    Serial.print("    Total segments written: ");
    Serial.print(segments_written);
    Serial.print(", Output size: ");
    Serial.print(rearrange_offset);
    Serial.println(" bytes");
    Serial.println("  Step 5 complete: Rearrange finished");
    
    // Close output file
    Serial.println("  ICER Flash Compression: Verifying output file...");
    output_file->close();
    delete output_file;
    
    // Verify output file size
    IFile* verify_file = filesystem->open(output_flash_file, FILE_READ);
    if (verify_file) {
        size_t file_size = verify_file->size();
        verify_file->close();
        delete verify_file;
        
        if (file_size == output.size_used) {
            // Success
            Serial.println("  ICER Flash Compression: SUCCESS - Output file verified");
            Serial.print("    Compressed size: ");
            Serial.print(output.size_used);
            Serial.print(" bytes (");
            Serial.print(output.size_used / 1024);
            Serial.println(" KB)");
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.compressed_size = output.size_used;
            result.flash_filename = output_flash_file;
            result.success = true;
            result.error_code = 0;
            return result;
        } else {
            // File size mismatch
            filesystem->remove(output_flash_file);
            gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
            result.error_code = -113;
            return result;
        }
    } else {
        // File not found
        gnss_free(datastream);
            freeIcerBuffers();
            if (!channels_pre_transformed) {
                filesystem->remove(y_transformed_file);
                filesystem->remove(u_transformed_file);
                filesystem->remove(v_transformed_file);
            }
        result.error_code = -114;
        return result;
    }
}

// Backward compatibility wrapper that accepts SDClass*
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
) {
    // Create temporary file system wrapper (doesn't take ownership)
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) {
        IcerCompressionResult result = {NULL, 0, false, -200, NULL};
        return result;
    }
    
    // Call interface-based function
    IcerCompressionResult result = compressYuvWithIcerFlash(fs, y_flash_file, u_flash_file, v_flash_file,
                                                           width, height, stages, filter_type, segments,
                                                           target_size, output_flash_file, channels_pre_transformed);
    
    // Clean up wrapper
    delete fs;
    
    return result;
}

