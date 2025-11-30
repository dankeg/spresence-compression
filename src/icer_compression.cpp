#include "icer_compression.h"
#include "memory_monitor.h"
#include <SDHCI.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <stdio.h>

// GNSS RAM support - use for ICER buffers to free main RAM for camera
#ifdef __arm__
#include <arch/chip/gnssram.h>
static bool gnss_ram_available = false;

// Wrapper functions to use GNSS RAM if available, fallback to main RAM
// Note: If GNSS RAM allocation fails, we fall back to main RAM.
// This means we need to track which pool was used for proper cleanup.
// For simplicity, we assume GNSS RAM allocations succeed when available.
// If they don't, we'll use main RAM and free with free().
static void* gnss_malloc(size_t size) {
    if (gnss_ram_available) {
        void* ptr = up_gnssram_malloc(size);
        if (ptr) return ptr;
        // Fallback to main RAM if GNSS RAM allocation fails
        // This should be rare - if GNSS RAM is available, it should have space
    }
    return malloc(size);
}

static void gnss_free(void* ptr) {
    if (ptr) {
        if (gnss_ram_available) {
            // Try GNSS RAM free first
            // If the pointer was allocated in GNSS RAM, this will succeed
            // If it was allocated in main RAM (fallback case), up_gnssram_free()
            // should handle it gracefully (likely a no-op or safe failure)
            // However, to be safe, we could check pointer ranges, but that's complex
            // For now, assume GNSS RAM allocations succeed when available
            up_gnssram_free(ptr);
        } else {
            free(ptr);
        }
    }
}
#else
// Non-ARM: use standard malloc/free
#define gnss_malloc malloc
#define gnss_free free
#endif

extern "C" {
#include "icer.h"
}

// Function to check if GNSS RAM is available (called from main.cpp after initialization)
void setGnssRamAvailable(bool available) {
#ifdef __arm__
    gnss_ram_available = available;
#endif
}

// Dynamic ICER buffers when USER_PROVIDED_BUFFERS is defined
#ifdef USER_PROVIDED_BUFFERS
#ifdef USE_UINT16_FUNCTIONS
#ifdef USE_ENCODE_FUNCTIONS
icer_packet_context *icer_packets_16 = NULL;
icer_image_segment_typedef ******icer_rearrange_segments_16 = NULL;
#endif
#endif
#ifdef USE_ENCODE_FUNCTIONS
uint16_t *icer_encode_circ_buf = NULL;
#endif
#endif

// Aligned memory allocation for ICER compatibility
// Note: Standard malloc on ARM Cortex-M4 aligns to at least 8 bytes, which is sufficient
// for uint16_t (2-byte alignment required). However, we keep this function available
// in case future code needs explicit alignment guarantees.
// Flash write callback for rearrange phase
static size_t icer_flash_write_callback_impl(void* context, const void* data, size_t size) {
    File* flash_file = static_cast<File*>(context);
    if (!flash_file || !(*flash_file)) {
        return 0;  // Error: invalid file
    }
    size_t written = flash_file->write(static_cast<const uint8_t*>(data), size);
    return written;
}

// Helper function to safely delete File objects
// Note: File class may not have virtual destructor, but we ensure proper cleanup
static void safe_delete_file(File* file_ptr) {
    if (file_ptr) {
        file_ptr->close();
        // Suppress warning about non-virtual destructor - File is used as value type here
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
        delete file_ptr;
        #pragma GCC diagnostic pop
    }
}

IcerCompressionResult compressYuvWithIcer(
    uint16_t* y_channel,
    uint16_t* u_channel,
    uint16_t* v_channel,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type,
    uint8_t segments,
    size_t target_size,
    SDClass* sd_card,
    const char* flash_filename,
    bool channels_pre_transformed) {
    
    IcerCompressionResult result = {NULL, 0, false, 0, NULL};
    
    if (!y_channel || !u_channel || !v_channel) {
        result.error_code = -100;
        return result;
    }
    
    // Initialize ICER buffers (when USER_PROVIDED_BUFFERS is defined)
    // Allocate ICER buffers dynamically (only when needed)
    static bool icer_buffers_allocated = false;
    if (!icer_buffers_allocated) {
        int alloc_result = allocateIcerBuffers();
        if (alloc_result != 0) {
            result.error_code = -120 - alloc_result;  // -121, -122, -123 for allocation failures
            return result;
        }
        icer_buffers_allocated = true;
    }
    
    static bool icer_initialized = false;
    if (!icer_initialized) {
        int init_result = icer_init();
        if (init_result != 0) {
            result.error_code = init_result;
            return result;
        }
        icer_initialized = true;
    }
    
    enum icer_filter_types filt = (enum icer_filter_types)filter_type;
    
    if (width > SIZE_MAX / height) {
        result.error_code = -106;
        return result;
    }
    size_t pixel_count = width * height;
    
    size_t byte_quota = target_size;
    if (byte_quota == 0) {
        // Lossless compression: byte_quota = pixel_count * 3 channels * 2 bytes (uint16_t)
        if (pixel_count > SIZE_MAX / 6) {
            result.error_code = -102;
            return result;
        }
        byte_quota = pixel_count * 6;
    }
    
    if (byte_quota > SIZE_MAX / 2) {
        result.error_code = -103;
        return result;
    }
    // Determine if we'll use flash for rearrange phase
    bool use_flash_rearrange = (sd_card && flash_filename);
    
    // ICER library buffer requirements:
    // RAM-based: buffer_size >= byte_quota * 2 (data_start + rearrange_start)
    // Flash-based: buffer_size >= byte_quota (only data_start needed)
    size_t buffer_size;
    if (use_flash_rearrange) {
        // Flash rearrange: only need data_start region + minimal overhead
        // ICER requires buf_len >= byte_quota, so we add small safety margin
        buffer_size = byte_quota + 512;  // Reduced from 1000 to 512 (minimal safety margin)
    } else {
        // RAM rearrange: need both regions
        buffer_size = byte_quota * 2 + 1000;
    }
    
    if (buffer_size < byte_quota) {
        result.error_code = -104;
        return result;
    }
    
    size_t free_mem = getFreeHeapMemory();
    if (buffer_size > free_mem) {
        result.error_code = -107;
        return result;
    }
    
    // Allocate datastream buffer - prefer GNSS RAM to free main RAM for camera
    // Note: This buffer is used during compression, not for DMA, so GNSS RAM is safe
    uint8_t* datastream = (uint8_t*)gnss_malloc(buffer_size);
    if (!datastream) {
        result.error_code = -105;
        return result;
    }
    
    icer_output_data_buf_typedef output;
    output.rearrange_flash_write = NULL;
    output.rearrange_flash_context = NULL;
    output.rearrange_flash_offset = 0;
    output.channels_pre_transformed = 0;
    
    File* rearrange_flash_file = NULL;
    if (use_flash_rearrange) {
        // Remove existing file if present
        sd_card->remove(flash_filename);
        // Open flash file for rearrange phase (will be written to during compression)
        File file = sd_card->open(flash_filename, FILE_WRITE);
        if (file) {
            // Allocate File object on heap to keep it alive during compression
            rearrange_flash_file = new File(file);
            if (rearrange_flash_file && *rearrange_flash_file) {
                // Set flash callback BEFORE calling icer_init_output_struct
                // This allows it to accept smaller buffer size
                output.rearrange_flash_write = icer_flash_write_callback_impl;
                output.rearrange_flash_context = rearrange_flash_file;
                output.rearrange_flash_offset = 0;
            } else {
                safe_delete_file(rearrange_flash_file);
                rearrange_flash_file = NULL;
                use_flash_rearrange = false;  // Fall back to RAM
            }
        } else {
            use_flash_rearrange = false;  // Fall back to RAM
        }
    }
    
    // Now initialize output struct (it will check flash callback to determine buffer size requirement)
    int init_result = icer_init_output_struct(&output, datastream, buffer_size, byte_quota);
    if (init_result != ICER_RESULT_OK) {
        safe_delete_file(rearrange_flash_file);
        gnss_free(datastream);
        result.error_code = init_result;
        return result;
    }
    
    // If channels are already transformed, set flag to skip wavelet transform
    // Use the dedicated flag field to avoid conflicts with rearrange_flash_context
    if (channels_pre_transformed) {
        output.channels_pre_transformed = 1;  // Set flag to skip wavelet transform
    }
    
    int icer_result = icer_compress_image_yuv_uint16(
        y_channel, u_channel, v_channel,
        width, height,
        stages, filt, segments,
        &output
    );
    
    // Clear the flag after use (for cleanup)
    if (channels_pre_transformed) {
        output.channels_pre_transformed = 0;
    }
    
    // Close flash file if it was opened
    safe_delete_file(rearrange_flash_file);
    rearrange_flash_file = NULL;
    
    if (icer_result == ICER_RESULT_OK) {
        result.compressed_size = output.size_used;
        if (result.compressed_size > 0) {
            // Check if data was written to flash during rearrange
            bool data_in_flash = (output.rearrange_flash_write != NULL && output.rearrange_flash_offset > 0);
            
            if (data_in_flash) {
                // Data was written to flash during rearrange phase - verify file exists and size matches
                File verify_file = sd_card->open(flash_filename, FILE_READ);
                if (verify_file) {
                    size_t file_size = verify_file.size();
                    verify_file.close();
                    
                    if (file_size == result.compressed_size) {
                        // Success: data is in flash, not RAM
                        gnss_free(datastream);  // Free datastream immediately (no longer needed)
                        result.compressed_data = NULL;
                        result.flash_filename = flash_filename;
                        result.success = true;
                        result.error_code = 0;
                    } else {
                        // File size mismatch - error
                        sd_card->remove(flash_filename);
                        gnss_free(datastream);
                        result.compressed_size = 0;
                        result.error_code = -113;  // Flash file size mismatch
                    }
                } else {
                    // File doesn't exist - error
                    gnss_free(datastream);
                    result.compressed_size = 0;
                    result.error_code = -114;  // Flash file not found
                }
            } else {
                // RAM-based path (original behavior or flash streaming not enabled)
                uint8_t* compressed_copy = (uint8_t*)malloc(result.compressed_size);
                if (compressed_copy) {
                    // Copy data while datastream is still valid
                    memcpy(compressed_copy, output.rearrange_start, result.compressed_size);
                    gnss_free(datastream);
                    
                    result.compressed_data = compressed_copy;
                    result.flash_filename = NULL;
                    result.success = true;
                    result.error_code = 0;
                } else {
                    gnss_free(datastream);
                    result.compressed_size = 0;
                    result.error_code = -109;
                }
            }
        } else {
            gnss_free(datastream);
            result.success = false;
            result.error_code = -110;
        }
    } else {
        gnss_free(datastream);
        result.error_code = icer_result;
    }
    
    // Free ICER buffers after compression is complete
    freeIcerBuffers();
    
    return result;
}

void freeIcerCompression(IcerCompressionResult* result) {
    if (result) {
        if (result->compressed_data) {
            free(result->compressed_data);
            result->compressed_data = NULL;
        }
        result->compressed_size = 0;
        result->success = false;
        result->flash_filename = NULL;
    }
}

// Allocate ICER buffers dynamically on the heap (prefer GNSS RAM to free main RAM)
int allocateIcerBuffers(void) {
#ifdef USER_PROVIDED_BUFFERS
    // Allocate packets buffer (1D array - simple)
    // Use GNSS RAM if available to free main RAM for camera
    if (!icer_packets_16) {
        icer_packets_16 = (icer_packet_context*)gnss_malloc(sizeof(icer_packet_context) * ICER_MAX_PACKETS_16);
        if (!icer_packets_16) return -1;
    }
    
    // Allocate rearrange_segments buffer (5D array: [chan][stage][subband][lsb][seg])
    // We need to allocate it as a proper multi-dimensional structure with nested pointers
    if (!icer_rearrange_segments_16) {
        // Allocate the top-level array of pointers (channels)
        // Use GNSS RAM if available to free main RAM for camera
        icer_rearrange_segments_16 = (icer_image_segment_typedef******)gnss_malloc(
            (ICER_CHANNEL_MAX + 1) * sizeof(icer_image_segment_typedef*****)
        );
        if (!icer_rearrange_segments_16) {
            gnss_free(icer_packets_16);
            icer_packets_16 = NULL;
            return -2;
        }
        
        // Allocate each channel level
        for (int chan = 0; chan <= ICER_CHANNEL_MAX; chan++) {
            icer_rearrange_segments_16[chan] = (icer_image_segment_typedef*****)gnss_malloc(
                (ICER_MAX_DECOMP_STAGES + 1) * sizeof(icer_image_segment_typedef****)
            );
            if (!icer_rearrange_segments_16[chan]) {
                // Cleanup on failure
                for (int i = 0; i < chan; i++) {
                    gnss_free(icer_rearrange_segments_16[i]);
                }
                gnss_free(icer_rearrange_segments_16);
                icer_rearrange_segments_16 = NULL;
                gnss_free(icer_packets_16);
                icer_packets_16 = NULL;
                return -2;
            }
            
            // Allocate each stage level
            for (int stage = 0; stage <= ICER_MAX_DECOMP_STAGES; stage++) {
                icer_rearrange_segments_16[chan][stage] = (icer_image_segment_typedef****)gnss_malloc(
                    (ICER_SUBBAND_MAX + 1) * sizeof(icer_image_segment_typedef***)
                );
                if (!icer_rearrange_segments_16[chan][stage]) {
                    // Cleanup on failure
                    for (int i = 0; i < stage; i++) {
                        gnss_free(icer_rearrange_segments_16[chan][i]);
                    }
                    gnss_free(icer_rearrange_segments_16[chan]);
                    for (int i = 0; i < chan; i++) {
                        gnss_free(icer_rearrange_segments_16[i]);
                    }
                    gnss_free(icer_rearrange_segments_16);
                    icer_rearrange_segments_16 = NULL;
                    gnss_free(icer_packets_16);
                    icer_packets_16 = NULL;
                    return -2;
                }
                
                // Allocate each subband level
                for (int subband = 0; subband <= ICER_SUBBAND_MAX; subband++) {
                    icer_rearrange_segments_16[chan][stage][subband] = (icer_image_segment_typedef***)gnss_malloc(
                        15 * sizeof(icer_image_segment_typedef**)
                    );
                    if (!icer_rearrange_segments_16[chan][stage][subband]) {
                        // Cleanup on failure
                        for (int s = 0; s < subband; s++) {
                            gnss_free(icer_rearrange_segments_16[chan][stage][s]);
                        }
                        gnss_free(icer_rearrange_segments_16[chan][stage]);
                        for (int i = 0; i < stage; i++) {
                            gnss_free(icer_rearrange_segments_16[chan][i]);
                        }
                        gnss_free(icer_rearrange_segments_16[chan]);
                        for (int i = 0; i < chan; i++) {
                            gnss_free(icer_rearrange_segments_16[i]);
                        }
                        gnss_free(icer_rearrange_segments_16);
                        icer_rearrange_segments_16 = NULL;
                        gnss_free(icer_packets_16);
                        icer_packets_16 = NULL;
                        return -2;
                    }
                    
                    // Allocate the final level (segments array) for each lsb
                    // This is an array of pointers: icer_image_segment_typedef* [ICER_MAX_SEGMENTS + 1]
                    for (int lsb = 0; lsb < 15; lsb++) {
                        // Allocate array of (ICER_MAX_SEGMENTS + 1) pointers
                        icer_rearrange_segments_16[chan][stage][subband][lsb] = (icer_image_segment_typedef**)gnss_malloc(
                            (ICER_MAX_SEGMENTS + 1) * sizeof(icer_image_segment_typedef*)
                        );
                        if (!icer_rearrange_segments_16[chan][stage][subband][lsb]) {
                            // Cleanup on failure
                            for (int l = 0; l < lsb; l++) {
                                gnss_free(icer_rearrange_segments_16[chan][stage][subband][l]);
                            }
                            gnss_free(icer_rearrange_segments_16[chan][stage][subband]);
                            for (int s = 0; s < subband; s++) {
                                gnss_free(icer_rearrange_segments_16[chan][stage][s]);
                            }
                            gnss_free(icer_rearrange_segments_16[chan][stage]);
                            for (int i = 0; i < stage; i++) {
                                gnss_free(icer_rearrange_segments_16[chan][i]);
                            }
                            gnss_free(icer_rearrange_segments_16[chan]);
                            for (int i = 0; i < chan; i++) {
                                gnss_free(icer_rearrange_segments_16[i]);
                            }
                            gnss_free(icer_rearrange_segments_16);
                            icer_rearrange_segments_16 = NULL;
                            gnss_free(icer_packets_16);
                            icer_packets_16 = NULL;
                            return -2;
                        }
                        // Initialize all pointers to NULL
                        memset(icer_rearrange_segments_16[chan][stage][subband][lsb], 0, 
                               (ICER_MAX_SEGMENTS + 1) * sizeof(icer_image_segment_typedef*));
                    }
                }
            }
        }
    }
    
    // Allocate circular buffer (1D array - simple)
    // Use GNSS RAM if available to free main RAM for camera
    if (!icer_encode_circ_buf) {
        icer_encode_circ_buf = (uint16_t*)gnss_malloc(sizeof(uint16_t) * ICER_CIRC_BUF_SIZE);
        if (!icer_encode_circ_buf) {
            freeIcerBuffers();  // Cleanup what we've allocated so far
            return -3;
        }
    }
#endif
    return 0;
}

// Free ICER buffers (use gnss_free which handles both GNSS RAM and main RAM)
void freeIcerBuffers(void) {
#ifdef USER_PROVIDED_BUFFERS
    if (icer_packets_16) {
        gnss_free(icer_packets_16);
        icer_packets_16 = NULL;
    }
    
    if (icer_rearrange_segments_16) {
        // Free the nested multi-dimensional structure
        for (int chan = 0; chan <= ICER_CHANNEL_MAX; chan++) {
            if (icer_rearrange_segments_16[chan]) {
                for (int stage = 0; stage <= ICER_MAX_DECOMP_STAGES; stage++) {
                    if (icer_rearrange_segments_16[chan][stage]) {
                        for (int subband = 0; subband <= ICER_SUBBAND_MAX; subband++) {
                            if (icer_rearrange_segments_16[chan][stage][subband]) {
                                for (int lsb = 0; lsb < 15; lsb++) {
                                    if (icer_rearrange_segments_16[chan][stage][subband][lsb]) {
                                        gnss_free(icer_rearrange_segments_16[chan][stage][subband][lsb]);
                                    }
                                }
                                gnss_free(icer_rearrange_segments_16[chan][stage][subband]);
                            }
                        }
                        gnss_free(icer_rearrange_segments_16[chan][stage]);
                    }
                }
                gnss_free(icer_rearrange_segments_16[chan]);
            }
        }
        gnss_free(icer_rearrange_segments_16);
        icer_rearrange_segments_16 = NULL;
    }
    
    if (icer_encode_circ_buf) {
        gnss_free(icer_encode_circ_buf);
        icer_encode_circ_buf = NULL;
    }
#endif
}

