#include "flash_wavelet.h"
#include "filesystem_interface.h"
#include "spresence_sd_filesystem.h"
#include <SDHCI.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <Arduino.h>  // For Serial progress reporting

extern "C" {
#include "icer.h"
}

// GNSS RAM allocation wrappers (consistent with flash_icer_compression.cpp)
#ifdef __arm__
#include <arch/chip/gnssram.h>
static bool gnss_ram_available = false;

// Set GNSS RAM availability (called from main.cpp after initialization)
// This must be called separately for flash_wavelet.cpp
void setGnssRamAvailable_wavelet(bool available) {
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
void setGnssRamAvailable_wavelet(bool available) { (void)available; } // No-op for non-ARM
#endif

// Apply wavelet transform to image in flash using standard ICER algorithms
// This streams data to minimize RAM usage while maintaining 100% ICER compatibility
int streamingWaveletTransform(
    IFileSystem* filesystem,
    const char* input_flash_file,
    const char* output_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type) {
    
    if (!filesystem || !input_flash_file || !output_flash_file || width == 0 || height == 0) {
        return -1;
    }
    
    enum icer_filter_types filt = (enum icer_filter_types)filter_type;
    
    // Remove output file if it exists
    filesystem->remove(output_flash_file);
    
    // Open input file (kept open for duration, closed at end)
    IFile* input_file = filesystem->open(input_flash_file, FILE_READ);
    if (!input_file) {
        return -2;
    }
    
    // For each stage, we need to process the current image dimensions
    // After each stage, dimensions halve (low-pass subband)
    // After stage 0, only the LL subband (top-left) is processed for subsequent stages
    size_t current_w = width;
    size_t current_h = height;
    size_t ll_offset_x = 0;  // X offset of LL subband region (always 0, top-left)
    size_t ll_offset_y = 0;  // Y offset of LL subband region (always 0, top-left)
    
    // Temporary file for intermediate results (row-transformed)
    const char* temp_file = "_wavelet_temp.tmp";
    
    // Process each stage
    Serial.print("    Wavelet transform: Processing ");
    Serial.print(stages);
    Serial.println(" stages...");
    
    for (uint8_t stage = 0; stage < stages; stage++) {
        Serial.print("      Stage ");
        Serial.print(stage + 1);
        Serial.print(" of ");
        Serial.print(stages);
        Serial.print(" (dimensions: ");
        Serial.print(current_w);
        Serial.print("x");
        Serial.print(current_h);
        Serial.println(")...");
        
        // Stage 0: read from input_file, write to output_file
        // Subsequent stages: read LL subband from output_file (previous stage), write LL subband back
        
        const char* stage_input = (stage == 0) ? input_flash_file : output_flash_file;
        IFile* stage_in = filesystem->open(stage_input, FILE_READ);
        if (!stage_in) {
            input_file->close();
            delete input_file;
            return -3;
        }
        
        filesystem->remove(temp_file);
        IFile* temp_out = filesystem->open(temp_file, FILE_WRITE);
        if (!temp_out) {
            stage_in->close();
            delete stage_in;
            input_file->close();
            delete input_file;
            return -4;
        }
        
        // PHASE 1: Row-wise transform (streaming)
        // Read rows from LL subband region, transform, write to temp file
        Serial.println("        Phase 1: Row-wise transform...");
        size_t row_size = current_w * sizeof(uint16_t);
        uint16_t* row_buffer = (uint16_t*)malloc(row_size);
        if (!row_buffer) {
            temp_out->close();
            delete temp_out;
            stage_in->close();
            delete stage_in;
            input_file->close();
            delete input_file;
            return -5;
        }
        
        unsigned long row_start_time = millis();
        for (size_t row = 0; row < current_h; row++) {
            // Report progress every 50 rows or every 2 seconds
            if (row % 50 == 0 || (millis() - row_start_time) > 2000) {
                int progress_percent = (int)((row * 100) / current_h);
                Serial.print("          Row transform: ");
                Serial.print(progress_percent);
                Serial.print("% (row ");
                Serial.print(row);
                Serial.print(" of ");
                Serial.print(current_h);
                Serial.println(")");
                row_start_time = millis();
            }
            // Calculate file position: LL subband region starts at (ll_offset_x, ll_offset_y)
            // Row position: (ll_offset_y + row) * width + ll_offset_x
            size_t file_pos = (ll_offset_y + row) * width * sizeof(uint16_t) + ll_offset_x * sizeof(uint16_t);
            stage_in->seek(file_pos);
            
            // Read row from LL subband region
            size_t bytes_read = stage_in->read((uint8_t*)row_buffer, row_size);
            if (bytes_read != row_size) {
                free(row_buffer);
                temp_out->close();
                delete temp_out;
                stage_in->close();
                delete stage_in;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -6;
            }
            
            // Apply row-wise transform using exact ICER function
            int res = icer_wavelet_transform_1d_uint16(row_buffer, current_w, 1, filt);
            if (res != ICER_RESULT_OK) {
                free(row_buffer);
                temp_out->close();
                delete temp_out;
                stage_in->close();
                delete stage_in;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -7;
            }
            
            // Write transformed row to temp file (compact, no rowstride)
            size_t bytes_written = temp_out->write((uint8_t*)row_buffer, row_size);
            if (bytes_written != row_size) {
                free(row_buffer);
                temp_out->close();
                delete temp_out;
                stage_in->close();
                delete stage_in;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -8;
            }
        }
        
        free(row_buffer);
        temp_out->close();
        delete temp_out;
        stage_in->close();
        delete stage_in;
        Serial.println("        Phase 1 complete: Row-wise transform finished");
        
        // PHASE 2: Column-wise transform (streaming)
        Serial.println("        Phase 2: Column-wise transform...");
        // Read columns from temp file, transform, write to output file
        IFile* temp_in = filesystem->open(temp_file, FILE_READ);
        if (!temp_in) {
            input_file->close();
            delete input_file;
            filesystem->remove(temp_file);
            return -10;
        }
        
        // For stage 0, create new output file
        // For subsequent stages, we need to read existing output, update LL subband, write back
        const char* stage_output_file = (stage == 0) ? output_flash_file : "_wavelet_stage_temp.tmp";
        if (stage == 0) {
            filesystem->remove(output_flash_file);
        } else {
            filesystem->remove(stage_output_file);
        }
        IFile* stage_out = filesystem->open(stage_output_file, FILE_WRITE);
        if (!stage_out) {
            temp_in->close();
            delete temp_in;
            input_file->close();
            delete input_file;
            filesystem->remove(temp_file);
            return -9;
        }
        
        // For stage 0, initialize output file with full image size
        // For subsequent stages, copy existing output file first, then update LL subband
        if (stage == 0) {
            // Initialize output file with zeros (full image size)
            Serial.println("        Initializing output file...");
            // EDGE CASE: Check for integer overflow in total_size calculation
            if (width > SIZE_MAX / height || (width * height) > SIZE_MAX / sizeof(uint16_t)) {
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                if (stage > 0) {
                    filesystem->remove(stage_output_file);
                }
                return -23;  // Integer overflow in total_size calculation
            }
            size_t total_size = width * height * sizeof(uint16_t);
            uint16_t zero = 0;
            // Use chunked writes for better performance
            size_t chunk_size = 4096;  // 4 KB chunks
            uint8_t* zero_chunk = (uint8_t*)malloc(chunk_size);
            if (zero_chunk) {
                memset(zero_chunk, 0, chunk_size);
                size_t remaining = total_size;
                while (remaining > 0) {
                    size_t to_write = (remaining > chunk_size) ? chunk_size : remaining;
                    stage_out->write(zero_chunk, to_write);
                    remaining -= to_write;
                }
                free(zero_chunk);
            } else {
                // Fallback: write zeros one at a time (slower)
                for (size_t i = 0; i < total_size; i += sizeof(uint16_t)) {
                    stage_out->write((uint8_t*)&zero, sizeof(uint16_t));
                }
            }
            stage_out->seek(0);  // Reset to beginning
            Serial.println("        Output file initialized");
        } else {
            // Copy existing output file to temp, then we'll update LL subband region
            IFile* existing_out = filesystem->open(output_flash_file, FILE_READ);
            if (!existing_out) {
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                filesystem->remove(stage_output_file);
                return -15;
            }
            // EDGE CASE: Check for integer overflow in total_size calculation
            if (width > SIZE_MAX / height || (width * height) > SIZE_MAX / sizeof(uint16_t)) {
                existing_out->close();
                delete existing_out;
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                filesystem->remove(stage_output_file);
                return -24;  // Integer overflow in total_size calculation
            }
            size_t total_size = width * height * sizeof(uint16_t);
            size_t copy_buffer_size = 4096;
            uint8_t* copy_buffer = (uint8_t*)malloc(copy_buffer_size);
            if (!copy_buffer) {
                existing_out->close();
                delete existing_out;
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                filesystem->remove(stage_output_file);
                return -16;
            }
            size_t remaining = total_size;
            while (remaining > 0) {
                size_t to_read = (remaining > copy_buffer_size) ? copy_buffer_size : remaining;
                size_t bytes_read = existing_out->read(copy_buffer, to_read);
                if (bytes_read != to_read) {
                    free(copy_buffer);
                    existing_out->close();
                    delete existing_out;
                    temp_in->close();
                    delete temp_in;
                    stage_out->close();
                    delete stage_out;
                    input_file->close();
                    delete input_file;
                    filesystem->remove(temp_file);
                    filesystem->remove(stage_output_file);
                    return -17;
                }
                size_t bytes_written = stage_out->write(copy_buffer, bytes_read);
                if (bytes_written != bytes_read) {
                    free(copy_buffer);
                    existing_out->close();
                    delete existing_out;
                    temp_in->close();
                    delete temp_in;
                    stage_out->close();
                    delete stage_out;
                    input_file->close();
                    delete input_file;
                    filesystem->remove(temp_file);
                    filesystem->remove(stage_output_file);
                    return -18;
                }
                remaining -= bytes_read;
            }
            free(copy_buffer);
            existing_out->close();
            delete existing_out;
            stage_out->seek(0);  // Reset to beginning
        }
        
        // CRITICAL OPTIMIZATION: Buffer multiple columns at once to reduce random seeks
        // Target: Use up to 300 KB for column buffering to dramatically speed up column transform
        // Strategy: Process columns in batches, reading/writing sequentially instead of random access
        
        // Calculate optimal batch size based on available memory
        // Each column requires: current_h * sizeof(uint16_t) bytes
        // With 300 KB available, we can buffer: 300KB / (current_h * 2) columns
        // But we need to leave some headroom, so target ~280 KB for buffering
        const size_t MAX_BUFFER_SIZE = 150 * 1024;  // 150 KB max for column buffer (reduced for memory optimization)
        
        // EDGE CASE: Check for integer overflow in col_size calculation
        // If current_h * sizeof(uint16_t) would overflow, we can't proceed
        if (current_h > SIZE_MAX / sizeof(uint16_t)) {
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
            return -19;  // Integer overflow in col_size calculation
        }
        size_t col_size = current_h * sizeof(uint16_t);  // Size of one column
        
        // EDGE CASE: If a single column exceeds MAX_BUFFER_SIZE, we can't buffer even one column
        // This would happen with extremely tall images (current_h > 140,000)
        // In this case, we must process one column at a time, which will exceed the buffer limit
        // but is the only option. The allocation will fail gracefully if memory is insufficient.
        size_t max_cols_per_batch;
        if (col_size > MAX_BUFFER_SIZE) {
            // Single column exceeds buffer limit - must process one at a time
            // This will allocate col_size bytes, which exceeds MAX_BUFFER_SIZE but is necessary
            max_cols_per_batch = 1;
        } else {
            max_cols_per_batch = MAX_BUFFER_SIZE / col_size;
        }
        
        // Ensure we buffer at least 1 column, but cap at reasonable maximum
        // For very small images, we might buffer all columns at once
        // For large images, we'll process in batches
        if (max_cols_per_batch < 1) max_cols_per_batch = 1;
        if (max_cols_per_batch > current_w) max_cols_per_batch = current_w;
        
        // For very large batches, cap at 200 columns to avoid excessive memory usage
        // This still gives us massive speedup while staying well under 300 KB
        if (max_cols_per_batch > 200) max_cols_per_batch = 200;
        
        size_t batch_size = max_cols_per_batch;
        
        // EDGE CASE: Check for integer overflow in buffer size calculation
        // If batch_size * col_size would overflow, we need to reduce batch_size
        // Note: col_size is guaranteed to be > 0 from earlier checks (height == 0 checked at start)
        if (col_size == 0 || batch_size > SIZE_MAX / col_size) {
            // Integer overflow would occur - reduce batch_size to maximum safe value
            batch_size = SIZE_MAX / col_size;
            if (batch_size < 1) batch_size = 1;
            if (batch_size > current_w) batch_size = current_w;
        }
        
        size_t actual_buffer_size = batch_size * col_size;
        
        Serial.print("        Buffering ");
        Serial.print(batch_size);
        Serial.print(" columns at once (");
        Serial.print(actual_buffer_size / 1024);
        Serial.println(" KB buffer)");
        
        // Allocate column buffer for batch processing (in GNSS RAM if available)
        uint16_t* col_buffer_batch = (uint16_t*)gnss_malloc(actual_buffer_size);
        if (!col_buffer_batch) {
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
            return -11;
        }
        
        // Process columns in batches
        unsigned long col_start_time = millis();
        for (size_t col_start = 0; col_start < current_w; col_start += batch_size) {
            size_t cols_in_batch = (col_start + batch_size > current_w) ? (current_w - col_start) : batch_size;
            
            // Report progress
            if (col_start % (batch_size * 4) == 0 || (millis() - col_start_time) > 2000) {
                int progress_percent = (int)((col_start * 100) / current_w);
                Serial.print("          Column transform: ");
                Serial.print(progress_percent);
                Serial.print("% (column ");
                Serial.print(col_start);
                Serial.print(" of ");
                Serial.print(current_w);
                Serial.print(", batch of ");
                Serial.print(cols_in_batch);
                Serial.println(")");
                col_start_time = millis();
            }
            
            // PHASE 2A: Read all columns in batch sequentially (row by row)
            // This is much faster than random seeks - we read entire rows from temp file
            for (size_t row = 0; row < current_h; row++) {
                // Read one row worth of data for all columns in batch
                // Temp file is stored row-major: row 0 has all columns, row 1 has all columns, etc.
                // EDGE CASE: Check for integer overflow in file position calculation
                // Must check multiplication overflow BEFORE computing row_offset
                if (current_w > 0 && row > SIZE_MAX / current_w) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -15;  // Integer overflow in row_offset calculation
                }
                size_t row_offset = row * current_w;
                size_t col_offset = col_start;
                if (row_offset > SIZE_MAX / sizeof(uint16_t) || 
                    col_offset > SIZE_MAX / sizeof(uint16_t) ||
                    (row_offset * sizeof(uint16_t)) > SIZE_MAX - (col_offset * sizeof(uint16_t))) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -15;  // Integer overflow in file position calculation
                }
                size_t temp_file_pos = row_offset * sizeof(uint16_t) + col_offset * sizeof(uint16_t);
                temp_in->seek(temp_file_pos);
                
                // Read all columns in batch for this row (sequential read - very fast!)
                // EDGE CASE: Verify buffer bounds - ensure we don't read past allocated buffer
                // Buffer size: batch_size * current_h elements
                // Read position: row * batch_size + cols_in_batch - 1 must be < batch_size * current_h
                // Since row < current_h and cols_in_batch <= batch_size, this is always safe
                // But check for integer overflow in pointer offset calculation
                // Note: batch_size is guaranteed to be >= 1 from earlier checks
                if (batch_size == 0 || row > SIZE_MAX / batch_size || (row * batch_size) > SIZE_MAX - cols_in_batch) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -17;  // Integer overflow in buffer offset calculation
                }
                size_t bytes_to_read = cols_in_batch * sizeof(uint16_t);
                size_t buffer_offset = row * batch_size;
                size_t bytes_read = temp_in->read((uint8_t*)(col_buffer_batch + buffer_offset), bytes_to_read);
                if (bytes_read != bytes_to_read) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -12;
                }
            }
            
            // PHASE 2B: Transform all columns in batch
            // Columns are stored with stride = batch_size (interleaved)
            // Row 0: [col0, col1, col2, ..., colN-1]
            // Row 1: [col0, col1, col2, ..., colN-1]
            // etc.
            // So column col_idx starts at col_buffer_batch + col_idx, with stride = batch_size
            for (size_t col_idx = 0; col_idx < cols_in_batch; col_idx++) {
                // Get pointer to this column (stored with stride = batch_size)
                // Column col_idx: elements at col_buffer_batch[col_idx], col_buffer_batch[col_idx + batch_size], ...
                uint16_t* col_ptr = col_buffer_batch + col_idx;
                
                // Apply column-wise transform using exact ICER function
                // Stride is batch_size because columns are interleaved in memory
                // This is correct: ICER will access col_ptr[0], col_ptr[batch_size], col_ptr[2*batch_size], etc.
                int res = icer_wavelet_transform_1d_uint16(col_ptr, current_h, batch_size, filt);
                if (res != ICER_RESULT_OK) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -13;
                }
            }
            
            // PHASE 2C: Write all columns in batch sequentially (row by row)
            // Write to output file using full image width as rowstride
            for (size_t row = 0; row < current_h; row++) {
                // Calculate position in output file for first column in batch
                // EDGE CASE: Check for integer overflow in file position calculation
                // Check addition overflow first
                if (ll_offset_y > SIZE_MAX - row) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -16;  // Integer overflow in row_offset calculation
                }
                size_t row_offset = ll_offset_y + row;
                size_t col_offset = ll_offset_x + col_start;
                // Check multiplication overflow before computing file position
                if (width > 0 && row_offset > SIZE_MAX / width) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -16;  // Integer overflow in row_offset * width calculation
                }
                if ((row_offset * width) > SIZE_MAX / sizeof(uint16_t) ||
                    col_offset > SIZE_MAX / sizeof(uint16_t) ||
                    (row_offset * width * sizeof(uint16_t)) > SIZE_MAX - (col_offset * sizeof(uint16_t))) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -16;  // Integer overflow in file position calculation
                }
                size_t file_pos = row_offset * width * sizeof(uint16_t) + col_offset * sizeof(uint16_t);
                stage_out->seek(file_pos);
                
                // Write all columns in batch for this row (sequential write - very fast!)
                // EDGE CASE: Verify buffer bounds - ensure we don't write past allocated buffer
                // Same bounds check as read operation
                // Note: batch_size is guaranteed to be >= 1 from earlier checks
                if (batch_size == 0 || row > SIZE_MAX / batch_size || (row * batch_size) > SIZE_MAX - cols_in_batch) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -18;  // Integer overflow in buffer offset calculation
                }
                size_t bytes_to_write = cols_in_batch * sizeof(uint16_t);
                size_t buffer_offset = row * batch_size;
                size_t bytes_written = stage_out->write((uint8_t*)(col_buffer_batch + buffer_offset), bytes_to_write);
                if (bytes_written != bytes_to_write) {
                    gnss_free(col_buffer_batch);
                temp_in->close();
                delete temp_in;
                stage_out->close();
                delete stage_out;
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                    if (stage > 0) {
                        filesystem->remove(stage_output_file);
                    }
                    return -14;
                }
            }
        }
        
        gnss_free(col_buffer_batch);
        temp_in->close();
        delete temp_in;
        stage_out->close();
        delete stage_out;
        Serial.println("        Phase 2 complete: Column-wise transform finished");
        
        // For subsequent stages, replace output file with updated version
        if (stage > 0) {
            Serial.println("        Copying updated output file...");
            filesystem->remove(output_flash_file);
            // Copy stage_output_file to output_flash_file
            IFile* temp_read = filesystem->open(stage_output_file, FILE_READ);
            IFile* final_write = filesystem->open(output_flash_file, FILE_WRITE);
            if (!temp_read || !final_write) {
                if (temp_read) { temp_read->close(); delete temp_read; }
                if (final_write) { final_write->close(); delete final_write; }
                filesystem->remove(stage_output_file);
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -19;
            }
            // EDGE CASE: Check for integer overflow in total_size calculation
            if (width > SIZE_MAX / height || (width * height) > SIZE_MAX / sizeof(uint16_t)) {
                temp_read->close();
                delete temp_read;
                final_write->close();
                delete final_write;
                filesystem->remove(stage_output_file);
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -25;  // Integer overflow in total_size calculation
            }
            size_t total_size = width * height * sizeof(uint16_t);
            size_t copy_buffer_size = 4096;
            uint8_t* copy_buffer = (uint8_t*)malloc(copy_buffer_size);
            if (!copy_buffer) {
                temp_read->close();
                delete temp_read;
                final_write->close();
                delete final_write;
                filesystem->remove(stage_output_file);
                input_file->close();
                delete input_file;
                filesystem->remove(temp_file);
                return -20;
            }
            size_t remaining = total_size;
            while (remaining > 0) {
                size_t to_read = (remaining > copy_buffer_size) ? copy_buffer_size : remaining;
                size_t bytes_read = temp_read->read(copy_buffer, to_read);
                if (bytes_read != to_read) {
                    free(copy_buffer);
                    temp_read->close();
                    delete temp_read;
                    final_write->close();
                    delete final_write;
                    filesystem->remove(stage_output_file);
                    input_file->close();
                    delete input_file;
                    filesystem->remove(temp_file);
                    return -21;
                }
                size_t bytes_written = final_write->write(copy_buffer, bytes_read);
                if (bytes_written != bytes_read) {
                    free(copy_buffer);
                    temp_read->close();
                    delete temp_read;
                    final_write->close();
                    delete final_write;
                    filesystem->remove(stage_output_file);
                    input_file->close();
                    delete input_file;
                    filesystem->remove(temp_file);
                    return -22;
                }
                remaining -= bytes_read;
            }
            free(copy_buffer);
            temp_read->close();
            delete temp_read;
            final_write->close();
            delete final_write;
            filesystem->remove(stage_output_file);
            Serial.println("        Output file updated");
        }
        
        // Clean up temp file
        filesystem->remove(temp_file);
        
        Serial.print("      Stage ");
        Serial.print(stage + 1);
        Serial.println(" complete");
        
        // Update dimensions and offset for next stage (LL subband is always at top-left, offset stays 0)
        // Dimensions halve for next stage's LL subband
        current_w = current_w / 2 + current_w % 2;
        current_h = current_h / 2 + current_h % 2;
        // ll_offset_x and ll_offset_y remain 0 (LL subband is always at top-left)
    }
    
    input_file->close();
    delete input_file;
    Serial.println("    Wavelet transform complete");
    
    return 0;
}

// Backward compatibility wrapper that accepts SDClass*
int streamingWaveletTransform(
    SDClass* sd_card,
    const char* input_flash_file,
    const char* output_flash_file,
    size_t width,
    size_t height,
    uint8_t stages,
    uint8_t filter_type
) {
    // Create temporary file system wrapper (doesn't take ownership)
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) {
        return -1;
    }
    
    // Call interface-based function
    int result = streamingWaveletTransform(fs, input_flash_file, output_flash_file,
                                          width, height, stages, filter_type);
    
    // Clean up wrapper
    delete fs;
    
    return result;
}
