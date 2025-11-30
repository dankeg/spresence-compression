#include "flash_partition.h"
#include "filesystem_interface.h"
#include <SDHCI.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

// Flash-based partition compression - reads segments from flash on-demand
// Maintains 100% compatibility with standard ICER output
int icer_compress_partition_uint16_flash(
    IFile* flash_file,
    size_t file_offset,
    const partition_param_typdef *params,
    size_t rowstride,
    icer_packet_context *pkt_context,
    icer_output_data_buf_typedef *output_data,
    const icer_image_segment_typedef *segments_encoded[]) {
    
    if (!flash_file || !params || !pkt_context || !output_data || !segments_encoded) {
        return ICER_FATAL_ERROR;
    }
    
    int res;
    size_t segment_w, segment_h;
    uint16_t segment_num = 0;
    
    size_t partition_col_ind;
    size_t partition_row_ind = 0;
    
    icer_context_model_typedef context_model;
    icer_encoder_context_typedef context;
    icer_image_segment_typedef *seg;
    
    uint32_t data_in_bytes;
    
           // Calculate maximum segment size for buffer allocation
           // Segments can vary in size, so we allocate for the largest possible segment
           size_t max_segment_w = (size_t)(params->x_t + 1);  // Largest width (x_t or x_t+1)
           size_t max_segment_h = (size_t)(params->y_t + 1);  // Largest height (y_t or y_t+1)
           if (params->x_b > 0 && (size_t)(params->x_b + 1) > max_segment_w) {
               max_segment_w = (size_t)(params->x_b + 1);
           }
           if (params->y_b > 0 && (size_t)(params->y_b + 1) > max_segment_h) {
               max_segment_h = (size_t)(params->y_b + 1);
           }
    
    // CRITICAL OPTIMIZATION: Allocate buffer with segment width + padding instead of full rowstride
    // icer_compress_bitplane accesses neighbors: pos-1, pos+1 (horizontal) and pos-rowstride, pos+rowstride (vertical)
    // We need 1 pixel padding on each side for boundary access
    // This reduces buffer from max_segment_h * rowstride to (max_segment_h + 2) * (max_segment_w + 2)
    // For 1280x960 image: old = 161 * 1280 * 2 = 402 KB, new = 163 * 216 * 2 = 69 KB (83% reduction!)
    size_t padded_w = max_segment_w + 2;  // Left + right padding
    size_t padded_h = max_segment_h + 2;  // Top + bottom padding
    size_t segment_buffer_size = padded_h * padded_w * sizeof(uint16_t);
    uint16_t* segment_buffer = (uint16_t*)malloc(segment_buffer_size);
    if (!segment_buffer) {
        return ICER_FATAL_ERROR;
    }
    
    // Clear buffer to ensure clean state (padding will be zeros)
    memset(segment_buffer, 0, segment_buffer_size);
    
    /*
     * Process top region which consists of c columns
     * height of top region is h_t and it contains r_t rows
     */
    for (uint16_t row = 0; row < params->r_t; row++) {
        /*
         * the first r_t0 rows have height y_t
         * the remainder have height y_t + 1
         */
        segment_h = params->y_t + ((row >= params->r_t0) ? 1 : 0);
        partition_col_ind = 0;
        
        for (uint16_t col = 0; col < params->c; col++) {
            /* the first c_t0 columns have width x_t
             * the remainder have width x_t + 1
             */
            segment_w = params->x_t + ((col >= params->c_t0) ? 1 : 0);
            
            // Calculate segment position in flash file
            // Segment starts at: file_offset + (partition_row_ind * rowstride + partition_col_ind) * sizeof(uint16_t)
            size_t segment_start_offset = file_offset + 
                (partition_row_ind * rowstride + partition_col_ind) * sizeof(uint16_t);
            
            // Read segment from flash into padded buffer
            // Buffer layout: [padding row][data row with left/right padding][padding row]
            // Segment data starts at buffer[1 * padded_w + 1] (skip top padding row and left padding)
            // This allows icer_compress_bitplane to access neighbors safely
            flash_file->seek(segment_start_offset);
            for (size_t seg_row = 0; seg_row < segment_h; seg_row++) {
                // Calculate position in flash file for this row
                size_t row_offset = segment_start_offset + seg_row * rowstride * sizeof(uint16_t);
                flash_file->seek(row_offset);
                
                // Read one row of the segment into buffer with padding
                // Write to buffer[(seg_row + 1) * padded_w + 1] (skip top padding and left padding)
                size_t row_bytes = segment_w * sizeof(uint16_t);
                size_t buffer_offset = (seg_row + 1) * padded_w + 1;  // +1 for top padding, +1 for left padding
                size_t bytes_read = flash_file->read(
                    (uint8_t*)(segment_buffer + buffer_offset),
                    row_bytes
                );
                
                if (bytes_read != row_bytes) {
                    free(segment_buffer);
                    return ICER_FATAL_ERROR;
                }
                
                // Pad left and right edges with edge pixel value (replication)
                // Left padding: copy first pixel
                if (buffer_offset > 0) {
                    segment_buffer[buffer_offset - 1] = segment_buffer[buffer_offset];
                }
                // Right padding: copy last pixel
                if (buffer_offset + segment_w < padded_w * padded_h) {
                    segment_buffer[buffer_offset + segment_w] = segment_buffer[buffer_offset + segment_w - 1];
                }
            }
            
            // Pad top and bottom rows by replicating first/last data row
            // Top padding row: replicate first data row (including left/right padding)
            if (segment_h > 0) {
                // Copy entire first data row (including its left/right padding) to top padding row
                for (size_t col = 0; col < padded_w; col++) {
                    segment_buffer[col] = segment_buffer[padded_w + col];  // Copy row 1 to row 0
                }
            }
            // Bottom padding row: replicate last data row (including left/right padding)
            if (segment_h > 0) {
                size_t last_data_row_start = segment_h * padded_w;
                size_t bottom_padding_row_start = (segment_h + 1) * padded_w;
                for (size_t col = 0; col < padded_w; col++) {
                    segment_buffer[bottom_padding_row_start + col] = segment_buffer[last_data_row_start + col];
                }
            }
            
            // Calculate segment_start pointer: skip top padding row and left padding column
            // This points to the actual segment data in the padded buffer
            const uint16_t* segment_start = segment_buffer + padded_w + 1;  // Skip top row (padded_w) and left column (+1)
            
            // Use padded_w as rowstride for icer_compress_bitplane
            // This allows it to access neighbors correctly within the padded buffer
            size_t segment_rowstride = padded_w;
            
            partition_col_ind += segment_w;
            
            // Initialize context model (same as standard ICER)
            icer_init_context_model_vals(&context_model, (enum icer_subband_types)pkt_context->subband_type);
            
            // Allocate data packet (same as standard ICER)
            res = icer_allocate_data_packet(&seg, output_data, segment_num, pkt_context);
            if (res != ICER_RESULT_OK) {
                free(segment_buffer);
                return res;
            }
            
            // Initialize entropy coder context (same as standard ICER)
            icer_init_entropy_coder_context(&context, icer_encode_circ_buf, ICER_CIRC_BUF_SIZE,
                                            (uint8_t *) seg + sizeof(icer_image_segment_typedef), seg->data_length);
            
            // Call standard ICER bitplane compression (NO ALGORITHM CHANGES)
            // This ensures 100% output compatibility
            // Use segment_rowstride (padded_w) instead of full image rowstride
            // The padded buffer provides boundary pixels for neighbor access
            res = icer_compress_bitplane_uint16(segment_start, segment_w, segment_h, segment_rowstride, &context_model, &context,
                                               pkt_context);
            if (res != ICER_RESULT_OK) {
                output_data->size_used -= sizeof(icer_image_segment_typedef);
                free(segment_buffer);
                return res;
            }
            
            // Calculate output size and update segment (same as standard ICER)
            data_in_bytes = context.output_ind + (context.output_bit_offset > 0);
            seg->data_length = context.output_ind * 8 + context.output_bit_offset;
            seg->data_crc32 = icer_calculate_segment_crc32(seg);
            seg->crc32 = icer_calculate_packet_crc32(seg);
            output_data->size_used += data_in_bytes;
            
            segments_encoded[segment_num] = seg;
            
            segment_num++;
        }
        partition_row_ind += segment_h;
    }
    
    /*
     * if the bottom region exists, process bottom region
     * which consists of c+1 columns
     */
    for (uint16_t row = 0; row < (params->r - params->r_t); row++) {
        /*
         * the first r_b0 rows have height y_b
         * the remainder have height y_b + 1
         */
        segment_h = params->y_b + ((row >= params->r_b0) ? 1 : 0);
        partition_col_ind = 0;
        
        for (uint16_t col = 0; col < (params->c + 1); col++) {
            /* the first c_b0 columns have width x_b
             * the remainder have width x_b + 1
             */
            segment_w = params->x_b + ((col >= params->c_b0) ? 1 : 0);
            
            // Calculate segment position in flash file
            size_t segment_start_offset = file_offset + 
                (partition_row_ind * rowstride + partition_col_ind) * sizeof(uint16_t);
            
            // Read segment from flash into padded buffer (same as top region)
            flash_file->seek(segment_start_offset);
            for (size_t seg_row = 0; seg_row < segment_h; seg_row++) {
                size_t row_offset = segment_start_offset + seg_row * rowstride * sizeof(uint16_t);
                flash_file->seek(row_offset);
                
                size_t row_bytes = segment_w * sizeof(uint16_t);
                size_t buffer_offset = (seg_row + 1) * padded_w + 1;  // +1 for top padding, +1 for left padding
                size_t bytes_read = flash_file->read(
                    (uint8_t*)(segment_buffer + buffer_offset),
                    row_bytes
                );
                
                if (bytes_read != row_bytes) {
                    free(segment_buffer);
                    return ICER_FATAL_ERROR;
                }
                
                // Pad left and right edges
                if (buffer_offset > 0) {
                    segment_buffer[buffer_offset - 1] = segment_buffer[buffer_offset];
                }
                if (buffer_offset + segment_w < padded_w * padded_h) {
                    segment_buffer[buffer_offset + segment_w] = segment_buffer[buffer_offset + segment_w - 1];
                }
            }
            
            // Pad top and bottom rows (including left/right padding)
            if (segment_h > 0) {
                // Copy entire first data row (including its left/right padding) to top padding row
                for (size_t col = 0; col < padded_w; col++) {
                    segment_buffer[col] = segment_buffer[padded_w + col];  // Copy row 1 to row 0
                }
                // Copy entire last data row (including its left/right padding) to bottom padding row
                size_t last_data_row_start = segment_h * padded_w;
                size_t bottom_padding_row_start = (segment_h + 1) * padded_w;
                for (size_t col = 0; col < padded_w; col++) {
                    segment_buffer[bottom_padding_row_start + col] = segment_buffer[last_data_row_start + col];
                }
            }
            
            const uint16_t* segment_start = segment_buffer + padded_w + 1;
            size_t segment_rowstride = padded_w;
            
            partition_col_ind += segment_w;
            
            // Initialize context model
            icer_init_context_model_vals(&context_model, (enum icer_subband_types)pkt_context->subband_type);
            
            // Allocate data packet
            res = icer_allocate_data_packet(&seg, output_data, segment_num, pkt_context);
            if (res != ICER_RESULT_OK) {
                free(segment_buffer);
                return res;
            }
            
            // Initialize entropy coder context
            icer_init_entropy_coder_context(&context, icer_encode_circ_buf, ICER_CIRC_BUF_SIZE,
                                            (uint8_t *) seg + sizeof(icer_image_segment_typedef), seg->data_length);
            
            // Call standard ICER bitplane compression (NO ALGORITHM CHANGES)
            res = icer_compress_bitplane_uint16(segment_start, segment_w, segment_h, segment_rowstride, &context_model, &context,
                                               pkt_context);
            if (res != ICER_RESULT_OK) {
                output_data->size_used -= sizeof(icer_image_segment_typedef);
                free(segment_buffer);
                return res;
            }
            
            // Calculate output size and update segment
            data_in_bytes = context.output_ind + (context.output_bit_offset > 0);
            seg->data_length = context.output_ind * 8 + context.output_bit_offset;
            seg->data_crc32 = icer_calculate_segment_crc32(seg);
            seg->crc32 = icer_calculate_packet_crc32(seg);
            output_data->size_used += data_in_bytes;
            
            segments_encoded[segment_num] = seg;
            
            segment_num++;
        }
        partition_row_ind += segment_h;
    }
    
    free(segment_buffer);
    return ICER_RESULT_OK;
}

