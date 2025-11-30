#ifndef FLASH_PARTITION_H
#define FLASH_PARTITION_H

#include <stdint.h>
#include <stddef.h>

// Forward declarations
class IFile;
class SDClass;

extern "C" {
#include "icer.h"
}

// Flash-based partition compression that reads segments from flash on-demand
// This maintains 100% compatibility with standard ICER output
// 
// Parameters:
// - flash_file: File handle to read transformed image data from flash
// - file_offset: Byte offset in flash file where subband data starts
// - params: Partition parameters (same as standard ICER)
// - rowstride: Full image width (for calculating segment positions)
// - pkt_context: Packet context (same as standard ICER)
// - output_data: Output buffer (same as standard ICER)
// - segments_encoded: Array to store segment pointers (same as standard ICER)
//
// Returns: ICER_RESULT_OK on success, error code on failure
//
// Algorithm:
// 1. For each segment defined by partition parameters:
//    a. Calculate segment position in flash file
//    b. Read segment data from flash into buffer (with correct rowstride)
//    c. Call standard icer_compress_bitplane_uint16() (no algorithm changes)
//    d. Process output normally
// 2. Output is identical to standard ICER partition function
//
// RAM Usage: ~segment_w * segment_h * sizeof(uint16_t) per segment
// For typical segments: ~10-50 KB
int icer_compress_partition_uint16_flash(
    IFile* flash_file,
    size_t file_offset,
    const partition_param_typdef *params,
    size_t rowstride,
    icer_packet_context *pkt_context,
    icer_output_data_buf_typedef *output_data,
    const icer_image_segment_typedef *segments_encoded[]
);

#endif // FLASH_PARTITION_H

