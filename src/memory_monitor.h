#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

#include <stdint.h>
#include <stddef.h>

// Get free heap memory in bytes
// Returns available heap memory, or 0 if unable to determine
size_t getFreeHeapMemory(void);

// Get total heap size (if available)
size_t getTotalHeapSize(void);

// Get used heap memory
size_t getUsedHeapMemory(void);

// Print memory statistics to Serial
void printMemoryStats(const char* label);

// Print detailed memory information
void printDetailedMemoryInfo(const char* label);

#endif // MEMORY_MONITOR_H



