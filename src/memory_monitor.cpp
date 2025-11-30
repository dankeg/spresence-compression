#include "memory_monitor.h"
#include <Arduino.h>
#include <stdlib.h>

#ifdef __arm__
#include <malloc.h>
#else
extern char* __brkval;
extern char* __heap_start;
#endif

size_t getFreeHeapMemory(void) {
#ifdef __arm__
    struct mallinfo mi = mallinfo();
    return (size_t)mi.fordblks;
#else
    // AVR: Use standard Arduino approach
    char stack_dummy = 0;
    char* stack_top = &stack_dummy;
    char* heap_top = (__brkval == 0) ? (char*)&__heap_start : __brkval;
    
    if (stack_top < heap_top) {
        return 0; // Invalid state
    }
    
    return (size_t)(stack_top - heap_top);
#endif
}

size_t getTotalHeapSize(void) {
#ifdef __arm__
    struct mallinfo mi = mallinfo();
    return (size_t)mi.arena;
#else
    return 0;
#endif
}

size_t getUsedHeapMemory(void) {
#ifdef __arm__
    struct mallinfo mi = mallinfo();
    return (size_t)mi.uordblks;
#else
    // AVR: Use standard Arduino approach
    char* heap_top = (__brkval == 0) ? (char*)&__heap_start : __brkval;
    
    extern char _end;
    char* heap_start = &_end;
    
    if (heap_top < heap_start) {
        return 0; // Invalid state
    }
    
    return (size_t)(heap_top - heap_start);
#endif
}

void printMemoryStats(const char* label) {
    if (!Serial) return;
    
    size_t free_mem = getFreeHeapMemory();
    
    Serial.print("[MEM] ");
    if (label) {
        Serial.print(label);
        Serial.print(": ");
    }
    Serial.print("Free heap: ");
    Serial.print(free_mem);
    Serial.print(" bytes (");
    Serial.print(free_mem / 1024);
    Serial.print(" KB)");
    Serial.println();
}

void printDetailedMemoryInfo(const char* label) {
    if (!Serial) return;
    
    size_t free_mem = getFreeHeapMemory();
    size_t used_mem = getUsedHeapMemory();
    
    Serial.println("========================================");
    Serial.print("[MEM] ");
    if (label) {
        Serial.println(label);
    } else {
        Serial.println("Memory Status");
    }
    Serial.println("----------------------------------------");
    Serial.print("  Free heap:  ");
    Serial.print(free_mem);
    Serial.print(" bytes (");
    Serial.print(free_mem / 1024);
    Serial.println(" KB)");
    
    if (used_mem > 0) {
        Serial.print("  Used heap:  ");
        Serial.print(used_mem);
        Serial.print(" bytes (");
        Serial.print(used_mem / 1024);
        Serial.println(" KB)");
        
        size_t total_approx = free_mem + used_mem;
        Serial.print("  Total (est): ");
        Serial.print(total_approx);
        Serial.print(" bytes (");
        Serial.print(total_approx / 1024);
        Serial.println(" KB)");
    }
    
    Serial.println("========================================");
}

