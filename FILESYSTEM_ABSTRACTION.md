# File System Abstraction Layer

## Overview

This abstraction layer allows the codebase to work with different file system implementations (Spresence SD card, LittleFS, SPIFFS, etc.) without changing application code.

## Architecture

### Interface Classes

1. **IFileSystem** - Abstract file system interface
   - `begin()` - Initialize file system
   - `open(filename, mode)` - Open a file, returns IFile*
   - `remove(filename)` - Delete a file
   - `exists(filename)` - Check if file exists

2. **IFile** - Abstract file interface
   - `read(buffer, size)` - Read data
   - `write(data, size)` - Write data
   - `seek(position)` - Seek to position
   - `position()` - Get current position
   - `size()` - Get file size
   - `flush()` - Flush buffers
   - `close()` - Close file
   - `isOpen()` - Check if open
   - `operator bool()` - Boolean conversion

### Implementation

**SpresenceSDFileSystem** - Wraps SDClass from SDHCI.h
- Created via `createSpresenceSDFileSystem(SDClass*, bool take_ownership)`
- Wraps existing SDClass instance
- SpresenceSDFile wraps File objects

## Usage

### Current Code (Backward Compatible)

```cpp
SDClass theSD;
theSD.begin();

// Old code still works - backward compatibility wrappers
convertJpegToSeparateChannels(jpeg_img, &width, &height, 
                              y_file, u_file, v_file, &theSD);
```

### New Code (Using Interface)

```cpp
#include "spresence_sd_filesystem.h"

SDClass theSD;
theSD.begin();

// Create file system wrapper
IFileSystem* fs = createSpresenceSDFileSystem(&theSD, false);

// Use interface
convertJpegToSeparateChannels(jpeg_img, &width, &height,
                              y_file, u_file, v_file, fs);

// Clean up (if you created it)
delete fs;
```

### Future: Different File System

```cpp
#include "littlefs_filesystem.h"  // Future implementation

// Create LittleFS file system
IFileSystem* fs = createLittleFSFileSystem();

// Same code works!
convertJpegToSeparateChannels(jpeg_img, &width, &height,
                              y_file, u_file, v_file, fs);
```

## Memory Management

- **IFileSystem**: Created with factory function, caller must delete
- **IFile**: Created by IFileSystem::open(), caller must delete
- **SpresenceSDFileSystem**: Can optionally take ownership of SDClass

## Migration Path

1. ✅ Interface defined (`filesystem_interface.h`)
2. ✅ Spresence implementation created (`spresence_sd_filesystem.cpp`)
3. ✅ Headers updated to use IFileSystem* (with backward compatibility)
4. ⏳ Implementation files need updating (in progress)
5. ⏳ Main.cpp needs updating to use interface

## Backward Compatibility

All existing code continues to work. The backward compatibility wrappers:
- Accept SDClass* parameters
- Create temporary IFileSystem wrapper
- Call the new interface-based functions
- Clean up automatically

This allows gradual migration without breaking existing code.

