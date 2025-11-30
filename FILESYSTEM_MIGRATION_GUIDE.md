# File System Abstraction Migration Guide

## Status

✅ **Interface Created**: `filesystem_interface.h` defines `IFileSystem` and `IFile`  
✅ **Spresence Implementation**: `spresence_sd_filesystem.cpp` wraps SDClass  
✅ **Headers Updated**: Function signatures updated to accept `IFileSystem*`  
⏳ **Implementations**: Need to be updated to use `IFile*` instead of `File`  
⏳ **Backward Compatibility**: Wrappers need to be added  

## What's Been Done

1. **Interface Definition** (`include/filesystem_interface.h`)
   - `IFileSystem` abstract class
   - `IFile` abstract class
   - Matches Spresence SD library API

2. **Spresence Implementation** (`src/spresence_sd_filesystem.cpp`)
   - `SpresenceSDFileSystem` wraps `SDClass`
   - `SpresenceSDFile` wraps `File`
   - Factory function: `createSpresenceSDFileSystem()`

3. **Header Updates**
   - All function signatures now accept `IFileSystem*`
   - Backward compatibility overloads accept `SDClass*`

## What Needs to Be Done

### Step 1: Update Implementation Files

Replace `File` with `IFile*` and manage memory:

**Before:**
```cpp
File file = sd_card->open(filename, FILE_WRITE);
if (file) {
    file.write(data, size);
    file.close();
}
```

**After:**
```cpp
IFile* file = filesystem->open(filename, FILE_WRITE);
if (file) {
    file->write(data, size);
    file->close();
    delete file;  // Must delete IFile* objects
}
```

### Step 2: Update Function Signatures

**Before:**
```cpp
int myFunction(SDClass* sd_card, const char* filename) {
    File file = sd_card->open(filename, FILE_READ);
    // ...
}
```

**After:**
```cpp
int myFunction(IFileSystem* filesystem, const char* filename) {
    IFile* file = filesystem->open(filename, FILE_READ);
    if (!file) return -1;
    
    // ... use file ...
    
    file->close();
    delete file;
    return 0;
}
```

### Step 3: Add Backward Compatibility Wrappers

At the end of each implementation file:

```cpp
// Backward compatibility wrapper
int myFunction(SDClass* sd_card, const char* filename) {
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) return -1;
    
    int result = myFunction(fs, filename);
    
    delete fs;
    return result;
}
```

## Files That Need Updates

1. `src/camera_yuv.cpp` - Replace `File` with `IFile*`
2. `src/flash_icer_compression.cpp` - Replace `File` with `IFile*`
3. `src/flash_wavelet.cpp` - Replace `File` with `IFile*`
4. `src/flash_partition.cpp` - Replace `File*` with `IFile*`
5. `src/main.cpp` - Update to use `IFileSystem*`

## Example: Minimal Migration

Here's a complete example showing how to migrate one function:

### Original Code
```cpp
int writeToFile(SDClass* sd_card, const char* filename, const uint8_t* data, size_t size) {
    File file = sd_card->open(filename, FILE_WRITE);
    if (!file) {
        return -1;
    }
    
    size_t written = file.write(data, size);
    file.close();
    
    if (written != size) {
        return -2;
    }
    return 0;
}
```

### Migrated Code
```cpp
// New interface-based function
int writeToFile(IFileSystem* filesystem, const char* filename, const uint8_t* data, size_t size) {
    IFile* file = filesystem->open(filename, FILE_WRITE);
    if (!file) {
        return -1;
    }
    
    size_t written = file->write(data, size);
    bool close_ok = file->close();
    delete file;  // Must delete IFile* objects
    
    if (!close_ok || written != size) {
        return -2;
    }
    return 0;
}

// Backward compatibility wrapper
int writeToFile(SDClass* sd_card, const char* filename, const uint8_t* data, size_t size) {
    IFileSystem* fs = createSpresenceSDFileSystem(sd_card, false);
    if (!fs) return -1;
    
    int result = writeToFile(fs, filename, data, size);
    
    delete fs;
    return result;
}
```

## Key Differences

| Old (SDClass/File) | New (IFileSystem/IFile) |
|-------------------|------------------------|
| `File file = sd->open(...)` | `IFile* file = fs->open(...)` |
| `file.write(...)` | `file->write(...)` |
| `file.close()` | `file->close(); delete file;` |
| Automatic cleanup | Manual `delete` required |
| Value semantics | Pointer semantics |

## Testing

After migration:
1. ✅ Existing code should still compile (backward compatibility)
2. ✅ Existing code should still work (wrappers)
3. ✅ New code can use `IFileSystem*` directly
4. ✅ Future file systems can be added by implementing the interface

## Next Steps

1. Update `camera_yuv.cpp` to use `IFile*`
2. Update `flash_wavelet.cpp` to use `IFile*`
3. Update `flash_icer_compression.cpp` to use `IFile*`
4. Update `flash_partition.cpp` to use `IFile*`
5. Update `main.cpp` to create and use `IFileSystem*`
6. Test thoroughly
7. Document any issues

## Future: Adding New File System

To add a new file system (e.g., LittleFS):

1. Create `littlefs_filesystem.cpp`:
```cpp
#include "filesystem_interface.h"
// ... LittleFS includes ...

class LittleFSFileSystem : public IFileSystem {
    // Implement all virtual functions
};

class LittleFSFile : public IFile {
    // Implement all virtual functions
};

IFileSystem* createLittleFSFileSystem() {
    return new LittleFSFileSystem();
}
```

2. Use it:
```cpp
IFileSystem* fs = createLittleFSFileSystem();
fs->begin();
// Use fs exactly like Spresence SD filesystem
```

No other code changes needed!

