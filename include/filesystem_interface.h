#ifndef FILESYSTEM_INTERFACE_H
#define FILESYSTEM_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Forward declaration
class IFile;

// File open modes (matching Spresence SD library)
enum FileMode {
    FILE_READ = 0,
    FILE_WRITE = 1
};

// Abstract file system interface
// This allows switching between different file system implementations
// (e.g., Spresence SD card, LittleFS, SPIFFS, etc.) without changing application code
class IFileSystem {
public:
    virtual ~IFileSystem() = default;
    
    // Initialize the file system
    // Returns true on success, false on failure
    virtual bool begin() = 0;
    
    // Open a file
    // Returns a pointer to IFile object, or NULL on failure
    // Caller is responsible for deleting the IFile object when done
    // Accepts int to be compatible with Arduino FILE_READ/FILE_WRITE macros
    virtual IFile* open(const char* filename, int mode) = 0;
    
    // Remove/delete a file
    // Returns true on success, false on failure
    virtual bool remove(const char* filename) = 0;
    
    // Check if file exists
    // Returns true if file exists, false otherwise
    virtual bool exists(const char* filename) = 0;
};

// Abstract file interface
// Represents an open file handle
class IFile {
public:
    virtual ~IFile() = default;
    
    // Read data from file
    // Returns number of bytes read (0 on error or EOF)
    virtual size_t read(uint8_t* buffer, size_t size) = 0;
    
    // Write data to file
    // Returns number of bytes written (0 on error)
    virtual size_t write(const uint8_t* data, size_t size) = 0;
    
    // Seek to a specific position in file
    // Returns true on success, false on failure
    virtual bool seek(size_t position) = 0;
    
    // Get current file position
    // Returns current position, or 0 on error
    virtual size_t position() = 0;
    
    // Get file size
    // Returns file size in bytes, or 0 on error
    virtual size_t size() = 0;
    
    // Flush any buffered data to storage
    // Returns true on success, false on failure
    virtual bool flush() = 0;
    
    // Close the file
    // Returns true on success, false on failure
    virtual bool close() = 0;
    
    // Check if file is open and valid
    // Returns true if file is open, false otherwise
    virtual bool isOpen() const = 0;
    
    // Conversion operator for boolean checks (e.g., if (file) { ... })
    virtual operator bool() const = 0;
};

#endif // FILESYSTEM_INTERFACE_H

