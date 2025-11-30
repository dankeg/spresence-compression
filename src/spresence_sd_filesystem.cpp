#include "filesystem_interface.h"
#include <SDHCI.h>

// Concrete implementation of IFileSystem and IFile for Spresence SD card library
// This wraps the SDClass and File classes from SDHCI.h

// Spresence SD card file implementation (defined first so it can be used by SpresenceSDFileSystem)
class SpresenceSDFile : public IFile {
private:
    File file_handle;
    bool is_open;
    
public:
    // Constructor: takes a File object (copies it)
    SpresenceSDFile(File file) : file_handle(file), is_open((bool)file) {}
    
    // Destructor: closes file if still open
    ~SpresenceSDFile() override {
        if (is_open) {
            close();
        }
    }
    
    // Read data from file
    size_t read(uint8_t* buffer, size_t size) override {
        if (!is_open) {
            return 0;
        }
        return file_handle.read(buffer, size);
    }
    
    // Write data to file
    size_t write(const uint8_t* data, size_t size) override {
        if (!is_open) {
            return 0;
        }
        return file_handle.write(data, size);
    }
    
    // Seek to a specific position
    bool seek(size_t position) override {
        if (!is_open) {
            return false;
        }
        return file_handle.seek(position);
    }
    
    // Get current file position
    size_t position() override {
        if (!is_open) {
            return 0;
        }
        return file_handle.position();
    }
    
    // Get file size
    size_t size() override {
        if (!is_open) {
            return 0;
        }
        return file_handle.size();
    }
    
    // Flush buffered data
    bool flush() override {
        if (!is_open) {
            return false;
        }
        file_handle.flush();  // Arduino File.flush() returns void
        return true;
    }
    
    // Close the file
    bool close() override {
        if (!is_open) {
            return true;  // Already closed
        }
        file_handle.close();  // Arduino File.close() returns void
        is_open = false;
        return true;
    }
    
    // Check if file is open
    bool isOpen() const override {
        return is_open;
    }
    
    // Boolean conversion operator
    operator bool() const override {
        // Just return is_open since we track the file state
        // The File object itself may have non-const operator bool(), but we've already
        // validated it when we created this object, and we track open state explicitly
        return is_open;
    }
    
    // Get the underlying File object (for compatibility if needed)
    File getFileHandle() const {
        return file_handle;
    }
};

// Spresence SD card file system implementation
class SpresenceSDFileSystem : public IFileSystem {
private:
    SDClass* sd_card;
    bool owns_sd_card;  // If true, we own the SDClass and should delete it in destructor
    
public:
    // Constructor: takes an existing SDClass pointer
    // If take_ownership is true, the SDClass will be deleted in destructor
    SpresenceSDFileSystem(SDClass* sd, bool take_ownership = false)
        : sd_card(sd), owns_sd_card(take_ownership) {}
    
    // Destructor
    ~SpresenceSDFileSystem() override {
        if (owns_sd_card && sd_card) {
            delete sd_card;
        }
    }
    
    // Initialize the file system
    bool begin() override {
        if (!sd_card) {
            return false;
        }
        return sd_card->begin();
    }
    
    // Open a file
    IFile* open(const char* filename, int mode) override {
        if (!sd_card) {
            return nullptr;
        }
        
        // Pass mode directly to Arduino SD library - it accepts FILE_READ/FILE_WRITE macros
        // which are integer constants. Also accepts our FileMode enum values (0/1).
        // Arduino File.open() accepts int flags directly
        File file = sd_card->open(filename, mode);
        if (!file) {
            return nullptr;
        }
        
        // Create a wrapper object
        // Note: The File object is copied, so the original can be destroyed
        return new SpresenceSDFile(file);
    }
    
    // Remove a file
    bool remove(const char* filename) override {
        if (!sd_card) {
            return false;
        }
        return sd_card->remove(filename);
    }
    
    // Check if file exists
    bool exists(const char* filename) override {
        if (!sd_card) {
            return false;
        }
        File file = sd_card->open(filename, FILE_READ);
        if (file) {
            file.close();
            return true;
        }
        return false;
    }
    
    // Get the underlying SDClass pointer (for compatibility if needed)
    SDClass* getSDClass() const {
        return sd_card;
    }
};

// Factory function to create a Spresence SD file system wrapper
// This is the recommended way to create the file system interface
IFileSystem* createSpresenceSDFileSystem(SDClass* sd_card, bool take_ownership = false) {
    return new SpresenceSDFileSystem(sd_card, take_ownership);
}

