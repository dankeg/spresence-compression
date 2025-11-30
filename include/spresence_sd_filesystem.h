#ifndef SPRESENCE_SD_FILESYSTEM_H
#define SPRESENCE_SD_FILESYSTEM_H

#include "filesystem_interface.h"

// Forward declaration
class SDClass;

// Factory function to create a Spresence SD file system wrapper
// This wraps an existing SDClass instance with the IFileSystem interface
// 
// Parameters:
//   sd_card: Pointer to existing SDClass instance (must remain valid for lifetime of IFileSystem)
//   take_ownership: If true, the SDClass will be deleted when IFileSystem is destroyed
//                   If false, caller is responsible for managing SDClass lifetime
//
// Returns:
//   Pointer to IFileSystem implementation, or NULL on failure
//   Caller is responsible for deleting the returned pointer
IFileSystem* createSpresenceSDFileSystem(SDClass* sd_card, bool take_ownership = false);

#endif // SPRESENCE_SD_FILESYSTEM_H

