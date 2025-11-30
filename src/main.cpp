#include <Arduino.h>
#include <SDHCI.h>
#include <Camera.h>
#include "camera_yuv.h"
#include "icer_compression.h"
#include "flash_icer_compression.h"
#include "flash_wavelet.h"
#include "memory_monitor.h"

// Try to use GNSS RAM if available (640 KB additional memory if GNSS not used)
// Requires SDK 3.2.0+ and bootloader update
#ifdef __arm__
#include <arch/chip/gnssram.h>
#endif

#define BAUDRATE 115200

SDClass theSD;
int take_picture_count = 0;

void setup() {
    Serial.begin(BAUDRATE);
    while (!Serial) { ; }

    Serial.println("Spresense Camera to ICER Pipeline");
    Serial.println("========================================");
    
    printDetailedMemoryInfo("Initial State");
    Serial.print("Heap configuration: Stack=");
    #ifdef CONFIG_MAIN_CORE_STACKSIZE
    Serial.print(CONFIG_MAIN_CORE_STACKSIZE / 1024);
    Serial.print(" KB (configured)");
    #else
    Serial.print("default");
    #endif
    Serial.print(", Heap=");
    #ifdef CONFIG_MAIN_CORE_HEAPSIZE
    Serial.print(CONFIG_MAIN_CORE_HEAPSIZE / 1024);
    Serial.print(" KB (configured)");
    #else
    Serial.print("default");
    #endif
    Serial.println();
    
    // Verify actual heap size
    size_t total_heap = getTotalHeapSize();
    Serial.print("Actual total heap size: ");
    Serial.print(total_heap / 1024);
    Serial.println(" KB");

    // Try to initialize GNSS RAM for additional memory (640 KB if GNSS not used)
    // This requires SDK 3.2.0+ and updated bootloader
    // Note: up_gnssram_initialize() returns void - it either succeeds or crashes
    #ifdef __arm__
    Serial.println("Attempting to initialize GNSS RAM for additional memory...");
    up_gnssram_initialize();
    Serial.println("GNSS RAM initialization called (640 KB additional memory if available)");
    Serial.println("Note: Requires SDK 3.2.0+ and updated bootloader");
    printMemoryStats("After GNSS RAM init");
    
    // Enable GNSS RAM usage for ICER buffers (frees main RAM for camera)
    // This moves ~115 KB of ICER buffers to GNSS RAM
    // Note: Must call for both icer_compression.cpp and flash_icer_compression.cpp
    // as they are separate compilation units with separate static variables
    setGnssRamAvailable(true);
    // Also set for flash_icer_compression (separate function in different compilation unit)
    // Function is declared in flash_icer_compression.h which is already included
    setGnssRamAvailable_flash(true);
    // Also set for flash_wavelet (separate function in different compilation unit)
    // Function is declared in flash_wavelet.h which is already included
    setGnssRamAvailable_wavelet(true);
    Serial.println("GNSS RAM enabled for ICER buffer allocation");
    #endif

    Serial.println("Initializing SD card...");
    while (!theSD.begin()) {
        Serial.println("Insert SD card.");
        delay(1000);
    }
    printMemoryStats("After SD card init");
    
    Serial.println("Setup complete. Camera will be initialized in loop() when needed.");
    Serial.println("========================================");
}

void loop() {
        if (take_picture_count < 1) {
        delay(2000);

        Serial.println("----------------------------------------");
        Serial.print("Picture #");
        Serial.println(take_picture_count);
        
        // First: Capture JPEG at maximum resolution
        // Maximize memory by ensuring clean state
        Serial.println("Capturing JPEG at max resolution...");
        CamImage jpeg_img;
        printMemoryStats("Before JPEG capture");
        
        // Ensure camera is not initialized (clean state)
        theCamera.end();
        delay(300);  // Give time for cleanup
        printMemoryStats("After ensuring camera.end()");
        
        // Initialize camera fresh for JPEG capture
        int err = theCamera.begin();
        if (err != 0) {
            Serial.print("Failed to initialize camera for JPEG: ");
            Serial.println(err);
        } else {
            Serial.println("Setting still picture format for JPEG...");
            printMemoryStats("After camera.begin()");
            
            // Try progressively lower resolutions until one works
            struct {
                int width;
                int height;
                const char* name;
            } resolutions[] = {
                {CAM_IMGSIZE_QUADVGA_H, CAM_IMGSIZE_QUADVGA_V, "QUADVGA (1280x960)"},
                {CAM_IMGSIZE_VGA_H, CAM_IMGSIZE_VGA_V, "VGA (640x480)"},
                {CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, "QVGA (320x240)"},
                {CAM_IMGSIZE_QQVGA_H, CAM_IMGSIZE_QQVGA_V, "QQVGA (160x120)"}
            };
            
            bool format_set = false;
            for (int i = 0; i < 4; i++) {
                Serial.print("Attempting to set format: ");
                Serial.println(resolutions[i].name);
                printMemoryStats("Before setStillPictureImageFormat");
                
                // For QUADVGA, try setting format with a delay to allow memory cleanup
                if (i == 0) {
                    delay(100);  // Give system time to free any fragmented memory
                    printMemoryStats("After delay before QUADVGA");
                }
                
                // Set JPEG format with increased buffer divisor to reduce buffer allocation
                // Default divisor is 7, using 8-10 reduces buffer size by ~12-30%
                // Formula: buffer_size = width * height * 2 / jpgbufsize_divisor
                // Higher divisor = smaller buffer, but must be large enough for compressed JPEG
                int jpgbufsize_divisor = 8;  // Reduced from default 7 to save memory
                
                err = theCamera.setStillPictureImageFormat(
                    resolutions[i].width,
                    resolutions[i].height,
                    CAM_IMAGE_PIX_FMT_JPG,
                    jpgbufsize_divisor
                );
                
                if (err == CAM_ERR_SUCCESS) {
                    Serial.print("JPEG format set successfully: ");
                    Serial.println(resolutions[i].name);
                    printMemoryStats("After setStillPictureImageFormat");
                    
                    // Set JPEG quality AFTER format setup to reduce final JPEG file size
                    // This doesn't affect buffer allocation (controlled by jpgbufsize_divisor)
                    // but reduces the actual compressed JPEG size, allowing smaller buffers to work
                    // Quality range: 1-100 (1 = lowest quality/smallest size, 100 = highest quality/largest size)
                    const int jpeg_quality = 50;  // Lower quality = smaller JPEGs = fits in smaller buffer
                    err = theCamera.setJPEGQuality(jpeg_quality);
                    if (err == CAM_ERR_SUCCESS) {
                        Serial.print("JPEG quality set to ");
                        Serial.print(jpeg_quality);
                        Serial.println("% (reduces final JPEG size)");
                    } else {
                        Serial.print("Warning: Failed to set JPEG quality (error: ");
                        Serial.print(err);
                        Serial.println("), continuing with default quality");
                    }
                    
                    format_set = true;
                    break;
                } else {
                    Serial.print("Failed to set JPEG format at ");
                    Serial.print(resolutions[i].name);
                    Serial.print(" (error: ");
                    Serial.print(err);
                    Serial.println(")");
                    printMemoryStats("After failed setStillPictureImageFormat");
                    
                    // For QUADVGA failure, try ending and reinitializing camera
                    // to free any partially allocated buffers
                    if (i == 0) {
                        Serial.println("Reinitializing camera after QUADVGA failure...");
                        theCamera.end();
                        delay(200);
                        err = theCamera.begin();
                        if (err != 0) {
                            Serial.print("Failed to reinitialize camera: ");
                            Serial.println(err);
                            break;
                        }
                        printMemoryStats("After camera reinitialization");
                    }
                }
            }
            
            if (!format_set) {
                Serial.println("ERROR: Could not set JPEG format at any resolution");
                Serial.println("Skipping JPEG capture, proceeding to ICER pipeline...");
            } else {
                // Format was set successfully, take picture
                Serial.println("Taking JPEG picture...");
                printMemoryStats("Before takePicture");
                
                jpeg_img = theCamera.takePicture();
                
                printMemoryStats("After takePicture");
                
                if (jpeg_img.isAvailable()) {
                    size_t jpeg_size = jpeg_img.getImgSize();
                    size_t jpeg_width = jpeg_img.getWidth();
                    size_t jpeg_height = jpeg_img.getHeight();
                    
                    Serial.print("JPEG captured: ");
                    Serial.print(jpeg_width);
                    Serial.print("x");
                    Serial.print(jpeg_height);
                    Serial.print(" (");
                    Serial.print(jpeg_size);
                    Serial.println(" bytes)");
                    
                    // Save JPEG to file
                    const char* jpeg_filename = "CAPTURE.JPG";
                    theSD.remove(jpeg_filename);
                    File jpegFile = theSD.open(jpeg_filename, FILE_WRITE);
                    if (jpegFile) {
                        const uint8_t* jpeg_buff = jpeg_img.getImgBuff();
                        size_t written = jpegFile.write(jpeg_buff, jpeg_size);
                        jpegFile.close();
                        
                        if (written == jpeg_size) {
                            Serial.print("Saved JPEG: ");
                            Serial.print(jpeg_filename);
                            Serial.print(" (");
                            Serial.print(written);
                            Serial.println(" bytes)");
                        } else {
                            Serial.print("WARNING: Only wrote ");
                            Serial.print(written);
                            Serial.print(" of ");
                            Serial.print(jpeg_size);
                            Serial.println(" bytes");
                        }
                    } else {
                        Serial.println("ERROR: Failed to save JPEG file");
                    }
                    
                    // Keep JPEG image data for potential YUV conversion
                } else {
                    Serial.println("Failed to capture JPEG image");
                }
            }
        }
        
        Serial.println();
        Serial.println("Preparing ICER input...");
        
        if (!jpeg_img.isAvailable()) {
            Serial.println("ERROR: JPEG capture was not available, cannot run ICER.");
            take_picture_count++;
            return;
        }
        
        const char* y_flash_file = "_y_channel.tmp";
        const char* u_flash_file = "_u_channel.tmp";
        const char* v_flash_file = "_v_channel.tmp";
        
        size_t img_width = 0;
        size_t img_height = 0;
        
        int convert_result = convertJpegToSeparateChannels(
            jpeg_img,
            &img_width, &img_height,
            y_flash_file, u_flash_file, v_flash_file, &theSD
        );
        Serial.println("Channel Separation Complete");
        
        // CRITICAL FIX: Do NOT use assignment operator to "reset" CamImage
        // The original buggy code: jpeg_img = CamImage(); was causing hard faults
        // Instead, let jpeg_img be destroyed naturally when it goes out of scope
        // We'll use a scope block just for the CamImage to ensure it's destroyed
        // before other cleanup operations
        
        // CRITICAL: Add delay after conversion to allow File destructors to complete
        // The debugging showed the fault occurs right at the return statement when
        // File destructors are called. We need to wait for them to complete safely.
        delay(500);
        
        // CRITICAL FIX: Clean up temporary RGB file AFTER all File objects are destroyed
        // The File objects from convertJpegToSeparateChannels are now destroyed,
        // so it's safe to remove the file they were referencing.
        const char* temp_rgb_file = "_temp_rgb.tmp";
        theSD.remove(temp_rgb_file);

        Serial.println("Removed temp file");
        
        printMemoryStats("After JPEG->YUV conversion");
        
        if (convert_result != 0) {
            Serial.print("JPEG conversion failed: ");
            Serial.println(convert_result);
            take_picture_count++;
            return;
        }
        
        Serial.println("JPEG to YUV conversion completed successfully!");
        Serial.print("Image prepared for ICER: ");
        Serial.print(img_width);
        Serial.print("x");
        Serial.print(img_height);
        Serial.println();
        
        size_t img_size = img_width * img_height * 2;  // approximate raw size
        
        // Deinitialize camera to free driver buffers
        Serial.println("Deinitializing camera...");
        theCamera.end();
        delay(200);
        printMemoryStats("After camera.end()");
        
        // Use flash-based ICER compression (minimal RAM usage)
        // This allows us to handle much larger images (e.g., 720p)
        Serial.println("Starting flash-based ICER compression...");
        printMemoryStats("Before flash-based ICER compression");
        unsigned long icer_start_ms = millis();
        
        uint8_t stages = 4;
        uint8_t filter_type = 0;
        uint8_t segments = 6;
        // BAND-AID FIX: Set target_size to prevent buffer overflow
        // With 800 segments and 512 KB buffer, we need:
        // - 25.6 KB for segment headers (800 × 32 bytes)
        // - ~400 KB for segment data (fits in 512 KB buffer)
        // This enables lossy compression that fits in available buffer
        // For lossless, we'd need ~7.3 MB buffer which exceeds available memory
        size_t target_size = 400 * 1024;  // 400 KB - lossy compression to fit in buffer
        
        const char* icer_flash_file = "_icer_result.tmp";
        IcerCompressionResult icer_result = compressYuvWithIcerFlash(
            &theSD,
            y_flash_file, u_flash_file, v_flash_file,
            img_width, img_height,
            stages, filter_type, segments, target_size,
            icer_flash_file,
            false  // Channels are not pre-transformed (we'll do wavelet transform)
        );
        unsigned long icer_elapsed_ms = millis() - icer_start_ms;
        
        // Clean up temporary channel files
        theSD.remove(y_flash_file);
        theSD.remove(u_flash_file);
        theSD.remove(v_flash_file);
        
        printMemoryStats("After flash-based ICER compression");
        
        if (!icer_result.success) {
            Serial.print("ICER compression failed: ");
            Serial.println(icer_result.error_code);
            if (icer_result.flash_filename) {
                theSD.remove(icer_result.flash_filename);
            }
            take_picture_count++;
            return;
        }
        
        Serial.print("ICER compression successful! Size: ");
        Serial.print(icer_result.compressed_size);
        Serial.print(" bytes (");
        Serial.print(icer_result.compressed_size / 1024);
        Serial.print(" KB)");
        Serial.print(" in ");
        Serial.print(icer_elapsed_ms / 1000.0f, 3);
        Serial.print(" s");
        if (img_size > 0) {
            Serial.print(" - ");
            Serial.print((icer_result.compressed_size * 100) / img_size);
            Serial.print("% of original");
        }
        Serial.println();
        
        // Save ICER result to final file
        const char* icer_filename = "CAPTURE.ICER";
        Serial.print("Saving to: ");
        Serial.println(icer_filename);
        
        if (icer_result.flash_filename) {
            // Result is in flash file, copy to final location
            File srcFile = theSD.open(icer_result.flash_filename, FILE_READ);
            if (srcFile) {
                theSD.remove(icer_filename);
                File dstFile = theSD.open(icer_filename, FILE_WRITE);
                if (dstFile) {
                    size_t total_written = 0;
                    uint8_t buffer[512];
                    while (srcFile.available()) {
                        size_t bytes_read = srcFile.read(buffer, sizeof(buffer));
                        if (bytes_read > 0) {
                            size_t bytes_written = dstFile.write(buffer, bytes_read);
                            total_written += bytes_written;
                        }
                    }
                    dstFile.close();
                    srcFile.close();
                    theSD.remove(icer_result.flash_filename);
                    
                    if (total_written == icer_result.compressed_size) {
                        Serial.print("Saved: ");
                        Serial.print(icer_filename);
                        Serial.print(" (");
                        Serial.print(icer_result.compressed_size);
                        Serial.println(" bytes)");
                    } else {
                        Serial.print("WARNING: Only wrote ");
                        Serial.print(total_written);
                        Serial.print(" of ");
                        Serial.print(icer_result.compressed_size);
                        Serial.println(" bytes");
                    }
                } else {
                    Serial.println("ERROR: Failed to open destination file");
                }
            } else {
                Serial.println("ERROR: Failed to open source flash file");
            }
        } else {
            // Result is in RAM
            theSD.remove(icer_filename);
            File icerFile = theSD.open(icer_filename, FILE_WRITE);
            if (icerFile) {
                size_t written = icerFile.write(icer_result.compressed_data, icer_result.compressed_size);
                icerFile.close();
                if (written == icer_result.compressed_size) {
                    Serial.print("Saved: ");
                    Serial.print(icer_filename);
                    Serial.print(" (");
                    Serial.print(icer_result.compressed_size);
                    Serial.println(" bytes)");
                } else {
                    Serial.print("WARNING: Only wrote ");
                    Serial.print(written);
                    Serial.print(" of ");
                    Serial.print(icer_result.compressed_size);
                    Serial.println(" bytes");
                }
            } else {
                Serial.println("ERROR: Failed to save ICER file");
            }
        }
        
        freeIcerCompression(&icer_result);
        printMemoryStats("After freeing ICER result");
        
        take_picture_count++;
        
        Serial.println("----------------------------------------");
        Serial.println("Pipeline complete!");
        Serial.println("========================================");
    } else {
        delay(1000);
    }
}
