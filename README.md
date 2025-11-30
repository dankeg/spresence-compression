# Spresense ICER Compression

Real-time JPEG-to-ICER image compression for Spresense camera system.

## Quick Start

1. **Build and upload**:
   ```bash
   pio run -e spresense --target upload
   ```

3. **Monitor output**:
   ```bash
   pio device monitor
   ```

## Features

- Memory-efficient JPEG decoding (STB library, minimal config)
- RGB to YUV color space conversion
- ICER wavelet-based compression
- Memory-aligned buffer allocation
- Real-time heap monitoring (NuttX `mallinfo()` API)

## Documentation

See [ICER_PIPELINE_DOCUMENTATION.md](ICER_PIPELINE_DOCUMENTATION.md) for:
- Complete pipeline architecture and flow
- Memory management and GNSS RAM usage
- Performance characteristics and runtime analysis
- Critical implementation details
- Comprehensive debugging guide
- Known issues and troubleshooting

## Requirements

- PlatformIO
- Spresense board
- SD card
- Camera module
