# Seekcamera Glass Detector

A thermal imaging application for detecting glass using Seek Thermal cameras and a heat source. Leverages the reflective properties of glass in thermal imaging—detecting glass surfaces by identifying reflected heat sources.

## Features

- **Real-time Thermal Imaging** - Live feed from Seek Thermal cameras
- **Histogram Equalization (CLAHE)** - Professional-grade contrast enhancement with vibrant color gradients
- **Dynamic Temperature Detection** - Adjustable threshold (20-100°C) with interactive slider
- **Bounding Box Visualization** - Color-coded boxes from yellow (warm) to red (hot)
- **Interactive Crosshair** - Real-time temperature readout at cursor position
- **Multiple Object Tracking** - Detects and labels multiple hot objects simultaneously
- **Compact Sidebar UI** - Shows detection summary, center pixel info, and targeted object details

## Building

### Prerequisites

- **CMake** 3.15 or higher
- **Visual Studio 2019/2022** (Windows) or **GCC/Clang** (Linux/macOS)
- **Seek Thermal SDK** 4.4.2 or higher
- **SDL2** 2.0.0 or higher
- **SDL2_ttf** 2.0.0 or higher

### Windows Build

1. **Install dependencies:**
   - Download [Seek Thermal SDK](https://www.thermal.com/developer-center.html)
   - Download [SDL2 Development Libraries](https://github.com/libsdl-org/SDL/releases)
   - Download [SDL2_ttf Development Libraries](https://github.com/libsdl-org/SDL_ttf/releases)

2. **Update paths in CMakeLists.txt:**
   ```cmake
   set(SEEKCAMERA_SDK_DIR "C:/path/to/Seek_Thermal_SDK_4.4.2.20/x64-windows")
   set(SDL2_DIR "C:/SDL2-2.32.10")
   set(SDL2_TTF_DIR "C:/SDL2_ttf-2.24.0")
   ```

3. **Build:**
   ```bash
   mkdir build
   cd build
   cmake ..
   cmake --build . --config Release
   ```

4. **Run:**
   ```bash
   cd Release
   ./seekcamera-glass-detector.exe
   ```

### Linux Build

```bash
# Install dependencies
sudo apt-get install libsdl2-dev libsdl2-ttf-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run
./seekcamera-glass-detector
```

## Usage

Connect your Seek Thermal camera and run:

```bash
./seekcamera-glass-detector
```

### Expected Output

```txt
seekcamera-glass-detector starting
Detected 1 Seek Thermal camera(s)
Using CLAHE histogram equalization
Temperature threshold: 40.0°C

User controls:
  1) Mouse: Point at objects to inspect temperature
  2) Slider: Adjust detection temperature threshold (20-100°C)
  3) Mouse Click: Cycle through color palettes
  4) Q: Quit application
```

## User Controls

### Temperature Threshold Adjustment
- **Drag the slider** in the sidebar to adjust the detection threshold (20-100°C)
- Objects above the threshold will be highlighted with bounding boxes
- Green crosshair appears when pointing at temperatures ≥ threshold

### Interactive Inspection
- **Move mouse cursor** over the thermal image to read temperature at any point
- Crosshair turns **green** when temperature ≥ threshold
- Sidebar shows real-time temperature and color indicator

### Quit
- Press **Q** to exit the application

## Display Layout

```
┌─────────────────────────────────────┬──────────────────┐
│                                     │  Temperature     │
│                                     │  Threshold:      │
│        Thermal Image                │  [═══○═════]     │
│      (with bounding boxes)          │   40.0 C         │
│                                     │                  │
│                                     │  Center Pixel:   │
│                                     │  32.5 C          │
│                                     │  [████████]      │
│                                     │                  │
│                                     │  Targeted:       │
│  41.9 C                             │  42.3 C          │
│  ┌─────────────────┐                │                  │
│  │                 │                │                  │
│  │    Hot Object   │                │  Detected: 3     │
│  │                 │                │  Avg: 42.1 C     │
│  └─────────────────┘                │  Max: 45.3 C     │
│                                     │                  │
└─────────────────────────────────────┴──────────────────┘
```

<img width="1402" height="757" alt="image" src="https://github.com/user-attachments/assets/0781e4b1-5f1d-498e-97ef-9822b9b70f2a" />


## Algorithm Details

### CLAHE Histogram Equalization
The application uses **Contrast Limited Adaptive Histogram Equalization** (CLAHE) for professional thermal imaging:

1. **Histogram Generation** - 256-bin temperature distribution
2. **Plateau Clipping** - Limits contrast at 2.5× average to prevent noise amplification
3. **Pixel Redistribution** - Spreads clipped pixels evenly across bins
4. **CDF Mapping** - Maps temperatures through cumulative distribution function
5. **Result** - Dramatic color separation matching professional thermal cameras

### Object Detection
Connected component analysis with these parameters:
- **Minimum object size**: 15 pixels
- **Temperature threshold**: User-adjustable (20-100°C)
- **Bounding box colors**: Yellow → Orange → Red based on relative temperature

## Configuration

Edit these constants in `src/seekcamera-glass-detector.cpp`:

```cpp
const int MIN_OBJECT_SIZE = 15;        // Minimum pixels for detection
const int SIDEBAR_WIDTH = 300;         // Sidebar width in pixels
const float PLATEAU_LIMIT = 2.5f;      // CLAHE contrast clipping threshold
```

## Project Structure

```
seekcamera-glass-detector/
├── CMakeLists.txt                    # Build configuration
├── README.md                         # This file
├── .gitignore                        # Git ignore rules
├── src/
│   └── seekcamera-glass-detector.cpp # Main application
└── build/                            # Build output (not in git)
    ├── Debug/
    └── Release/
        ├── seekcamera-glass-detector.exe
        ├── seekcamera.dll
        ├── SDL2.dll
        └── SDL2_ttf.dll
```

## Development

### Quick Rebuild After Code Changes
```bash
cmake --build build --config Release
```

### Switch Between Versions
```bash
# Use different source file
cp thermal-tracker-v2.cpp src/seekcamera-glass-detector.cpp
cmake --build build --config Release
```

### Clean Build
```bash
rm -rf build
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

## Technical Specifications

<img width="1050" height="590" alt="image" src="https://github.com/user-attachments/assets/8a37b9be-d667-4fbb-92a5-d4c4ee83f7db" />

- **Frame Rate**: ~30 FPS on 320×240 thermal sensor
- **Temperature Range**: Depends on camera model (typically -40°C to 330°C)
- **Temperature Accuracy**: ±3°C or ±5% (camera-dependent)
- **Display Resolution**: Native sensor resolution (typically 320×240)
- **Supported Cameras**: All Seek Thermal USB cameras (Compact, CompactPRO, etc.)

## Credits

- **Seek Thermal SDK** - Thermal imaging interface
- **SDL2** - Graphics rendering
- **SDL2_ttf** - Text rendering
- **CLAHE Algorithm** - Based on thermal imaging industry standards

## License

This project uses the Seek Thermal SDK. Please refer to Seek Thermal's licensing terms for SDK usage.

## Support

For issues related to:
- **Seek SDK**: Visit [Seek Thermal Developer Center](https://www.thermal.com/developer-center.html)
- **This Application**: Open an issue on GitHub
- **Camera Hardware**: Contact Seek Thermal support
