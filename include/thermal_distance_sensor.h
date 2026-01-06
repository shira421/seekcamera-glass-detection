/**
 * @file thermal_distance_sensor.h
 * @brief Implementation of core detection and ranging functions
 */

#ifndef THERMAL_DISTANCE_SENSOR_H
#define THERMAL_DISTANCE_SENSOR_H

#include <vector>
#include <cmath>
#include <algorithm>

#if defined(__linux__) || defined(__APPLE__)
#   include <SDL2/SDL.h>
#elif defined(_WIN32)
#   define SDL_MAIN_HANDLED
#   include <SDL.h>
#endif

namespace thermal {

constexpr int SIDEBAR_WIDTH = 280;

// Camera specifications (Seek Thermal SD314SPX)
constexpr float HFOV_DEGREES = 56.0f;
constexpr int SENSOR_WIDTH = 320;
constexpr int SENSOR_HEIGHT = 240;

// PCB Grid Physical Dimensions
namespace PCBGrid {
    constexpr float H_SPACING_CM = 0.8f;     // Horizontal dot spacing
    constexpr float V_SPACING_CM = 1.2f;     // Vertical row spacing
    
    // Main grid
    constexpr int COLS = 13;                  // Columns 0-12
    constexpr int ROWS = 3;                   // Rows 0-2
    
    // Dot counts per row
    constexpr int ROW0_DOTS = 13;             // All columns
    constexpr int ROW1_DOTS = 9;              // Cols 0-4 (5) + cols 8-11 (4)
    constexpr int ROW2_DOTS = 13;             // All columns
    constexpr int MAIN_DOTS = 35;             // 13 + 9 + 13
    
    // Anchor points
    constexpr int ANCHOR_DOTS_EACH = 5;       // 5 dots per anchor (diamond)
    constexpr int TOTAL_ANCHORS = 2;
    constexpr int ANCHOR_DOTS = 10;           // 5 × 2
    
    // Total dots
    constexpr int TOTAL_DOTS = 45;            // 35 + 10
    
    // Camera hole in row 1 (cols 5-7 missing)
    constexpr int CAM_COL_START = 5;
    constexpr int CAM_COL_END = 7;
    
    // Anchor positions (in reflected view)
    // Physical left anchor (col 2) appears on right in reflection (col 10)
    // Physical right anchor (col 10) appears on left in reflection (col 2)
    constexpr int ANCHOR_LEFT_COL = 2;        // Right anchor appears here in reflection
    constexpr int ANCHOR_RIGHT_COL = 10;      // Left anchor appears here in reflection
    
    // Anchor geometry
    constexpr float ANCHOR_CENTER_BELOW_CM = 1.2f;  // Center is 1.2cm below row 2
    constexpr float ANCHOR_CORNER_DIST_CM = 0.3f;   // Corner dots 0.3cm from center
    
    // Physical size of main grid
    constexpr float GRID_WIDTH_CM = 12.0f * H_SPACING_CM;   // 12 gaps = 9.6cm
    constexpr float GRID_HEIGHT_CM = 2.0f * V_SPACING_CM;   // 2 gaps = 2.4cm
}

/**
 * @brief A detected/expected dot position
 */
struct GridDot {
    // Grid position
    int row;                // 0-2 for main grid, 3 for anchors
    int col;                // 0-12 for main grid
    int anchor_id;          // 0 = left anchor, 1 = right anchor, -1 = not anchor
    int anchor_pos;         // 0-4 within anchor (0=center, 1-4=corners), -1 = not anchor
    
    // Expected position (normalized, relative to grid center)
    float expected_x_cm;
    float expected_y_cm;
    
    // Detected position (pixels, smoothed)
    float pixel_x;
    float pixel_y;
    float temperature;
    bool detected;
};

/**
 * @brief Result from grid detection
 */
struct GridResult {
    bool valid;             // Detection successful?
    
    // Measurements
    float distance_mm;      // Distance to glass surface in millimeters
    float yaw_deg;          // Camera yaw (+ = pointing right)
    float pitch_deg;        // Camera pitch (+ = pointing up)
    
    // Detection confidence (1.0 = fresh detection, decays during holdover)
    float confidence;
    
    // Grid info
    float grid_width_px;    // Detected grid width in pixels
    float grid_height_px;   // Detected grid height in pixels
    float pixels_per_mm;    // Scale factor
    float grid_center_x;    // Grid center X in image (camera position)
    float grid_center_y;    // Grid center Y in image
    
    // Detected camera hole circle
    float circle_radius;    // Detected circle radius in pixels
    bool circle_found;      // Was the camera circle detected?
    
    // Row bounds
    float main_grid_x_min;  // Left edge X
    float main_grid_x_max;  // Right edge X
    float main_grid_y_min;  // Top row Y
    float main_grid_y_max;  // Bottom row Y
    int num_main_rows;      // Rows detected
    
    // Detection stats
    int total_dots_detected;
    int main_dots_detected;
    int anchor_dots_detected;
    float max_temp;
    float hot_region_temp;
    
    // All 45 dot positions (detected or expected)
    std::vector<GridDot> dots;
    
    // Anchor centers (legacy - not used)
    float left_anchor_x, left_anchor_y;
    float right_anchor_x, right_anchor_y;
    bool left_anchor_valid, right_anchor_valid;
};

/**
 * @brief Main sensor class for PCB grid distance measurement
 */
class PCBGridSensor {
public:
    PCBGridSensor();
    
    /**
     * @brief Hot spot detected in thermal image
     */
    struct HotSpot {
        float x, y;
        float temperature;
    };
    
    /**
     * @brief Process a thermal frame and detect the grid
     */
    GridResult detect(float* temps, int width, int height);
    
    /**
     * @brief Find local temperature maxima (hot spots) - public for use by helpers
     * @param scale Pixels per cm - used to adjust detection radius for close/far distances
     */
    std::vector<HotSpot> findHotSpots(float* temps, int w, int h, float threshold, float scale = 10.0f);

private:
    // Camera parameters
    float vfov_degrees_;
    float focal_length_px_;
    
    // Expected dot positions (45 dots)
    std::vector<GridDot> expected_dots_;
    
    // Smoothed dot positions (persistent across frames)
    std::vector<float> smoothed_x_;
    std::vector<float> smoothed_y_;
    
    // Filter state
    bool initialized_;
    float filt_dist_, filt_yaw_, filt_pitch_;
    float filt_center_x_, filt_center_y_;
    float filt_scale_;
    
    // Smoothed circle position (heavy filtering to reduce jitter)
    float smooth_circle_x_, smooth_circle_y_;
    bool circle_initialized_;
    
    // Filtered bounding box (for stability)
    float filt_x_min_, filt_x_max_, filt_y_min_, filt_y_max_;
    
    // Velocity tracking for adaptive smoothing
    float prev_center_x_, prev_center_y_;
    float velocity_;
    
    // Detection persistence (temporal smoothing)
    int frames_since_detection_;      // Frames since last valid detection
    float detection_confidence_;      // 0.0 to 1.0, decays over time
    static const int MAX_HOLDOVER_FRAMES = 10;  // Hold last position for this many frames
    
    // Median buffers
    std::vector<float> dist_buffer_;
    std::vector<float> yaw_buffer_;
    std::vector<float> pitch_buffer_;
    static const int MEDIAN_SIZE = 1;  // Minimal for maximum responsiveness
    
    /**
     * @brief Initialize the expected 45-dot grid layout
     */
    void initializeExpectedGrid();
    
    /**
     * @brief Match detected hot spots to expected grid positions
     */
    void matchDotsToGrid(const std::vector<HotSpot>& spots, 
                         float center_x, float center_y, float scale,
                         std::vector<GridDot>& dots);
    
    /**
     * @brief Apply heavy smoothing to reduce jitter
     */
    float smooth(float raw, float& filtered, float alpha);
    float median(std::vector<float>& buffer, float new_val);
};

SDL_Color mapTemperature(float temp, float min_temp, float max_temp, bool isolation);

float sharpenPixel(float* temps, int x, int y, int w, int h, float strength);
void sharpenFrame(float* input, float* output, int w, int h, float strength);
SDL_Color mapTemperatureSharp(float temp, float sharpened_temp,
                               float min_temp, float max_temp, bool isolation);

} // namespace thermal

#endif // THERMAL_DISTANCE_SENSOR_H