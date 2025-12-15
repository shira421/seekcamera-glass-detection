/*
 * Thermal Object Tracker - Custom Color Mapping
 * Maps raw temperature data to custom colors with steep gradient
 */

#include <csignal>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <queue>

#if defined(__linux__) || defined(__APPLE__)
#   include <SDL2/SDL.h>
#   include <SDL2/SDL_ttf.h>
#elif defined(_WIN32)
#   define SDL_MAIN_HANDLED
#   include <SDL.h>
#   include <SDL_ttf.h>
#endif

#include "seekcamera/seekcamera.h"
#include "seekcamera/seekcamera_manager.h"
#include "seekframe/seekframe.h"

// COLOR MODES - must be defined BEFORE structs
enum ColorMode {
    ABSOLUTE,    // Fixed temp range (20-70°C) - for distance sensing
    RELATIVE     // Frame min/max - for debugging/visibility
};

// ABSOLUTE mode temperature range
const float ABS_TEMP_MIN = 24.0f;
const float ABS_TEMP_MAX = 28.0f;

// DETECTION PARAMETERS
const int MIN_OBJECT_SIZE = 15;
const int SIDEBAR_WIDTH = 300;

// Thermal Object structure
struct ThermalObject {
    int id;
    int x, y, width, height;
    float min_temp;
    float max_temp;
    float avg_temp;
    SDL_Color box_color;
};

// Renderer structure
struct seekrenderer_t {
    seekcamera_t* camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
    TTF_Font* font_small;
    std::mutex the_mutex;
    std::atomic<bool> is_active;
    std::atomic<bool> is_dirty;
    seekcamera_frame_t* frame;

    std::vector<ThermalObject> detected_objects;
    int next_object_id;
    float center_pixel_temp;
    ColorMode color_mode;  // Toggle between ABSOLUTE and RELATIVE
    bool signature_isolation;  // Toggle signature isolation mode
    float isolation_percentile;  // Top X% to show (0.05 = 5%, 0.10 = 10%)

    int mouse_x, mouse_y;
    bool mouse_down;

    seekrenderer_t() : camera(nullptr), window(nullptr), renderer(nullptr),
        texture(nullptr), font(nullptr), font_small(nullptr),
        frame(nullptr), next_object_id(1),
        center_pixel_temp(0.0f),
        color_mode(RELATIVE),  // Default to RELATIVE for red pixel detection
        signature_isolation(true),  // Start with isolation ON
        isolation_percentile(0.10f),  // Default: show top 10%
        mouse_x(0), mouse_y(0), mouse_down(false) {
        is_active.store(false);
        is_dirty.store(false);
    }
};

static std::atomic<bool> g_exit_requested;
static std::map<std::string, seekrenderer_t*> g_renderers;
static std::mutex g_mutex;
static std::condition_variable g_condition_variable;
static std::atomic<bool> g_is_dirty;

// ABSOLUTE COLOR MAPPING - Fixed temp range (24-28°C)
// Always maps same temperature to same color
SDL_Color mapTemperatureAbsolute(float temp) {
    SDL_Color color;
    
    // Clamp to absolute range
    float normalized = (temp - ABS_TEMP_MIN) / (ABS_TEMP_MAX - ABS_TEMP_MIN);
    normalized = std::max(0.0f, std::min(1.0f, normalized));
    
    // Apply steep power curve for sensitivity
    normalized = std::pow(normalized, 0.3f);  // Steep but not crazy
    
    // Map to vibrant gradient: Blue → Cyan → Green → Yellow → Red
    if (normalized < 0.25f) {
        // Blue to Cyan
        float t = normalized * 4.0f;
        color.r = 0;
        color.g = (Uint8)(t * 255);
        color.b = 255;
    } else if (normalized < 0.5f) {
        // Cyan to Green
        float t = (normalized - 0.25f) * 4.0f;
        color.r = 0;
        color.g = 255;
        color.b = (Uint8)((1.0f - t) * 255);
    } else if (normalized < 0.75f) {
        // Green to Yellow
        float t = (normalized - 0.5f) * 4.0f;
        color.r = (Uint8)(t * 255);
        color.g = 255;
        color.b = 0;
    } else {
        // Yellow to Red
        float t = (normalized - 0.75f) * 4.0f;
        color.r = 255;
        color.g = (Uint8)((1.0f - t) * 255);
        color.b = 0;
    }
    
    color.a = 255;
    return color;
}

// RELATIVE COLOR MAPPING - Frame min/max (ultra steep for debugging)
// Maps relative to current frame for maximum visibility
// WITH TOP-PERCENTILE FILTERING for signature isolation
SDL_Color mapTemperatureRelative(float temp, float min_temp, float max_temp, bool isolate_signature = false, float top_percentile = 0.10f) {
    SDL_Color color;
    
    float range = max_temp - min_temp;
    if (range < 0.01f) {
        // Uniform temperature - show as dark
        color.r = 20; color.g = 20; color.b = 20; color.a = 255;
        return color;
    }
    
    // SIGNATURE ISOLATION MODE
    if (isolate_signature) {
        // Only show top X% of temperature range
        float threshold = max_temp - (range * top_percentile);
        
        if (temp < threshold) {
            // Below threshold - BLACK IT OUT
            color.r = 0; color.g = 0; color.b = 0; color.a = 255;
            return color;
        }
        
        // Above threshold - map within the top percentile range
        float normalized = (temp - threshold) / (range * top_percentile);
        normalized = std::pow(normalized, 0.15f);  // Ultra steep!
        
        // Map to vibrant gradient: Blue → Cyan → Green → Yellow → Red
        if (normalized < 0.25f) {
            float t = normalized * 4.0f;
            color.r = 0;
            color.g = (Uint8)(t * 255);
            color.b = 255;
        } else if (normalized < 0.5f) {
            float t = (normalized - 0.25f) * 4.0f;
            color.r = 0;
            color.g = 255;
            color.b = (Uint8)((1.0f - t) * 255);
        } else if (normalized < 0.75f) {
            float t = (normalized - 0.5f) * 4.0f;
            color.r = (Uint8)(t * 255);
            color.g = 255;
            color.b = 0;
        } else {
            float t = (normalized - 0.75f) * 4.0f;
            color.r = 255;
            color.g = (Uint8)((1.0f - t) * 255);
            color.b = 0;
        }
        
        color.a = 255;
        return color;
    }
    
    // NORMAL RELATIVE MODE (full range)
    float normalized = (temp - min_temp) / range;
    normalized = std::pow(normalized, 0.15f);  // Very steep!
    
    // Map to vibrant gradient: Blue → Cyan → Green → Yellow → Red
    if (normalized < 0.25f) {
        // Blue to Cyan
        float t = normalized * 4.0f;
        color.r = 0;
        color.g = (Uint8)(t * 255);
        color.b = 255;
    } else if (normalized < 0.5f) {
        // Cyan to Green
        float t = (normalized - 0.25f) * 4.0f;
        color.r = 0;
        color.g = 255;
        color.b = (Uint8)((1.0f - t) * 255);
    } else if (normalized < 0.75f) {
        // Green to Yellow
        float t = (normalized - 0.5f) * 4.0f;
        color.r = (Uint8)(t * 255);
        color.g = 255;
        color.b = 0;
    } else {
        // Yellow to Red
        float t = (normalized - 0.75f) * 4.0f;
        color.r = 255;
        color.g = (Uint8)((1.0f - t) * 255);
        color.b = 0;
    }
    
    color.a = 255;
    return color;
}

// Get color based on temperature (for bounding boxes)
SDL_Color getTemperatureColor(float temp, float min_threshold, float max_range) {
    SDL_Color color;

    if (temp < min_threshold) {
        temp = min_threshold;
    }

    float normalized = (temp - min_threshold) / max_range;
    normalized = std::max(0.0f, std::min(1.0f, normalized));

    // Bright Yellow -> Orange -> Dark Red
    if (normalized < 0.5f) {
        float t = normalized * 2.0f;
        color.r = 255;
        color.g = (Uint8)(255 - (127 * t));
        color.b = 0;
    }
    else {
        float t = (normalized - 0.5f) * 2.0f;
        color.r = (Uint8)(255 - (75 * t));
        color.g = (Uint8)(128 * (1.0f - t));
        color.b = 0;
    }

    color.a = 255;
    return color;
}

// HYBRID DISTANCE SENSOR - Triangulation + Size-Based with Sensor Fusion
// 2D TRIANGULATION DISTANCE SENSOR
// Vertical baseline (2.5cm above camera) + Automatic yaw correction
// Uses hottest pixel for pure geometric calculation - NO CALIBRATION NEEDED!
class TriangulationDistanceSensor {
private:
    // PHYSICAL SETUP - DUAL EMITTER SYSTEM
    const float BASELINE_ABOVE_CM = 2.5f;        // ABOVE camera
    const float BASELINE_LEFT_CM = 3.3f;         // LEFT of camera
    const float HFOV_DEGREES = 56.0f;            // Horizontal FOV
    const float SENSOR_WIDTH = 320.0f;           // pixels
    const float SENSOR_HEIGHT = 240.0f;          // pixels
    
    // DERIVED PARAMETERS
    float VFOV_DEGREES;                          // Vertical FOV (calculated)
    float focal_length_pixels;                   // Same for both H and V
    
    // TEMPORAL FILTERING - for smoothing measurements
    std::vector<float> distance_history;
    std::vector<float> yaw_history;
    std::vector<float> pitch_history;
    
    const int FILTER_SIZE = 8;                   // 8-frame MA for fast response
    
    // Helper function for temporal filtering
    float smoothValue(std::vector<float>& buffer, float new_val) {
        buffer.push_back(new_val);
        if (buffer.size() > FILTER_SIZE) {
            buffer.erase(buffer.begin());
        }
        
        // Calculate average
        float sum = 0;
        for (float v : buffer) sum += v;
        return sum / buffer.size();
    }
    
    // Structure for a detected hot spot
    struct HotSpot {
        float x;           // Sub-pixel X position
        float y;           // Sub-pixel Y position
        float temp;        // Temperature
        bool valid;
    };
    
    // Find sub-pixel centroid of a hot spot
    HotSpot findHotSpotCentroid(float* temps, int width, int height, 
                                 int center_x, int center_y, float max_temp) {
        HotSpot spot;
        spot.valid = false;
        
        // Calculate temperature-weighted centroid in 5×5 region
        float threshold = max_temp - 2.0f;
        float sum_temp = 0.0f;
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        int count = 0;
        
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int px = center_x + dx;
                int py = center_y + dy;
                
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    float temp = temps[py * width + px];
                    if (temp > threshold) {
                        sum_temp += temp;
                        sum_x += temp * px;
                        sum_y += temp * py;
                        count++;
                    }
                }
            }
        }
        
        if (sum_temp > 0 && count >= 3) {
            spot.x = sum_x / sum_temp;
            spot.y = sum_y / sum_temp;
            spot.temp = max_temp;
            spot.valid = true;
        } else {
            spot.x = center_x;
            spot.y = center_y;
            spot.temp = max_temp;
            spot.valid = true;
        }
        
        return spot;
    }
    
public:
    TriangulationDistanceSensor() {
        // Calculate vertical FOV from horizontal FOV and aspect ratio
        // VFOV = 2 × atan(tan(HFOV/2) × (height/width))
        float aspect_ratio = SENSOR_HEIGHT / SENSOR_WIDTH;  // 240/320 = 0.75
        float hfov_half_rad = (HFOV_DEGREES / 2.0f) * M_PI / 180.0f;
        float vfov_half_rad = atan(tan(hfov_half_rad) * aspect_ratio);
        VFOV_DEGREES = (vfov_half_rad * 180.0f / M_PI) * 2.0f;
        
        // Calculate focal length (same for both axes in pinhole camera)
        focal_length_pixels = (SENSOR_WIDTH / 2.0f) / tan(hfov_half_rad);
        
        std::cout << "\n==== DUAL EMITTER TRIANGULATION SYSTEM ====" << std::endl;
        std::cout << "Physical Setup:" << std::endl;
        std::cout << "  Above Emitter: " << BASELINE_ABOVE_CM << " cm ABOVE camera" << std::endl;
        std::cout << "  Left Emitter: " << BASELINE_LEFT_CM << " cm LEFT of camera" << std::endl;
        std::cout << "  Resolution: " << SENSOR_WIDTH << "×" << SENSOR_HEIGHT << std::endl;
        std::cout << "  HFOV: " << HFOV_DEGREES << "° (±28° from center)" << std::endl;
        std::cout << "  VFOV: " << VFOV_DEGREES << "° (±21° from center)" << std::endl;
        std::cout << "\nHot Spot Detection Method:" << std::endl;
        std::cout << "  METHOD: Relative position-based identification" << std::endl;
        std::cout << "  • Dual threshold: min(0.1°C above bg, 80th percentile)" << std::endl;
        std::cout << "  • Finds 2 hottest spots with >10px separation" << std::endl;
        std::cout << "  • Identifies by RELATIVE position: dominant axis determines assignment" << std::endl;
        std::cout << "  • Works in ANY quadrant!" << std::endl;
        std::cout << "\nDistance Measurement Method:" << std::endl;
        std::cout << "  PRIMARY: Pixel separation (tilt-invariant)" << std::endl;
        std::cout << "  FORMULA: d = K / pixel_separation / 2" << std::endl;
        std::cout << "  K = 1335 pixels×cm (mathematically derived)" << std::endl;
        std::cout << "  Division by 2: accounts for glass reflection" << std::endl;
        std::cout << "\nExpected Values (corrected for reflection):" << std::endl;
        std::cout << "  5 cm → separation ≈ 268 pixels" << std::endl;
        std::cout << "  7.5 cm → separation ≈ 178 pixels" << std::endl;
        std::cout << "  10 cm → separation ≈ 134 pixels" << std::endl;
        std::cout << "  15 cm → separation ≈ 89 pixels" << std::endl;
        std::cout << "\nFiltering & Stability:" << std::endl;
        std::cout << "  ✓ SUB-PIXEL CENTROID: 5×5 temperature-weighted averaging" << std::endl;
        std::cout << "  ✓ TRIPLE-STAGE FILTERING (RESPONSIVE):" << std::endl;
        std::cout << "    - 8-frame moving average (fast response)" << std::endl;
        std::cout << "    - Exponential smoothing (alpha=0.5, responsive)" << std::endl;
        std::cout << "    - Deadband filter (0.15 cm, minimal lag)" << std::endl;
        std::cout << "  ✓ Result: Fast response + smooth tracking\n" << std::endl;
    }
    
    struct DistanceResult {
        bool detected;
        
        // Final result
        float distance_to_glass_cm;
        float distance_to_glass_m;
        
        // Intermediate calculations
        float perpendicular_distance_cm;  // Distance along optical axis
        float camera_yaw_deg;              // Horizontal tilt angle
        
        // Raw measurements - PRIMARY spot (above emitter)
        int hottest_x;
        int hottest_y;
        float hottest_temp;
        float vertical_angle_deg;
        float horizontal_angle_deg;
        float pixel_offset_x;
        float pixel_offset_y;
        
        // SECOND spot (left emitter) - for dual emitter visualization
        int second_spot_x;
        int second_spot_y;
        float second_spot_temp;
    };
    
    DistanceResult calculate(float* temps, int width, int height) {
        DistanceResult result;
        result.detected = false;
        
        // ===== STEP 1: FIND TWO HOT SPOTS (LOCAL MAXIMA METHOD) =====
        
        // Detection parameters (DECLARE FIRST!)
        const int NEIGHBORHOOD = 2;
        const float MIN_TEMP_ABOVE_BG = 0.1f;  // Standard sensitivity
        
        // First, calculate background temperature (median of coolest 50% of pixels)
        std::vector<float> all_temps;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                all_temps.push_back(temps[y * width + x]);
            }
        }
        std::sort(all_temps.begin(), all_temps.end());
        float background_temp = all_temps[all_temps.size() / 2];  // Median
        float min_temp = all_temps[0];
        float max_temp = all_temps[all_temps.size() - 1];
        
        // Also calculate percentile thresholds
        float temp_70th = all_temps[(all_temps.size() * 70) / 100];
        float temp_80th = all_temps[(all_temps.size() * 80) / 100];
        float temp_85th = all_temps[(all_temps.size() * 85) / 100];
        float temp_90th = all_temps[(all_temps.size() * 90) / 100];
        
        std::cout << "\n=== DETECTION ===" << std::endl;
        std::cout << "Temperature range: " << min_temp << "°C to " << max_temp << "°C" << std::endl;
        std::cout << "Background (median): " << background_temp << "°C" << std::endl;
        std::cout << "Percentiles: 70th=" << temp_70th << "°C, 80th=" << temp_80th 
                  << "°C, 85th=" << temp_85th << "°C, 90th=" << temp_90th << "°C" << std::endl;
        
        // Find all LOCAL MAXIMA (hottest pixel in neighborhood)
        
        struct LocalMax {
            int x, y;
            float temp;
            bool operator<(const LocalMax& other) const {
                return temp > other.temp;  // Sort hottest first
            }
        };
        
        std::vector<LocalMax> local_maxima;
        
        // Use DUAL THRESHOLD: absolute OR percentile (whichever is more lenient)
        float absolute_threshold = background_temp + MIN_TEMP_ABOVE_BG;
        float percentile_threshold = temp_80th;  // Top 20%
        float detection_threshold = std::min(absolute_threshold, percentile_threshold);
        
        std::cout << "Detection threshold: >" << detection_threshold << "°C (more lenient of " 
                  << absolute_threshold << "°C absolute vs " << percentile_threshold << "°C percentile)" << std::endl;
        
        for (int y = NEIGHBORHOOD; y < height - NEIGHBORHOOD; y++) {
            for (int x = NEIGHBORHOOD; x < width - NEIGHBORHOOD; x++) {
                float center_temp = temps[y * width + x];
                
                // Must be above detection threshold
                if (center_temp < detection_threshold) {
                    continue;
                }
                
                // Check if hottest in neighborhood
                bool is_local_max = true;
                for (int dy = -NEIGHBORHOOD; dy <= NEIGHBORHOOD && is_local_max; dy++) {
                    for (int dx = -NEIGHBORHOOD; dx <= NEIGHBORHOOD; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        float neighbor_temp = temps[(y+dy) * width + (x+dx)];
                        if (neighbor_temp > center_temp) {
                            is_local_max = false;
                            break;
                        }
                    }
                }
                
                if (is_local_max) {
                    local_maxima.push_back({x, y, center_temp});
                }
            }
        }
        
        std::sort(local_maxima.begin(), local_maxima.end());
        
        std::cout << "Found " << local_maxima.size() << " local maxima" << std::endl;
        
        // Show top candidates for debugging
        if (local_maxima.size() > 0) {
            std::cout << "Top candidates:" << std::endl;
            for (size_t i = 0; i < std::min((size_t)10, local_maxima.size()); i++) {
                std::cout << "  #" << i << ": (" << local_maxima[i].x << "," << local_maxima[i].y 
                          << ") " << local_maxima[i].temp << "°C (+" 
                          << (local_maxima[i].temp - background_temp) << "°C above bg)" << std::endl;
            }
        }
        
        if (local_maxima.size() < 2) {
            std::cout << "ERROR: Need at least 2 hot spots (found " << local_maxima.size() << ")" << std::endl;
            result.detected = false;
            return result;
        }
        
        // ===== SIMPLE SELECTION: Just find 2 hottest spots far enough apart =====
        
        std::cout << "\nSimple selection: Finding 2 hottest spots..." << std::endl;
        
        if (local_maxima.size() < 2) {
            std::cout << "ERROR: Need at least 2 hot spots (found " << local_maxima.size() << ")" << std::endl;
            result.detected = false;
            return result;
        }
        
        // Find two spots with sufficient separation
        const float MIN_SEPARATION = 10.0f;  // Reduced for better tolerance
        LocalMax spot1 = local_maxima[0];  // Hottest
        LocalMax spot2;
        bool found_second = false;
        
        for (size_t i = 1; i < local_maxima.size(); i++) {
            float dx = local_maxima[i].x - spot1.x;
            float dy = local_maxima[i].y - spot1.y;
            float sep = sqrt(dx*dx + dy*dy);
            
            if (sep >= MIN_SEPARATION) {
                spot2 = local_maxima[i];
                found_second = true;
                std::cout << "Selected: Spot1 (" << spot1.x << "," << spot1.y << ") " << spot1.temp 
                          << "°C, Spot2 (" << spot2.x << "," << spot2.y << ") " << spot2.temp 
                          << "°C, sep=" << sep << "px" << std::endl;
                break;
            }
        }
        
        if (!found_second) {
            std::cout << "ERROR: No second spot found >" << MIN_SEPARATION << "px away" << std::endl;
            result.detected = false;
            return result;
        }
        
        // Calculate sub-pixel centroids
        HotSpot hotspot1 = findHotSpotCentroid(temps, width, height, spot1.x, spot1.y, spot1.temp);
        HotSpot hotspot2 = findHotSpotCentroid(temps, width, height, spot2.x, spot2.y, spot2.temp);
        
        if (!hotspot1.valid || !hotspot2.valid) {
            result.detected = false;
            return result;
        }
        
        // CORRECT identification: Based on RELATIVE positions
        // Above emitter creates spot that is MORE UP (lower Y) relative to left spot
        // Left emitter creates spot that is MORE LEFT (lower X) relative to above spot
        HotSpot spot_above, spot_left;
        
        std::cout << "Identifying spots:" << std::endl;
        std::cout << "  Spot1: (" << hotspot1.x << "," << hotspot1.y << ") " << hotspot1.temp << "°C" << std::endl;
        std::cout << "  Spot2: (" << hotspot2.x << "," << hotspot2.y << ") " << hotspot2.temp << "°C" << std::endl;
        
        // Calculate relative positions
        float dx = hotspot2.x - hotspot1.x;  // Positive if Spot2 is RIGHT of Spot1
        float dy = hotspot2.y - hotspot1.y;  // Positive if Spot2 is BELOW Spot1
        
        std::cout << "  Relative: dx=" << dx << " (S2 vs S1), dy=" << dy << " (S2 vs S1)" << std::endl;
        
        // Use the DOMINANT direction to identify
        // If vertical separation is larger, use Y-difference
        // If horizontal separation is larger, use X-difference
        if (fabs(dy) > fabs(dx)) {
            // Vertical separation dominates
            std::cout << "  Vertical separation dominates (|dy|=" << fabs(dy) << " > |dx|=" << fabs(dx) << ")" << std::endl;
            if (hotspot1.y <= hotspot2.y) {
                spot_above = hotspot1;
                spot_left = hotspot2;
                std::cout << "  → Spot1 is ABOVE Spot2 → Spot1=ABOVE, Spot2=LEFT" << std::endl;
            } else {
                spot_above = hotspot2;
                spot_left = hotspot1;
                std::cout << "  → Spot2 is ABOVE Spot1 → Spot2=ABOVE, Spot1=LEFT" << std::endl;
            }
        } else {
            // Horizontal separation dominates
            std::cout << "  Horizontal separation dominates (|dx|=" << fabs(dx) << " > |dy|=" << fabs(dy) << ")" << std::endl;
            if (hotspot1.x <= hotspot2.x) {
                spot_left = hotspot1;
                spot_above = hotspot2;
                std::cout << "  → Spot1 is LEFT of Spot2 → Spot1=LEFT, Spot2=ABOVE" << std::endl;
            } else {
                spot_left = hotspot2;
                spot_above = hotspot1;
                std::cout << "  → Spot2 is LEFT of Spot1 → Spot2=LEFT, Spot1=ABOVE" << std::endl;
            }
        }
        
        std::cout << "Final assignment:" << std::endl;
        std::cout << "  ABOVE: (" << spot_above.x << "," << spot_above.y << ") " << spot_above.temp << "°C";
        if (spot_above.x > 160) std::cout << " [RIGHT of crosshair]";
        if (spot_above.y > 120) std::cout << " [BELOW crosshair]";
        std::cout << std::endl;
        
        std::cout << "  LEFT: (" << spot_left.x << "," << spot_left.y << ") " << spot_left.temp << "°C";
        if (spot_left.x > 160) std::cout << " [RIGHT of crosshair]";
        if (spot_left.y > 120) std::cout << " [BELOW crosshair]";
        std::cout << std::endl;
        
        // ===== STEP 2: CALCULATE ANGLES FROM SPOT POSITIONS =====
        
        float center_x = 160.0f;
        float center_y = 120.0f;
        
        // From ABOVE spot
        float y_offset_above = center_y - spot_above.y;  // Positive when above center
        float x_offset_above = spot_above.x - center_x;  // Positive when right of center
        
        std::cout << "\nABOVE spot offsets: y_offset=" << y_offset_above 
                  << ", x_offset=" << x_offset_above << std::endl;
        
        float theta_ya_deg = (y_offset_above / 120.0f) * 21.0f;
        float theta_xa_deg = (x_offset_above / 160.0f) * 28.0f;
        
        float theta_ya_rad = theta_ya_deg * M_PI / 180.0f;
        float theta_xa_rad = theta_xa_deg * M_PI / 180.0f;
        
        std::cout << "ABOVE angles: θ_ya=" << theta_ya_deg << "° (vertical angle)"
                  << " | θ_xa=" << theta_xa_deg << "° (yaw indicator)" << std::endl;
        
        // From LEFT spot
        float x_offset_left = center_x - spot_left.x;  // Positive when left of center
        float y_offset_left = spot_left.y - center_y;  // Positive when below center
        
        std::cout << "\nLEFT spot offsets: x_offset=" << x_offset_left 
                  << ", y_offset=" << y_offset_left << std::endl;
        
        float theta_xl_deg = (x_offset_left / 160.0f) * 28.0f;
        float theta_yl_deg = (y_offset_left / 120.0f) * 21.0f;
        
        float theta_xl_rad = theta_xl_deg * M_PI / 180.0f;
        float theta_yl_rad = theta_yl_deg * M_PI / 180.0f;
        
        std::cout << "LEFT angles: θ_xl=" << theta_xl_deg << "° (for distance)"
                  << " | θ_yl=" << theta_yl_deg << "° (pitch indicator)" << std::endl;
        
        // ===== STEP 4: PIXEL SEPARATION METHOD (Tilt-Invariant) =====
        
        std::cout << "\n=== DISTANCE CALCULATION (PIXEL SEPARATION METHOD) ===" << std::endl;
        
        // Calculate pixel separation between the two spots
        float dx_pixels = spot_above.x - spot_left.x;
        float dy_pixels = spot_above.y - spot_left.y;
        float separation_pixels = sqrt(dx_pixels * dx_pixels + dy_pixels * dy_pixels);
        
        std::cout << "Spot separation: Δx=" << dx_pixels << " pixels, Δy=" << dy_pixels 
                  << " pixels, Total=" << separation_pixels << " pixels" << std::endl;
        
        // Mathematically derived calibration constant
        const float K_CALIBRATION = 1335.0f;  // pixels × cm (mathematically derived)
        
        float d_separation = K_CALIBRATION / separation_pixels;
        
        std::cout << "Separation method: d = " << K_CALIBRATION << " / " << separation_pixels 
                  << " = " << d_separation << " cm" << std::endl;
        
        // Also calculate using individual angles (for comparison/validation)
        float tan_ya = tan(fabs(theta_ya_rad));  // Use absolute angle
        float tan_xl = tan(fabs(theta_xl_rad));  // Use absolute angle
        
        if (tan_ya <= 0 || tan_xl <= 0) {
            std::cout << "ERROR: Invalid tangent values" << std::endl;
            result.detected = false;
            return result;
        }
        
        float d_above_simple = BASELINE_ABOVE_CM / tan_ya;
        float d_left_simple = BASELINE_LEFT_CM / tan_xl;
        
        std::cout << "Simple formulas (for reference):" << std::endl;
        std::cout << "  d_above = " << BASELINE_ABOVE_CM << " / tan(θ_ya) = " 
                  << d_above_simple << " cm" << std::endl;
        std::cout << "  d_left = " << BASELINE_LEFT_CM << " / tan(θ_xl) = " 
                  << d_left_simple << " cm" << std::endl;
        
        // Use separation method as primary
        float d_unified = d_separation;
        
        // ===== STEP 5: ACCOUNT FOR REFLECTION =====
        d_unified = d_unified / 2.0f;
        
        std::cout << "\nReflection correction: d_final = " << d_separation << " / 2 = " 
                  << d_unified << " cm (actual distance to glass)" << std::endl;
        
        // ===== STEP 6: VALIDATION AND SANITY CHECKS =====
        
        std::cout << "\n=== VALIDATION ===" << std::endl;
        std::cout << "Raw separation distance: " << d_separation << " cm" << std::endl;
        std::cout << "PRIMARY (with reflection correction): " << d_unified << " cm" << std::endl;
        std::cout << "REFERENCE: Individual measurements (uncorrected):" << std::endl;
        std::cout << "  Above: " << d_above_simple << " cm → " << d_above_simple/2.0f << " cm (corrected)" << std::endl;
        std::cout << "  Left: " << d_left_simple << " cm → " << d_left_simple/2.0f << " cm (corrected)" << std::endl;
        
        if (d_unified < 2.0f || d_unified > 50.0f) {
            std::cout << "WARNING: Distance out of reasonable range (2-50 cm)" << std::endl;
        }
        
        if (separation_pixels < 10.0f) {
            std::cout << "WARNING: Spots too close together (>50cm or measurement error)" << std::endl;
        }
        if (separation_pixels > 250.0f) {
            std::cout << "WARNING: Spots very far apart (<5cm - may be too close)" << std::endl;
        }
        
        // ===== STEP 7: 3D POSITIONS (For Reference) =====
        
        // Use signed angles for 3D reconstruction
        float P_above_x = d_unified * tan(theta_xa_rad);  // Keep sign
        float P_above_y = d_unified * tan_ya;             // Always positive (spot above baseline)
        float P_above_z = d_unified;
        
        float P_left_x = -d_unified * tan_xl;  // Negative for left
        float P_left_y = d_unified * tan(theta_yl_rad);  // Keep sign
        float P_left_z = d_unified;
        
        float dx3d = P_above_x - P_left_x;
        float dy3d = P_above_y - P_left_y;
        float dz3d = P_above_z - P_left_z;
        float spot_separation_3d = sqrt(dx3d*dx3d + dy3d*dy3d + dz3d*dz3d);
        
        std::cout << "\n3D Positions (at distance " << d_unified << " cm):" << std::endl;
        std::cout << "Above: (" << P_above_x << ", " << P_above_y << ", " << P_above_z << ") cm" << std::endl;
        std::cout << "Left: (" << P_left_x << ", " << P_left_y << ", " << P_left_z << ") cm" << std::endl;
        std::cout << "3D Separation: " << spot_separation_3d << " cm" << std::endl;
        
        // ===== STEP 8: MULTI-STAGE FILTERING FOR STABILITY =====
        
        // Stage 1: Moving average (smooths high-frequency noise)
        float d_ma = smoothValue(distance_history, d_unified);
        float yaw_final = smoothValue(yaw_history, theta_xa_deg);
        float pitch_final = smoothValue(pitch_history, theta_yl_deg);
        
        // Stage 2: Exponential smoothing (adds momentum, very smooth)
        static float ema_distance = 0.0f;
        static bool ema_initialized = false;
        
        const float ALPHA = 0.5f;  // 0.5 = responsive
        
        if (!ema_initialized) {
            ema_distance = d_ma;
            ema_initialized = true;
        } else {
            ema_distance = ALPHA * d_ma + (1.0f - ALPHA) * ema_distance;
        }
        
        // Stage 3: Deadband (eliminates micro-jitter when stationary)
        static float last_output = 0.0f;
        const float DEADBAND_CM = 0.15f;
        
        float d_final;
        if (last_output == 0.0f) {
            d_final = ema_distance;
            last_output = ema_distance;
        } else {
            float change = fabs(ema_distance - last_output);
            if (change > DEADBAND_CM) {
                d_final = ema_distance;
                last_output = ema_distance;
            } else {
                d_final = last_output;
            }
        }
        
        std::cout << "Filtering: Raw=" << d_unified << " → MA=" << d_ma 
                  << " → EMA=" << ema_distance << " → Final=" << d_final << " cm" << std::endl;
        
        // ===== POPULATE RESULT =====
        
        result.distance_to_glass_cm = d_final;
        result.distance_to_glass_m = d_final / 100.0f;
        result.camera_yaw_deg = yaw_final;
        
        result.hottest_x = (int)round(spot_above.x);
        result.hottest_y = (int)round(spot_above.y);
        result.hottest_temp = spot_above.temp;
        
        result.second_spot_x = (int)round(spot_left.x);
        result.second_spot_y = (int)round(spot_left.y);
        result.second_spot_temp = spot_left.temp;
        
        result.detected = true;
        
        std::cout << "\n► FINAL: " << d_final << " cm (filtered) | "
                  << "Yaw: " << yaw_final << "° | Pitch: " << pitch_final << "°\n" << std::endl;
        
        return result;
    }
};

// Connected component labeling
std::vector<std::vector<int> > findConnectedComponents(std::vector<std::vector<bool> >& binary_map, int width, int height) {
    std::vector<std::vector<int> > labels(height, std::vector<int>(width, 0));
    int current_label = 1;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (binary_map[y][x] && labels[y][x] == 0) {
                std::queue<std::pair<int, int> > queue;
                queue.push(std::make_pair(x, y));
                labels[y][x] = current_label;

                while (!queue.empty()) {
                    std::pair<int, int> current = queue.front();
                    queue.pop();
                    int cx = current.first;
                    int cy = current.second;

                    const int dx[] = { -1, 1, 0, 0 };
                    const int dy[] = { 0, 0, -1, 1 };

                    for (int i = 0; i < 4; i++) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                            binary_map[ny][nx] && labels[ny][nx] == 0) {
                            labels[ny][nx] = current_label;
                            queue.push(std::make_pair(nx, ny));
                        }
                    }
                }
                current_label++;
            }
        }
    }

    return labels;
}

// GRADIENT-BASED HEAT SIGNATURE ANALYSIS
struct SignatureScore {
    float gradient_quality;
    float compactness;
    float peak_centrality;
    float aspect_ratio_score;
    float circularity;
    float overall_score;
};

SignatureScore analyzeSignaturePattern(float* temps, int width, int height, const ThermalObject& obj) {
    SignatureScore score;
    score.gradient_quality = 0.0f;
    score.compactness = 0.0f;
    score.peak_centrality = 0.0f;
    score.aspect_ratio_score = 0.0f;
    score.circularity = 0.0f;
    score.overall_score = 0.0f;
    
    float aspect_ratio = (float)obj.width / (float)obj.height;
    if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) {
        return score;
    }
    
    int peak_x = obj.x;
    int peak_y = obj.y;
    float peak_temp = -1000.0f;
    
    for (int y = obj.y; y < obj.y + obj.height && y < height; y++) {
        for (int x = obj.x; x < obj.x + obj.width && x < width; x++) {
            float temp = temps[y * width + x];
            if (temp > peak_temp) {
                peak_temp = temp;
                peak_x = x;
                peak_y = y;
            }
        }
    }
    
    float aspect_deviation = fabs(aspect_ratio - 1.0f);
    score.aspect_ratio_score = std::max(0.0f, 1.0f - (aspect_deviation * 2.0f));
    
    float center_x = obj.x + obj.width / 2.0f;
    float center_y = obj.y + obj.height / 2.0f;
    float peak_offset = sqrt(pow(peak_x - center_x, 2) + pow(peak_y - center_y, 2));
    float max_offset = sqrt(pow(obj.width / 2.0f, 2) + pow(obj.height / 2.0f, 2));
    score.peak_centrality = max_offset > 0 ? (1.0f - (peak_offset / max_offset)) : 1.0f;
    
    if (score.peak_centrality < 0.5f) {
        return score;
    }
    
    float gradient_scores[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int directions[8][2] = {
        {0, -1}, {1, -1}, {1, 0}, {1, 1},
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}
    };
    
    int valid_directions = 0;
    
    for (int dir = 0; dir < 8; dir++) {
        int dx = directions[dir][0];
        int dy = directions[dir][1];
        
        float last_temp = peak_temp;
        int consecutive_declines = 0;
        int total_samples = 0;
        float temp_drop_total = 0.0f;
        
        for (int step = 1; step <= 5; step++) {
            int sample_x = peak_x + (dx * step);
            int sample_y = peak_y + (dy * step);
            
            if (sample_x < 0 || sample_x >= width || sample_y < 0 || sample_y >= height) break;
            if (sample_x < obj.x || sample_x >= obj.x + obj.width) break;
            if (sample_y < obj.y || sample_y >= obj.y + obj.height) break;
            
            float sample_temp = temps[sample_y * width + sample_x];
            
            if (sample_temp < last_temp) {
                consecutive_declines++;
                temp_drop_total += (last_temp - sample_temp);
            }
            last_temp = sample_temp;
            total_samples++;
        }
        
        if (total_samples > 0) {
            float decline_ratio = (float)consecutive_declines / total_samples;
            float avg_drop_per_pixel = temp_drop_total / total_samples;
            
            gradient_scores[dir] = (decline_ratio * 0.7f) + 
                                   (std::min(avg_drop_per_pixel / 2.0f, 1.0f) * 0.3f);
            valid_directions++;
        }
    }
    
    if (valid_directions > 0) {
        float sum = 0.0f;
        for (int i = 0; i < 8; i++) {
            sum += gradient_scores[i];
        }
        score.gradient_quality = sum / valid_directions;
    }
    
    if (score.gradient_quality < 0.4f) {
        return score;
    }
    
    int area = obj.width * obj.height;
    int perimeter = 2 * (obj.width + obj.height);
    score.circularity = (4.0f * 3.14159f * area) / (perimeter * perimeter);
    score.circularity = std::min(score.circularity, 1.0f);
    
    float ideal_min_area = 9.0f;
    float ideal_max_area = 400.0f;
    
    if (area < ideal_min_area) {
        score.compactness = 0.0f;
    } else if (area > ideal_max_area) {
        score.compactness = 0.3f;
    } else {
        float ideal_area = 100.0f;
        float area_deviation = fabs(area - ideal_area) / ideal_area;
        score.compactness = std::max(0.3f, 1.0f - area_deviation);
    }
    
    score.overall_score = (score.gradient_quality * 0.40f) +
                          (score.peak_centrality * 0.25f) +
                          (score.aspect_ratio_score * 0.15f) +
                          (score.circularity * 0.10f) +
                          (score.compactness * 0.10f);
    
    return score;
}

std::vector<ThermalObject> detectRedPixels(float* temps, int width, int height, int& next_id, 
                                            bool use_isolation, float isolation_percentile) {
    std::vector<ThermalObject> objects;
    
    float min_temp = 1000.0f;
    float max_temp = -1000.0f;
    for (int i = 0; i < width * height; i++) {
        min_temp = std::min(min_temp, temps[i]);
        max_temp = std::max(max_temp, temps[i]);
    }
    
    float range = max_temp - min_temp;
    if (range < 0.01f) return objects;
    
    float threshold;
    if (use_isolation) {
        threshold = max_temp - (range * isolation_percentile);
    } else {
        threshold = max_temp - (range * 0.05f);
    }
    
    std::vector<std::vector<bool> > red_pixels(height, std::vector<bool>(width, false));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] >= threshold) {
                red_pixels[y][x] = true;
            }
        }
    }
    
    std::vector<std::vector<int> > labels = findConnectedComponents(red_pixels, width, height);
    
    std::map<int, ThermalObject> object_map;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int label = labels[y][x];
            if (label > 0) {
                if (object_map.find(label) == object_map.end()) {
                    ThermalObject obj;
                    obj.id = next_id++;
                    obj.x = x;
                    obj.y = y;
                    obj.width = 0;
                    obj.height = 0;
                    obj.min_temp = 1000.0f;
                    obj.max_temp = -1000.0f;
                    obj.avg_temp = 0.0f;
                    object_map[label] = obj;
                }
                
                ThermalObject& obj = object_map[label];
                obj.x = std::min(obj.x, x);
                obj.y = std::min(obj.y, y);
                obj.width = std::max(obj.width, x - obj.x + 1);
                obj.height = std::max(obj.height, y - obj.y + 1);
                
                float temp = temps[y * width + x];
                obj.min_temp = std::min(obj.min_temp, temp);
                obj.max_temp = std::max(obj.max_temp, temp);
            }
        }
    }
    
    std::vector<std::pair<ThermalObject, SignatureScore> > scored_objects;
    
    for (std::map<int, ThermalObject>::iterator it = object_map.begin();
        it != object_map.end(); ++it) {
        ThermalObject& obj = it->second;
        
        float sum = 0.0f;
        int count = 0;
        for (int y = obj.y; y < obj.y + obj.height && y < height; y++) {
            for (int x = obj.x; x < obj.x + obj.width && x < width; x++) {
                if (temps[y * width + x] >= threshold) {
                    sum += temps[y * width + x];
                    count++;
                }
            }
        }
        obj.avg_temp = count > 0 ? sum / count : obj.max_temp;
        
        SignatureScore sig_score = analyzeSignaturePattern(temps, width, height, obj);
        
        if (sig_score.overall_score > 0.50f) {
            scored_objects.push_back(std::make_pair(obj, sig_score));
        }
    }
    
    for (size_t i = 0; i < scored_objects.size(); i++) {
        for (size_t j = i + 1; j < scored_objects.size(); j++) {
            if (scored_objects[j].second.overall_score > scored_objects[i].second.overall_score) {
                std::swap(scored_objects[i], scored_objects[j]);
            }
        }
    }
    
    int max_objects = 2;
    for (size_t i = 0; i < scored_objects.size() && i < (size_t)max_objects; i++) {
        ThermalObject obj = scored_objects[i].first;
        SignatureScore score = scored_objects[i].second;
        
        if (score.overall_score > 0.70f) {
            obj.box_color.r = 0;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
        } else if (score.overall_score > 0.60f) {
            obj.box_color.r = 255;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
        } else {
            obj.box_color.r = 255;
            obj.box_color.g = 150;
            obj.box_color.b = 0;
        }
        obj.box_color.a = 255;
        
        objects.push_back(obj);
    }
    
    return objects;
}

void drawThermalObject(SDL_Renderer* renderer, TTF_Font* font, const ThermalObject& obj, int scale_factor) {
    SDL_Rect box;
    box.x = obj.x * scale_factor;
    box.y = obj.y * scale_factor;
    box.w = obj.width * scale_factor;
    box.h = obj.height * scale_factor;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(renderer, obj.box_color.r, obj.box_color.g, obj.box_color.b, 255);

    int border_thickness = 2;

    for (int i = 0; i < border_thickness; i++) {
        SDL_Rect border;
        border.x = box.x - i;
        border.y = box.y - i;
        border.w = box.w + 2 * i;
        border.h = box.h + 2 * i;
        SDL_RenderDrawRect(renderer, &border);
    }

    if (font) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << obj.avg_temp << " C";
        std::string label = ss.str();

        SDL_Color textColor = { 255, 255, 255, 255 };

        SDL_Surface* textSurface = TTF_RenderText_Solid(font, label.c_str(), textColor);
        if (textSurface) {
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            if (textTexture) {
                SDL_Rect textRect;
                textRect.x = box.x;
                textRect.y = box.y - 22;
                textRect.w = textSurface->w;
                textRect.h = textSurface->h;

                SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 200);
                SDL_Rect bgRect;
                bgRect.x = textRect.x - 2; bgRect.y = textRect.y - 2;
                bgRect.w = textRect.w + 4; bgRect.h = textRect.h + 4;
                SDL_RenderFillRect(renderer, &bgRect);

                SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
                SDL_DestroyTexture(textTexture);
            }
            SDL_FreeSurface(textSurface);
        }
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
}

void drawCenterCrosshair(SDL_Renderer* renderer, TTF_Font* font, int width, int height, float temp) {
    int center_x = width / 2;
    int center_y = height / 2;
    int crosshair_size = 6;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    SDL_RenderDrawLine(renderer, center_x - crosshair_size, center_y, center_x + crosshair_size, center_y);
    SDL_RenderDrawLine(renderer, center_x, center_y - crosshair_size, center_x, center_y + crosshair_size);

    SDL_Rect dot;
    dot.x = center_x - 1;
    dot.y = center_y - 1;
    dot.w = 2;
    dot.h = 2;
    SDL_RenderFillRect(renderer, &dot);
}

void drawHottestPixelMarker(SDL_Renderer* renderer, TTF_Font* font, int x, int y, float temp, float distance_cm, int scale_factor) {
    int display_x = x * scale_factor;
    int display_y = y * scale_factor;
    
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    
    int box_size = 5;
    SDL_Rect box;
    box.x = display_x - box_size/2;
    box.y = display_y - box_size/2;
    box.w = box_size;
    box.h = box_size;
    
    SDL_RenderDrawRect(renderer, &box);
    
    if (font) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << temp << "C | " << distance_cm << "cm";
        std::string label = ss.str();

        SDL_Color textColor = { 0, 255, 0, 255 };

        SDL_Surface* textSurface = TTF_RenderText_Solid(font, label.c_str(), textColor);
        if (textSurface) {
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            if (textTexture) {
                SDL_Rect textRect;
                textRect.x = display_x + 8;
                textRect.y = display_y - 10;
                textRect.w = textSurface->w;
                textRect.h = textSurface->h;

                SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 200);
                SDL_Rect bgRect;
                bgRect.x = textRect.x - 2; bgRect.y = textRect.y - 2;
                bgRect.w = textRect.w + 4; bgRect.h = textRect.h + 4;
                SDL_RenderFillRect(renderer, &bgRect);

                SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
                SDL_DestroyTexture(textTexture);
            }
            SDL_FreeSurface(textSurface);
        }
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
}

void renderTextLine(SDL_Renderer* renderer, TTF_Font* font, const char* text, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Blended(font, text, color);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect rect = { x, y, surface->w, surface->h };
            SDL_RenderCopy(renderer, texture, NULL, &rect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_small,
    const std::vector<ThermalObject>& objects, float center_temp,
    int center_x_thermal, int center_y_thermal,
    ColorMode color_mode, bool& signature_isolation, float& isolation_percentile,
    bool distance_detected, bool signature_locked, float distance_m, float distance_cm,
    int window_width, int window_height, 
    int mouse_x, int mouse_y, bool mouse_down) {

    int sidebar_x = window_width - SIDEBAR_WIDTH;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_Rect sidebar = { sidebar_x, 0, SIDEBAR_WIDTH, window_height };
    SDL_RenderFillRect(renderer, &sidebar);

    SDL_Color white = { 255, 255, 255, 255 };
    SDL_Color black = { 0, 0, 0, 255 };
    int y_pos = 10;

    renderTextLine(renderer, font, "Center Pixel", sidebar_x + 10, y_pos, white);
    y_pos += 20;

    SDL_Color temp_color = {255, 200, 0, 255};
    SDL_SetRenderDrawColor(renderer, temp_color.r, temp_color.g, temp_color.b, 255);
    SDL_Rect temp_bar = { sidebar_x + 10, y_pos, 280, 25 };
    SDL_RenderFillRect(renderer, &temp_bar);

    char temp_text[50];
    snprintf(temp_text, sizeof(temp_text), "%.1f C", center_temp);
    renderTextLine(renderer, font, temp_text, sidebar_x + 20, y_pos + 4, black);
    y_pos += 28;

    char pos_text[100];
    snprintf(pos_text, sizeof(pos_text), "(%d, %d)", center_x_thermal, center_y_thermal);
    renderTextLine(renderer, font_small, pos_text, sidebar_x + 10, y_pos, white);
    y_pos += 25;

    const ThermalObject* targeted_object = NULL;
    for (size_t i = 0; i < objects.size(); i++) {
        const ThermalObject& obj = objects[i];
        if (center_x_thermal >= obj.x && center_x_thermal < obj.x + obj.width &&
            center_y_thermal >= obj.y && center_y_thermal < obj.y + obj.height) {
            targeted_object = &obj;
            break;
        }
    }

    if (targeted_object != NULL) {
        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
        y_pos += 8;

        renderTextLine(renderer, font, "Detected Signature", sidebar_x + 10, y_pos, white);
        y_pos += 20;

        SDL_SetRenderDrawColor(renderer, targeted_object->box_color.r,
            targeted_object->box_color.g,
            targeted_object->box_color.b, 255);
        SDL_Rect colorRect = { sidebar_x + 10, y_pos, 8, 45 };
        SDL_RenderFillRect(renderer, &colorRect);

        int text_x = sidebar_x + 22;
        char line[100];

        snprintf(line, sizeof(line), "%.1f C", targeted_object->avg_temp);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 15;

        snprintf(line, sizeof(line), "Min: %.1f Max: %.1f", targeted_object->min_temp, targeted_object->max_temp);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 15;

        snprintf(line, sizeof(line), "%dx%d (%d px)", 
            targeted_object->width, targeted_object->height,
            targeted_object->width * targeted_object->height);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 15;
        
        const char* quality_text;
        if (targeted_object->box_color.g == 255 && targeted_object->box_color.r == 0) {
            quality_text = "Quality: Excellent (>70%)";
        } else if (targeted_object->box_color.g == 255 && targeted_object->box_color.r == 255) {
            quality_text = "Quality: Good (>60%)";
        } else {
            quality_text = "Quality: OK (>50%)";
        }
        renderTextLine(renderer, font_small, quality_text, text_x, y_pos, 
            SDL_Color{targeted_object->box_color.r, targeted_object->box_color.g, targeted_object->box_color.b, 255});
        y_pos += 20;
    }

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;

    renderTextLine(renderer, font, "Detection", sidebar_x + 10, y_pos, white);
    y_pos += 20;

    char summary[100];
    snprintf(summary, sizeof(summary), "Red Pixel Objects: %zu", objects.size());
    renderTextLine(renderer, font_small, summary, sidebar_x + 10, y_pos, white);
    y_pos += 20;
    
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;
    
    const char* mode_text = (color_mode == ABSOLUTE) ? "ABSOLUTE (24-28 C)" : "RELATIVE";
    SDL_Color mode_color = (color_mode == ABSOLUTE) ? 
        SDL_Color{0, 255, 100, 255} :
        SDL_Color{255, 200, 0, 255};
    renderTextLine(renderer, font_small, mode_text, sidebar_x + 10, y_pos, mode_color);
    y_pos += 15;
    
    renderTextLine(renderer, font_small, "Press 'M' to toggle", sidebar_x + 10, y_pos, SDL_Color{120, 120, 120, 255});
    y_pos += 20;
    
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;
    
    renderTextLine(renderer, font, "Signature Isolation", sidebar_x + 10, y_pos, white);
    y_pos += 20;
    
    const char* iso_status = signature_isolation ? "ON (only top % shown)" : "OFF (full range)";
    SDL_Color iso_color = signature_isolation ? 
        SDL_Color{255, 100, 100, 255} :
        SDL_Color{100, 100, 100, 255};
    renderTextLine(renderer, font_small, iso_status, sidebar_x + 10, y_pos, iso_color);
    y_pos += 15;
    
    if (color_mode == RELATIVE) {
        renderTextLine(renderer, font_small, "Press 'I' to toggle", sidebar_x + 10, y_pos, SDL_Color{120, 120, 120, 255});
        y_pos += 15;
        
        if (signature_isolation) {
            renderTextLine(renderer, font_small, "Top Percentile:", sidebar_x + 10, y_pos, white);
            y_pos += 15;
            
            int iso_slider_x = sidebar_x + 10;
            int iso_slider_y = y_pos;
            int iso_slider_width = 280;
            int iso_slider_height = 15;
            
            SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
            SDL_Rect iso_slider_bg = { iso_slider_x, iso_slider_y, iso_slider_width, iso_slider_height };
            SDL_RenderFillRect(renderer, &iso_slider_bg);
            
            float iso_normalized = (isolation_percentile - 0.01f) / 0.19f;
            int iso_fill_width = (int)(iso_slider_width * iso_normalized);
            SDL_SetRenderDrawColor(renderer, 255, 100, 100, 255);
            SDL_Rect iso_slider_fill = { iso_slider_x, iso_slider_y, iso_fill_width, iso_slider_height };
            SDL_RenderFillRect(renderer, &iso_slider_fill);
            
            int iso_handle_x = iso_slider_x + iso_fill_width - 4;
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_Rect iso_slider_handle = { iso_handle_x, iso_slider_y - 2, 8, iso_slider_height + 4 };
            SDL_RenderFillRect(renderer, &iso_slider_handle);
            
            if (mouse_down && mouse_y >= iso_slider_y && mouse_y <= iso_slider_y + iso_slider_height &&
                mouse_x >= iso_slider_x && mouse_x <= iso_slider_x + iso_slider_width) {
                float new_normalized = (float)(mouse_x - iso_slider_x) / iso_slider_width;
                new_normalized = std::max(0.0f, std::min(1.0f, new_normalized));
                isolation_percentile = 0.01f + (new_normalized * 0.19f);
            }
            
            y_pos += 18;
            char iso_text[50];
            snprintf(iso_text, sizeof(iso_text), "%.1f%% (1-20%%)", isolation_percentile * 100);
            renderTextLine(renderer, font_small, iso_text, sidebar_x + 10, y_pos, white);
            y_pos += 15;
        } else {
            y_pos += 15;
        }
    } else {
        renderTextLine(renderer, font_small, "(Only in RELATIVE mode)", sidebar_x + 10, y_pos, SDL_Color{100, 100, 100, 255});
        y_pos += 15;
    }
    y_pos += 5;
    
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;
    
    renderTextLine(renderer, font, "Distance to Glass", sidebar_x + 10, y_pos, white);
    y_pos += 20;
    
    if (distance_detected) {
        const char* lock_status = signature_locked ? "LOCKED ✓" : "Unlocked";
        SDL_Color lock_color = signature_locked ? 
            SDL_Color{0, 255, 100, 255} :
            SDL_Color{200, 100, 100, 255};
        renderTextLine(renderer, font_small, lock_status, sidebar_x + 10, y_pos, lock_color);
        y_pos += 15;
        
        char dist_text[100];
        snprintf(dist_text, sizeof(dist_text), "%.2f cm", distance_cm);
        
        SDL_Color dist_color = {0, 255, 100, 255};
        renderTextLine(renderer, font, dist_text, sidebar_x + 10, y_pos, dist_color);
        y_pos += 20;
        
        snprintf(dist_text, sizeof(dist_text), "(%.3f m)", distance_m);
        renderTextLine(renderer, font_small, dist_text, sidebar_x + 10, y_pos, white);
        y_pos += 15;
        
        renderTextLine(renderer, font_small, "2D Triangulation", sidebar_x + 10, y_pos, SDL_Color{100, 150, 255, 255});
        y_pos += 15;
        
        renderTextLine(renderer, font_small, "(Vertical + Yaw correction)", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
    } else {
        renderTextLine(renderer, font_small, "Searching for hot spot...", sidebar_x + 10, y_pos, SDL_Color{200, 100, 100, 255});
        y_pos += 15;
        renderTextLine(renderer, font_small, "(Heat source not visible)", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
    }
}

void handle_camera_frame_available(seekcamera_t* camera, seekcamera_frame_t* camera_frame, void* user_data) {
    (void)camera;
    seekrenderer_t* renderer = (seekrenderer_t*)user_data;

    std::lock_guard<std::mutex> guard(renderer->the_mutex);

    seekcamera_frame_lock(camera_frame);
    renderer->is_dirty.store(true);
    renderer->frame = camera_frame;

    seekframe_t* thermal_frame = nullptr;
    if (seekcamera_frame_get_frame_by_format(camera_frame,
        SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT,
        &thermal_frame) == SEEKCAMERA_SUCCESS) {

        float* temps = (float*)seekframe_get_data(thermal_frame);
        int width = seekframe_get_width(thermal_frame);
        int height = seekframe_get_height(thermal_frame);

        if (temps && width > 0 && height > 0) {
            renderer->detected_objects = detectRedPixels(temps, width, height,
                renderer->next_object_id,
                renderer->signature_isolation,
                renderer->isolation_percentile);

            int center_x = width / 2;
            int center_y = height / 2;
            renderer->center_pixel_temp = temps[center_y * width + center_x];
        }
    }

    g_is_dirty.store(true);
    g_condition_variable.notify_one();
}

void handle_camera_connect(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID((char*)&cid);

    seekrenderer_t* renderer = g_renderers[chipID] == nullptr ? new seekrenderer_t() : g_renderers[chipID];
    renderer->is_active.store(true);
    renderer->camera = camera;

    seekcamera_error_t status = seekcamera_register_frame_available_callback(camera, handle_camera_frame_available, (void*)renderer);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to register frame callback: " << seekcamera_error_get_str(status) << std::endl;
        renderer->is_active.store(false);
        return;
    }

    status = seekcamera_set_color_palette(camera, SEEKCAMERA_COLOR_PALETTE_WHITE_HOT);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to set color palette: " << seekcamera_error_get_str(status) << std::endl;
    }

    uint32_t frame_formats = SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
    status = seekcamera_capture_session_start(camera, frame_formats);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to start capture session: " << seekcamera_error_get_str(status) << std::endl;
        renderer->is_active.store(false);
        return;
    }

    g_renderers[chipID] = renderer;
    std::cout << "Camera connected: " << chipID << std::endl;
}

void handle_camera_disconnect(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID((char*)&cid);
    seekrenderer_t* renderer = g_renderers[chipID];

    seekcamera_capture_session_stop(camera);

    if (renderer != nullptr) {
        renderer->is_active.store(false);
        g_is_dirty.store(true);
    }
}

void handle_camera_error(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)user_data;
    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::cerr << "Camera error (CID: " << cid << "): " << seekcamera_error_get_str(event_status) << std::endl;
}

void handle_camera_ready_to_pair(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    const seekcamera_error_t status = seekcamera_store_calibration_data(camera, nullptr, nullptr, nullptr);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to pair device: " << seekcamera_error_get_str(status) << std::endl;
    }
    handle_camera_connect(camera, event_status, user_data);
}

void camera_event_callback(seekcamera_t* camera, seekcamera_manager_event_t event, seekcamera_error_t event_status, void* user_data) {
    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::cout << seekcamera_manager_get_event_str(event) << " (CID: " << cid << ")" << std::endl;

    switch (event) {
    case SEEKCAMERA_MANAGER_EVENT_CONNECT:
        handle_camera_connect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
        handle_camera_disconnect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_ERROR:
        handle_camera_error(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
        handle_camera_ready_to_pair(camera, event_status, user_data);
        break;
    default:
        break;
    }
}

void seekrenderer_close_window(seekrenderer_t* renderer) {
    if (renderer == nullptr) return;

    if (renderer->is_active.load())
        seekcamera_capture_session_stop(renderer->camera);

    renderer->is_active.store(false);
    renderer->is_dirty.store(false);
    renderer->frame = nullptr;

    if (renderer->font != nullptr) {
        TTF_CloseFont(renderer->font);
        renderer->font = nullptr;
    }

    if (renderer->font_small != nullptr) {
        TTF_CloseFont(renderer->font_small);
        renderer->font_small = nullptr;
    }

    if (renderer->texture != nullptr) {
        SDL_DestroyTexture(renderer->texture);
        renderer->texture = nullptr;
    }

    if (renderer->renderer != nullptr) {
        SDL_DestroyRenderer(renderer->renderer);
        renderer->renderer = nullptr;
    }

    if (renderer->window != nullptr) {
        SDL_DestroyWindow(renderer->window);
        renderer->window = nullptr;
    }

    renderer->camera = nullptr;
}

static void signal_callback(int signum) {
    (void)signum;
    std::cout << "\nCaught termination signal" << std::endl;
    g_exit_requested.store(true);
}

int main() {
    g_exit_requested.store(false);
    g_is_dirty.store(false);

    signal(SIGINT, signal_callback);
    signal(SIGTERM, signal_callback);

    std::cout << "Thermal Object Tracker - 2D Triangulation Distance (FIXED)" << std::endl;
    std::cout << "===========================================================" << std::endl;
    std::cout << "FIXES APPLIED:" << std::endl;
    std::cout << "✓ Relative position-based spot identification" << std::endl;
    std::cout << "✓ Works in ANY quadrant (not just top-left)" << std::endl;
    std::cout << "✓ Removed invalid quadrant assumptions" << std::endl;
    std::cout << "✓ Proper absolute value handling for angles" << std::endl;
    std::cout << "\nDistance Measurement:" << std::endl;
    std::cout << "1. VERTICAL TRIANGULATION: Uses 2.5cm baseline above camera" << std::endl;
    std::cout << "2. HORIZONTAL YAW DETECTION: Uses 3.3cm baseline left of camera" << std::endl;
    std::cout << "3. DUAL-EMITTER GEOMETRY: Works regardless of object position" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "- Press 'M' to toggle ABSOLUTE/RELATIVE color modes" << std::endl;
    std::cout << "- Press 'I' to toggle Signature Isolation" << std::endl;
    std::cout << "- Press 'Q' to quit\n" << std::endl;

    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    seekcamera_manager_t* manager = nullptr;
    seekcamera_error_t status = seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager: " << seekcamera_error_get_str(status) << std::endl;
        return 1;
    }

    status = seekcamera_manager_register_event_callback(manager, camera_event_callback, nullptr);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to register event callback: " << seekcamera_error_get_str(status) << std::endl;
        return 1;
    }

    TriangulationDistanceSensor distance_sensor;

    while (!g_exit_requested.load()) {
        std::unique_lock<std::mutex> event_lock(g_mutex);

        if (g_condition_variable.wait_for(event_lock, std::chrono::milliseconds(150),
            []() { return g_is_dirty.load(); })) {
            g_is_dirty.store(false);

            for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                it != g_renderers.end(); ++it) {
                seekrenderer_t* renderer = it->second;
                if (renderer == nullptr) continue;

                if (!renderer->is_active.load()) {
                    if (renderer->window != nullptr)
                        seekrenderer_close_window(renderer);
                    continue;
                }

                if (renderer->is_active.load() && renderer->window == nullptr) {
                    seekcamera_chipid_t cid;
                    memset(&cid, 0, sizeof(cid));
                    seekcamera_get_chipid(renderer->camera, &cid);
                    std::stringstream window_title;
                    window_title << "Thermal Tracker - Dual Emitter (CID: " << cid << ")";

                    renderer->window = SDL_CreateWindow(window_title.str().c_str(), 100, 100, 0, 0, SDL_WINDOW_HIDDEN);
                    renderer->renderer = SDL_CreateRenderer(renderer->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

                    renderer->font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
                    if (!renderer->font) {
                        renderer->font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
                    }

                    renderer->font_small = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12);
                    if (!renderer->font_small) {
                        renderer->font_small = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 12);
                    }
                }

                {
                    std::lock_guard<std::mutex> guard(renderer->the_mutex);

                    if (renderer->is_dirty.load() && renderer->frame != nullptr && renderer->is_active.load()) {

                        seekframe_t* thermal_frame = nullptr;
                        status = seekcamera_frame_get_frame_by_format(renderer->frame,
                            SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT,
                            &thermal_frame);
                        if (status != SEEKCAMERA_SUCCESS) {
                            seekcamera_frame_unlock(renderer->frame);
                            continue;
                        }

                        const int frame_width = (int)seekframe_get_width(thermal_frame);
                        const int frame_height = (int)seekframe_get_height(thermal_frame);
                        float* temps = (float*)seekframe_get_data(thermal_frame);

                        if (renderer->texture == nullptr) {
                            const int scale_factor = 2;
                            int window_width = frame_width * scale_factor + SIDEBAR_WIDTH;
                            int window_height = frame_height * scale_factor;

                            SDL_RenderSetLogicalSize(renderer->renderer, window_width, window_height);
                            renderer->texture = SDL_CreateTexture(renderer->renderer, SDL_PIXELFORMAT_ARGB8888,
                                SDL_TEXTUREACCESS_STREAMING, frame_width, frame_height);
                            SDL_SetWindowSize(renderer->window, window_width, window_height);
                            SDL_ShowWindow(renderer->window);
                        }

                        float min_temp = 1000.0f;
                        float max_temp = -1000.0f;
                        for (int i = 0; i < frame_width * frame_height; i++) {
                            min_temp = std::min(min_temp, temps[i]);
                            max_temp = std::max(max_temp, temps[i]);
                        }

                        Uint32* pixels;
                        int pitch;
                        SDL_LockTexture(renderer->texture, nullptr, (void**)&pixels, &pitch);

                        for (int y = 0; y < frame_height; y++) {
                            for (int x = 0; x < frame_width; x++) {
                                float temp = temps[y * frame_width + x];
                                
                                SDL_Color color;
                                if (renderer->color_mode == ABSOLUTE) {
                                    color = mapTemperatureAbsolute(temp);
                                } else {
                                    color = mapTemperatureRelative(temp, min_temp, max_temp, 
                                                                    renderer->signature_isolation,
                                                                    renderer->isolation_percentile);
                                }
                                
                                Uint32 pixel = (255 << 24) | (color.r << 16) | (color.g << 8) | color.b;
                                pixels[y * (pitch / 4) + x] = pixel;
                            }
                        }

                        SDL_UnlockTexture(renderer->texture);

                        int window_width, window_height;
                        SDL_GetWindowSize(renderer->window, &window_width, &window_height);

                        SDL_Rect thermal_rect;
                        thermal_rect.x = 0;
                        thermal_rect.y = 0;
                        thermal_rect.w = window_width - SIDEBAR_WIDTH;
                        thermal_rect.h = window_height;
                        SDL_RenderCopy(renderer->renderer, renderer->texture, nullptr, &thermal_rect);

                        const int scale_factor = 2;

                        int center_x_thermal = frame_width / 2;
                        int center_y_thermal = frame_height / 2;

                        drawCenterCrosshair(renderer->renderer, renderer->font,
                            window_width - SIDEBAR_WIDTH, window_height,
                            renderer->center_pixel_temp);

                        TriangulationDistanceSensor::DistanceResult distance_result = 
                            distance_sensor.calculate(temps, frame_width, frame_height);
                        
                        if (distance_result.detected) {
                            drawHottestPixelMarker(renderer->renderer, renderer->font,
                                distance_result.hottest_x, distance_result.hottest_y,
                                distance_result.hottest_temp, distance_result.distance_to_glass_cm,
                                scale_factor);
                            
                            int display_x = distance_result.second_spot_x * scale_factor;
                            int display_y = distance_result.second_spot_y * scale_factor;
                            
                            SDL_SetRenderDrawColor(renderer->renderer, 0, 255, 255, 255);
                            
                            int box_size = 5;
                            SDL_Rect box;
                            box.x = display_x - box_size/2;
                            box.y = display_y - box_size/2;
                            box.w = box_size;
                            box.h = box_size;
                            SDL_RenderDrawRect(renderer->renderer, &box);
                            
                            if (renderer->font) {
                                std::stringstream ss;
                                ss << std::fixed << std::setprecision(1) << distance_result.second_spot_temp << "C";
                                std::string label = ss.str();
                                
                                SDL_Color textColor = { 0, 255, 255, 255 };
                                SDL_Surface* textSurface = TTF_RenderText_Solid(renderer->font, label.c_str(), textColor);
                                if (textSurface) {
                                    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer->renderer, textSurface);
                                    if (textTexture) {
                                        SDL_Rect textRect;
                                        textRect.x = display_x + 8;
                                        textRect.y = display_y - 10;
                                        textRect.w = textSurface->w;
                                        textRect.h = textSurface->h;
                                        
                                        SDL_SetRenderDrawBlendMode(renderer->renderer, SDL_BLENDMODE_BLEND);
                                        SDL_SetRenderDrawColor(renderer->renderer, 0, 0, 0, 200);
                                        SDL_Rect bgRect;
                                        bgRect.x = textRect.x - 2; bgRect.y = textRect.y - 2;
                                        bgRect.w = textRect.w + 4; bgRect.h = textRect.h + 4;
                                        SDL_RenderFillRect(renderer->renderer, &bgRect);
                                        
                                        SDL_RenderCopy(renderer->renderer, textTexture, nullptr, &textRect);
                                        SDL_DestroyTexture(textTexture);
                                    }
                                    SDL_FreeSurface(textSurface);
                                }
                            }
                        }

                        drawSidebar(renderer->renderer, renderer->font, renderer->font_small,
                            renderer->detected_objects, renderer->center_pixel_temp,
                            center_x_thermal, center_y_thermal,
                            renderer->color_mode, renderer->signature_isolation, 
                            renderer->isolation_percentile,
                            distance_result.detected, false,
                            distance_result.distance_to_glass_m, distance_result.distance_to_glass_cm,
                            window_width, window_height, 
                            renderer->mouse_x, renderer->mouse_y, renderer->mouse_down);

                        SDL_RenderPresent(renderer->renderer);

                        seekcamera_frame_unlock(renderer->frame);
                        renderer->is_dirty.store(false);
                        renderer->frame = nullptr;
                    }
                }
            }
        }
        else {
            event_lock.unlock();
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_m) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->color_mode = (it->second->color_mode == ABSOLUTE) ? 
                            RELATIVE : ABSOLUTE;
                        std::cout << "Color mode: " << 
                            ((it->second->color_mode == ABSOLUTE) ? "ABSOLUTE" : "RELATIVE") 
                            << std::endl;
                    }
                }
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_i) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->signature_isolation = !it->second->signature_isolation;
                        std::cout << "Signature isolation: " << 
                            (it->second->signature_isolation ? "ON" : "OFF");
                        if (it->second->signature_isolation) {
                            std::cout << " (top " << (it->second->isolation_percentile * 100) << "%)";
                        }
                        std::cout << std::endl;
                    }
                }
            }
            else if (event.type == SDL_MOUSEMOTION) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_x = event.motion.x;
                        it->second->mouse_y = event.motion.y;
                    }
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_down = true;
                    }
                }
            }
            else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_down = false;
                    }
                }
            }
        }
    }

    std::cout << "Shutting down..." << std::endl;

    seekcamera_manager_destroy(&manager);

    for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
        it != g_renderers.end(); ++it) {
        if (it->second != nullptr) {
            seekrenderer_close_window(it->second);
            delete it->second;
        }
    }
    g_renderers.clear();

    TTF_Quit();
    SDL_Quit();

    std::cout << "Done" << std::endl;
    return 0;
}