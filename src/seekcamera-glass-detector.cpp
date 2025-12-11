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

// SIZE-BASED DISTANCE SENSOR - Now with SIGNATURE LOCKING
class TriangulationDistanceSensor {
private:
    // SIZE-BASED CALIBRATION
    const float CALIBRATION_DISTANCE = 0.03f;  // 3cm (known distance)
    float calibration_area = 0.0f;
    bool is_calibrated = false;
    
    // SIGNATURE LOCKING
    bool signature_locked = false;
    int locked_signature_id = -1;
    
    // Locked signature characteristics (for tracking across frames)
    struct LockedSignature {
        float center_x, center_y;  // Last known position
        float temp_min, temp_max;  // Temperature range
        float area_expected;       // Expected area (for validation)
        int frames_lost;           // Counter for lost tracking
    };
    
    LockedSignature locked_sig;
    
    // CAMERA SPECIFICATIONS
    const float SENSOR_WIDTH = 320.0f;
    const float SENSOR_HEIGHT = 240.0f;
    const int MAX_FRAMES_LOST = 5;  // Allow 5 frames of lost tracking before unlock
    
public:
    TriangulationDistanceSensor() {
        locked_sig.center_x = 0;
        locked_sig.center_y = 0;
        locked_sig.temp_min = 0;
        locked_sig.temp_max = 0;
        locked_sig.area_expected = 0;
        locked_sig.frames_lost = 0;
        
        std::cout << "\n=== Gradient Pattern-Based Distance Sensor ===" << std::endl;
        std::cout << "Tracks signature by spatial temperature pattern" << std::endl;
        std::cout << "Press 'U' to UNLOCK and redetect\n" << std::endl;
    }
    
    // Lock onto a specific signature
    void lockSignature(const ThermalObject& obj) {
        signature_locked = true;
        locked_signature_id = obj.id;
        
        // Store signature characteristics
        locked_sig.center_x = obj.x + obj.width / 2.0f;
        locked_sig.center_y = obj.y + obj.height / 2.0f;
        locked_sig.temp_min = obj.min_temp;
        locked_sig.temp_max = obj.max_temp;
        locked_sig.area_expected = obj.width * obj.height;
        locked_sig.frames_lost = 0;
        
        // Auto-calibrate on lock
        if (!is_calibrated) {
            calibration_area = locked_sig.area_expected;
            is_calibrated = true;
            std::cout << "✓ AUTO-LOCKED & CALIBRATED: Area = " << calibration_area 
                      << " px² at " << (CALIBRATION_DISTANCE * 100) << " cm" << std::endl;
        } else {
            std::cout << "✓ AUTO-LOCKED onto signature" << std::endl;
        }
        
        std::cout << "  Position: (" << locked_sig.center_x << ", " << locked_sig.center_y << ")" << std::endl;
        std::cout << "  Temp: " << locked_sig.temp_min << "-" << locked_sig.temp_max << " °C" << std::endl;
        std::cout << "  Area: " << locked_sig.area_expected << " px²" << std::endl;
    }
    
    // Unlock signature
    void unlockSignature() {
        if (signature_locked) {
            signature_locked = false;
            locked_signature_id = -1;
            std::cout << "✗ UNLOCKED signature" << std::endl;
        }
    }
    
    // Check if signature is locked
    bool isLocked() const {
        return signature_locked;
    }
    
    // Find the locked signature in current frame
    ThermalObject* findLockedSignature(std::vector<ThermalObject>& objects) {
        if (!signature_locked) return nullptr;
        
        ThermalObject* best_match = nullptr;
        float best_score = 999999.0f;
        
        for (size_t i = 0; i < objects.size(); i++) {
            // Calculate match score based on:
            // 1. Position proximity
            // 2. Temperature similarity
            // 3. Size similarity
            
            float obj_center_x = objects[i].x + objects[i].width / 2.0f;
            float obj_center_y = objects[i].y + objects[i].height / 2.0f;
            
            // Position distance (normalized)
            float pos_diff = sqrt(pow(obj_center_x - locked_sig.center_x, 2) + 
                                 pow(obj_center_y - locked_sig.center_y, 2));
            float pos_score = pos_diff / 50.0f;  // Normalize to ~50 pixel max movement
            
            // Temperature similarity
            float temp_center = (locked_sig.temp_min + locked_sig.temp_max) / 2.0f;
            float obj_temp_center = (objects[i].min_temp + objects[i].max_temp) / 2.0f;
            float temp_diff = fabs(temp_center - obj_temp_center);
            float temp_score = temp_diff / 2.0f;  // Normalize to 2°C tolerance
            
            // Size similarity
            float obj_area = objects[i].width * objects[i].height;
            float size_ratio = obj_area / locked_sig.area_expected;
            if (size_ratio > 1.0f) size_ratio = 1.0f / size_ratio;  // Normalize
            float size_score = 1.0f - size_ratio;
            
            // Combined score (lower is better)
            float score = pos_score + temp_score + size_score;
            
            if (score < best_score && score < 3.0f) {  // Threshold for valid match
                best_score = score;
                best_match = &objects[i];
            }
        }
        
        if (best_match != nullptr) {
            // Update locked signature position for next frame
            locked_sig.center_x = best_match->x + best_match->width / 2.0f;
            locked_sig.center_y = best_match->y + best_match->height / 2.0f;
            locked_sig.area_expected = best_match->width * best_match->height;
            locked_sig.frames_lost = 0;
        } else {
            // Lost tracking
            locked_sig.frames_lost++;
            if (locked_sig.frames_lost >= MAX_FRAMES_LOST) {
                std::cout << "⚠ Lost tracking for " << MAX_FRAMES_LOST 
                          << " frames - auto-unlocking" << std::endl;
                unlockSignature();
            }
        }
        
        return best_match;
    }
    
    // Calculate distance from pixel area
    float calculateDistanceFromSize(float pixel_area) {
        if (!is_calibrated) {
            calibration_area = pixel_area;
            is_calibrated = true;
            return CALIBRATION_DISTANCE;
        }
        
        float distance = CALIBRATION_DISTANCE * sqrt(calibration_area / pixel_area);
        return distance;
    }
    
    struct DistanceResult {
        bool detected;
        bool is_locked;
        float distance_m;
        float distance_cm;
        float confidence;
        
        int pixel_x;
        int pixel_y;
        int pixel_width;
        int pixel_height;
        int pixel_area;
        float avg_temp;
    };
    
    DistanceResult measure(std::vector<ThermalObject>& objects, bool auto_lock = true) {
        DistanceResult result;
        result.detected = false;
        result.is_locked = signature_locked;
        
        ThermalObject* signature = nullptr;
        
        if (signature_locked) {
            // Use LOCKED signature only
            signature = findLockedSignature(objects);
            
            if (signature == nullptr) {
                // Still locked but not found this frame
                return result;
            }
        } else {
            // Find first object (should be the red dot after isolation)
            if (!objects.empty()) {
                signature = &objects[0];
                
                // AUTO-LOCK on first detection
                if (auto_lock) {
                    lockSignature(*signature);
                }
            }
        }
        
        if (signature == nullptr) {
            return result;
        }
        
        // Calculate pixel area
        int pixel_area = signature->width * signature->height;
        
        // Calculate distance from size
        result.distance_m = calculateDistanceFromSize((float)pixel_area);
        result.distance_cm = result.distance_m * 100.0f;
        
        // Confidence based on temperature
        float temp_diff = fabs(signature->avg_temp - 27.5f);
        result.confidence = std::max(0.0f, 1.0f - (temp_diff / 3.0f));
        
        // Debug info
        result.pixel_x = signature->x + signature->width / 2;
        result.pixel_y = signature->y + signature->height / 2;
        result.pixel_width = signature->width;
        result.pixel_height = signature->height;
        result.pixel_area = pixel_area;
        result.avg_temp = signature->avg_temp;
        result.detected = true;
        
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
// Analyzes the spatial temperature pattern to identify the specific signature
struct SignatureScore {
    float gradient_quality;    // How smoothly temperature declines from center
    float compactness;         // How concentrated the signature is
    float peak_centrality;     // How centered the hottest point is
    float overall_score;       // Combined score
};

SignatureScore analyzeSignaturePattern(float* temps, int width, int height, const ThermalObject& obj) {
    SignatureScore score;
    score.gradient_quality = 0.0f;
    score.compactness = 0.0f;
    score.peak_centrality = 0.0f;
    score.overall_score = 0.0f;
    
    // Find peak temperature location within object
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
    
    // 1. PEAK CENTRALITY: Peak should be near center of bounding box
    float center_x = obj.x + obj.width / 2.0f;
    float center_y = obj.y + obj.height / 2.0f;
    float peak_offset = sqrt(pow(peak_x - center_x, 2) + pow(peak_y - center_y, 2));
    float max_offset = sqrt(pow(obj.width / 2.0f, 2) + pow(obj.height / 2.0f, 2));
    score.peak_centrality = max_offset > 0 ? (1.0f - (peak_offset / max_offset)) : 1.0f;
    
    // 2. GRADIENT QUALITY: Temperature should decline smoothly from peak
    // Sample in 4 directions from peak and measure gradient
    float gradient_scores[4] = {0, 0, 0, 0};
    int directions[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};  // Right, Down, Left, Up
    
    for (int dir = 0; dir < 4; dir++) {
        int dx = directions[dir][0];
        int dy = directions[dir][1];
        
        float last_temp = peak_temp;
        int consecutive_declines = 0;
        int total_samples = 0;
        
        // Sample up to 5 pixels in this direction
        for (int step = 1; step <= 5; step++) {
            int sample_x = peak_x + (dx * step);
            int sample_y = peak_y + (dy * step);
            
            // Check bounds
            if (sample_x < 0 || sample_x >= width || sample_y < 0 || sample_y >= height) break;
            if (sample_x < obj.x || sample_x >= obj.x + obj.width) break;
            if (sample_y < obj.y || sample_y >= obj.y + obj.height) break;
            
            float sample_temp = temps[sample_y * width + sample_x];
            
            // Check if temperature declined
            if (sample_temp < last_temp) {
                consecutive_declines++;
            }
            last_temp = sample_temp;
            total_samples++;
        }
        
        // Score this direction based on consistent decline
        gradient_scores[dir] = total_samples > 0 ? (float)consecutive_declines / total_samples : 0.0f;
    }
    
    // Average gradient quality across all directions
    score.gradient_quality = (gradient_scores[0] + gradient_scores[1] + 
                              gradient_scores[2] + gradient_scores[3]) / 4.0f;
    
    // 3. COMPACTNESS: Signature should be small and concentrated
    // Score based on size (smaller is better for the dot signature)
    int area = obj.width * obj.height;
    float ideal_min_area = 9.0f;    // ~3x3 pixels minimum
    float ideal_max_area = 400.0f;  // ~20x20 pixels maximum
    
    if (area < ideal_min_area) {
        score.compactness = 0.0f;  // Too small to be reliable
    } else if (area > ideal_max_area) {
        score.compactness = 0.3f;  // Too large, probably not our signature
    } else {
        // Prefer medium-sized signatures
        float size_ratio = (float)area / ideal_max_area;
        score.compactness = 1.0f - (size_ratio * 0.5f);  // Peaks at small sizes
    }
    
    // 4. OVERALL SCORE: Weighted combination
    // Gradient quality is most important (signature characteristic)
    // Peak centrality ensures it's a focused blob
    // Compactness ensures appropriate size
    score.overall_score = (score.gradient_quality * 0.5f) +   // 50% weight
                          (score.peak_centrality * 0.3f) +     // 30% weight
                          (score.compactness * 0.2f);          // 20% weight
    
    return score;
}

// AUTOMATIC RED-PIXEL DETECTION WITH GRADIENT PATTERN MATCHING
// Detects objects that would appear RED (top percentile) and match signature pattern
std::vector<ThermalObject> detectRedPixels(float* temps, int width, int height, int& next_id, 
                                            bool use_isolation, float isolation_percentile) {
    std::vector<ThermalObject> objects;
    
    // Find min/max temps
    float min_temp = 1000.0f;
    float max_temp = -1000.0f;
    for (int i = 0; i < width * height; i++) {
        min_temp = std::min(min_temp, temps[i]);
        max_temp = std::max(max_temp, temps[i]);
    }
    
    float range = max_temp - min_temp;
    if (range < 0.01f) return objects;  // No variation
    
    // Calculate threshold - only pixels that would be RED
    float threshold;
    if (use_isolation) {
        // Top X% (the red pixels after isolation)
        threshold = max_temp - (range * isolation_percentile);
    } else {
        // Top 5% by default
        threshold = max_temp - (range * 0.05f);
    }
    
    // Create binary map of "red" pixels
    std::vector<std::vector<bool> > red_pixels(height, std::vector<bool>(width, false));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] >= threshold) {
                red_pixels[y][x] = true;
            }
        }
    }
    
    // Find connected components of red pixels
    std::vector<std::vector<int> > labels = findConnectedComponents(red_pixels, width, height);
    
    // Extract objects from labeled components
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
    
    // Calculate averages and analyze patterns
    std::vector<std::pair<ThermalObject, SignatureScore> > scored_objects;
    
    for (std::map<int, ThermalObject>::iterator it = object_map.begin();
        it != object_map.end(); ++it) {
        ThermalObject& obj = it->second;
        
        // Calculate average temperature
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
        
        // ANALYZE GRADIENT PATTERN
        SignatureScore sig_score = analyzeSignaturePattern(temps, width, height, obj);
        
        // Only keep objects with good signature pattern (score > 0.4)
        if (sig_score.overall_score > 0.4f) {
            scored_objects.push_back(std::make_pair(obj, sig_score));
        }
    }
    
    // Sort by signature score (best match first)
    for (size_t i = 0; i < scored_objects.size(); i++) {
        for (size_t j = i + 1; j < scored_objects.size(); j++) {
            if (scored_objects[j].second.overall_score > scored_objects[i].second.overall_score) {
                std::swap(scored_objects[i], scored_objects[j]);
            }
        }
    }
    
    // Return only the best matching signature(s)
    // Typically just the top 1-2 best matches
    int max_objects = 2;  // Return at most 2 best matches
    for (size_t i = 0; i < scored_objects.size() && i < (size_t)max_objects; i++) {
        ThermalObject obj = scored_objects[i].first;
        SignatureScore score = scored_objects[i].second;
        
        // Color based on signature quality
        if (score.overall_score > 0.7f) {
            // Excellent match - bright green
            obj.box_color.r = 0;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
        } else if (score.overall_score > 0.55f) {
            // Good match - yellow
            obj.box_color.r = 255;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
        } else {
            // Weak match - orange/red
            obj.box_color.r = 255;
            obj.box_color.g = 100;
            obj.box_color.b = 0;
        }
        obj.box_color.a = 255;
        
        objects.push_back(obj);
    }
    
    return objects;
}

// Draw bounding box
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

    // Draw text label
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

// Draw center crosshair
void drawCenterCrosshair(SDL_Renderer* renderer, TTF_Font* font, int width, int height, float temp) {
    int center_x = width / 2;
    int center_y = height / 2;
    int crosshair_size = 6;

    // Always draw white crosshair (no threshold comparison needed)
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

// Helper to render text
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

// Draw sidebar
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

    // Center Pixel Info (COMPACT)
    renderTextLine(renderer, font, "Center Pixel", sidebar_x + 10, y_pos, white);
    y_pos += 20;

    SDL_Color temp_color = {255, 200, 0, 255};  // Yellow/orange
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

    // Find targeted object
    const ThermalObject* targeted_object = NULL;
    for (size_t i = 0; i < objects.size(); i++) {
        const ThermalObject& obj = objects[i];
        if (center_x_thermal >= obj.x && center_x_thermal < obj.x + obj.width &&
            center_y_thermal >= obj.y && center_y_thermal < obj.y + obj.height) {
            targeted_object = &obj;
            break;
        }
    }

    // Targeted Object (COMPACT)
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
        
        // Show signature quality based on box color
        const char* quality_text;
        if (targeted_object->box_color.g == 255 && targeted_object->box_color.r == 0) {
            quality_text = "Quality: Excellent";
        } else if (targeted_object->box_color.g == 255 && targeted_object->box_color.r == 255) {
            quality_text = "Quality: Good";
        } else {
            quality_text = "Quality: Weak";
        }
        renderTextLine(renderer, font_small, quality_text, text_x, y_pos, 
            SDL_Color{targeted_object->box_color.r, targeted_object->box_color.g, targeted_object->box_color.b, 255});
        y_pos += 20;
    }

    // Detection Summary (COMPACT)
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;  // Reduced from 12

    renderTextLine(renderer, font, "Detection", sidebar_x + 10, y_pos, white);
    y_pos += 20;  // Reduced from 25

    char summary[100];
    snprintf(summary, sizeof(summary), "Red Pixel Objects: %zu", objects.size());
    renderTextLine(renderer, font_small, summary, sidebar_x + 10, y_pos, white);
    y_pos += 20;
    
    // Color Mode (COMPACT)
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;  // Reduced from 12
    
    const char* mode_text = (color_mode == ABSOLUTE) ? "ABSOLUTE (24-28 C)" : "RELATIVE";
    SDL_Color mode_color = (color_mode == ABSOLUTE) ? 
        SDL_Color{0, 255, 100, 255} :  // Green for ABSOLUTE
        SDL_Color{255, 200, 0, 255};    // Yellow for RELATIVE
    renderTextLine(renderer, font_small, mode_text, sidebar_x + 10, y_pos, mode_color);
    y_pos += 15;  // Reduced from 43
    
    renderTextLine(renderer, font_small, "Press 'M' to toggle", sidebar_x + 10, y_pos, SDL_Color{120, 120, 120, 255});
    y_pos += 20;  // Reduced from 25
    
    // SIGNATURE ISOLATION (NEW!)
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;
    
    renderTextLine(renderer, font, "Signature Isolation", sidebar_x + 10, y_pos, white);
    y_pos += 20;
    
    // Isolation toggle status
    const char* iso_status = signature_isolation ? "ON (only top % shown)" : "OFF (full range)";
    SDL_Color iso_color = signature_isolation ? 
        SDL_Color{255, 100, 100, 255} :  // Red for ON
        SDL_Color{100, 100, 100, 255};   // Gray for OFF
    renderTextLine(renderer, font_small, iso_status, sidebar_x + 10, y_pos, iso_color);
    y_pos += 15;
    
    // Only show slider in RELATIVE mode
    if (color_mode == RELATIVE) {
        renderTextLine(renderer, font_small, "Press 'I' to toggle", sidebar_x + 10, y_pos, SDL_Color{120, 120, 120, 255});
        y_pos += 15;
        
        if (signature_isolation) {
            // Percentile slider
            renderTextLine(renderer, font_small, "Top Percentile:", sidebar_x + 10, y_pos, white);
            y_pos += 15;
            
            int iso_slider_x = sidebar_x + 10;
            int iso_slider_y = y_pos;
            int iso_slider_width = 280;
            int iso_slider_height = 15;
            
            SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
            SDL_Rect iso_slider_bg = { iso_slider_x, iso_slider_y, iso_slider_width, iso_slider_height };
            SDL_RenderFillRect(renderer, &iso_slider_bg);
            
            // Map 1% to 20% range
            float iso_normalized = (isolation_percentile - 0.01f) / 0.19f;  // 0.01 to 0.20
            int iso_fill_width = (int)(iso_slider_width * iso_normalized);
            SDL_SetRenderDrawColor(renderer, 255, 100, 100, 255);
            SDL_Rect iso_slider_fill = { iso_slider_x, iso_slider_y, iso_fill_width, iso_slider_height };
            SDL_RenderFillRect(renderer, &iso_slider_fill);
            
            int iso_handle_x = iso_slider_x + iso_fill_width - 4;
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_Rect iso_slider_handle = { iso_handle_x, iso_slider_y - 2, 8, iso_slider_height + 4 };
            SDL_RenderFillRect(renderer, &iso_slider_handle);
            
            // Handle slider interaction
            if (mouse_down && mouse_y >= iso_slider_y && mouse_y <= iso_slider_y + iso_slider_height &&
                mouse_x >= iso_slider_x && mouse_x <= iso_slider_x + iso_slider_width) {
                float new_normalized = (float)(mouse_x - iso_slider_x) / iso_slider_width;
                new_normalized = std::max(0.0f, std::min(1.0f, new_normalized));
                isolation_percentile = 0.01f + (new_normalized * 0.19f);  // 1% to 20%
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
    
    // DISTANCE MEASUREMENT (ALWAYS VISIBLE)
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 8;  // Reduced from 12
    
    renderTextLine(renderer, font, "Distance to Glass", sidebar_x + 10, y_pos, white);
    y_pos += 20;  // Reduced from 25
    
    if (distance_detected) {
        // Lock status
        const char* lock_status = signature_locked ? "LOCKED ✓" : "Unlocked";
        SDL_Color lock_color = signature_locked ? 
            SDL_Color{0, 255, 100, 255} :  // Green for locked
            SDL_Color{200, 100, 100, 255};  // Red for unlocked
        renderTextLine(renderer, font_small, lock_status, sidebar_x + 10, y_pos, lock_color);
        y_pos += 15;
        
        // Large distance display
        char dist_text[100];
        snprintf(dist_text, sizeof(dist_text), "%.2f cm", distance_cm);
        
        SDL_Color dist_color = {0, 255, 100, 255};  // Bright green
        renderTextLine(renderer, font, dist_text, sidebar_x + 10, y_pos, dist_color);
        y_pos += 20;  // Reduced from 25
        
        // Meters
        snprintf(dist_text, sizeof(dist_text), "(%.3f m)", distance_m);
        renderTextLine(renderer, font_small, dist_text, sidebar_x + 10, y_pos, white);
        y_pos += 15;  // Reduced from 18
        
        // Instructions
        if (!signature_locked) {
            renderTextLine(renderer, font_small, "(Auto-locks on detection)", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
        } else {
            renderTextLine(renderer, font_small, "Press 'U' to unlock", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
        }
    } else {
        if (signature_locked) {
            renderTextLine(renderer, font_small, "Locked but lost tracking", sidebar_x + 10, y_pos, SDL_Color{200, 100, 100, 255});
            y_pos += 15;
            renderTextLine(renderer, font_small, "Press 'U' to unlock", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
        } else {
            renderTextLine(renderer, font_small, "No red pixels detected", sidebar_x + 10, y_pos, SDL_Color{200, 100, 100, 255});
            y_pos += 15;
            renderTextLine(renderer, font_small, "(Adjust isolation %)", sidebar_x + 10, y_pos, SDL_Color{150, 150, 150, 255});
        }
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
            // Detect red pixels (top percentile - the visible signature)
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

    // Use WHITE_HOT as base - we'll override with custom colors anyway
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

    std::cout << "Thermal Object Tracker - Gradient Pattern Recognition" << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "- GRADIENT ANALYSIS: Detects signature by temperature pattern" << std::endl;
    std::cout << "- Pattern matching: Peak temperature + radial decline" << std::endl;
    std::cout << "- Ignores flat temperature regions (hands, walls, etc.)" << std::endl;
    std::cout << "- Auto-locks to best matching signature" << std::endl;
    std::cout << "- Size-based distance (adapts as signature grows/shrinks)" << std::endl;
    std::cout << "\nSignature Characteristics:" << std::endl;
    std::cout << "- Small concentrated dot (3x3 to 20x20 pixels)" << std::endl;
    std::cout << "- Hot center with gradual temperature decline" << std::endl;
    std::cout << "- Consistent pattern regardless of distance" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "- Press 'M' to toggle ABSOLUTE/RELATIVE color modes" << std::endl;
    std::cout << "- Press 'I' to toggle Signature Isolation" << std::endl;
    std::cout << "- Press 'U' to UNLOCK and redetect" << std::endl;
    std::cout << "- Drag red slider to adjust isolation % (1-20%)" << std::endl;
    std::cout << "- Press 'Q' to quit" << std::endl;
    std::cout << "\nDefault: RELATIVE + Isolation @ 10% → Auto-detects gradient!\n" << std::endl;

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

    // Initialize triangulation distance sensor
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
                    window_title << "Thermal Object Tracker - Custom Colors (CID: " << cid << ")";

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

                        // Get RAW THERMOGRAPHY DATA
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

                        // CUSTOM COLOR MAPPING - bypass hardware palette!
                        // Find min/max temps in frame
                        float min_temp = 1000.0f;
                        float max_temp = -1000.0f;
                        for (int i = 0; i < frame_width * frame_height; i++) {
                            min_temp = std::min(min_temp, temps[i]);
                            max_temp = std::max(max_temp, temps[i]);
                        }

                        // Create custom colored image based on mode
                        Uint32* pixels;
                        int pitch;
                        SDL_LockTexture(renderer->texture, nullptr, (void**)&pixels, &pitch);

                        for (int y = 0; y < frame_height; y++) {
                            for (int x = 0; x < frame_width; x++) {
                                float temp = temps[y * frame_width + x];
                                
                                SDL_Color color;
                                if (renderer->color_mode == ABSOLUTE) {
                                    // ABSOLUTE: Fixed 24-28°C range
                                    color = mapTemperatureAbsolute(temp);
                                } else {
                                    // RELATIVE: Frame min/max with optional isolation
                                    color = mapTemperatureRelative(temp, min_temp, max_temp, 
                                                                    renderer->signature_isolation,
                                                                    renderer->isolation_percentile);
                                }
                                
                                // Convert to ARGB8888
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

                        for (size_t i = 0; i < renderer->detected_objects.size(); i++) {
                            drawThermalObject(renderer->renderer, renderer->font,
                                renderer->detected_objects[i], scale_factor);
                        }

                        drawCenterCrosshair(renderer->renderer, renderer->font,
                            window_width - SIDEBAR_WIDTH, window_height,
                            renderer->center_pixel_temp);

                        // Measure distance using SIZE-BASED method with LOCKING
                        TriangulationDistanceSensor::DistanceResult distance_result = 
                            distance_sensor.measure(renderer->detected_objects);
                        
                        // Print distance to console if detected
                        if (distance_result.detected) {
                            std::cout << (distance_result.is_locked ? "[LOCKED] " : "[Unlocked] ")
                                      << "Distance: " << distance_result.distance_cm << " cm | " 
                                      << "Area: " << distance_result.pixel_area << " px² | " 
                                      << "Size: " << distance_result.pixel_width << "x" 
                                      << distance_result.pixel_height << " | Temp: " 
                                      << distance_result.avg_temp << "°C" << std::endl;
                        }

                        drawSidebar(renderer->renderer, renderer->font, renderer->font_small,
                            renderer->detected_objects, renderer->center_pixel_temp,
                            center_x_thermal, center_y_thermal,
                            renderer->color_mode, renderer->signature_isolation, 
                            renderer->isolation_percentile,
                            distance_result.detected, distance_result.is_locked,
                            distance_result.distance_m, distance_result.distance_cm,
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
                // Toggle color mode with 'M' key
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
                // Toggle signature isolation with 'I' key
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
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_u) {
                // Unlock signature with 'U' key
                distance_sensor.unlockSignature();
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