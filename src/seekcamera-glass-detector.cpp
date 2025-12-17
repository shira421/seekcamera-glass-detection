/*
 * Thermal Distance Sensor - Dual Emitter Triangulation
 * Clean version - Distance and Tilt output only
 * C++11/14 compatible
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
#include <chrono>

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

// Configuration
const float ISOLATION_THRESHOLD = 0.15f;  // Top 15% of temperatures shown
const int SIDEBAR_WIDTH = 280;

// Thermal Object structure
struct ThermalObject {
    int id;
    int x, y, width, height;
    float min_temp, max_temp, avg_temp;
    SDL_Color box_color;
};

// Renderer structure
struct seekrenderer_t {
    seekcamera_t* camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
    TTF_Font* font_large;
    std::mutex the_mutex;
    std::atomic<bool> is_active;
    std::atomic<bool> is_dirty;
    seekcamera_frame_t* frame;

    std::vector<ThermalObject> detected_objects;
    int next_object_id;
    float center_pixel_temp;
    bool isolation_mode;  // Toggle between relative and relative+isolation

    seekrenderer_t() : camera(nullptr), window(nullptr), renderer(nullptr),
        texture(nullptr), font(nullptr), font_large(nullptr),
        frame(nullptr), next_object_id(1), center_pixel_temp(0.0f),
        isolation_mode(true) {  // Start with isolation ON
        is_active.store(false);
        is_dirty.store(false);
    }
};

static std::atomic<bool> g_exit_requested;
static std::map<std::string, seekrenderer_t*> g_renderers;
static std::mutex g_mutex;
static std::condition_variable g_condition_variable;
static std::atomic<bool> g_is_dirty;

// Color mapping - Relative mode with optional signature isolation
SDL_Color mapTemperature(float temp, float min_temp, float max_temp, bool isolation_mode) {
    SDL_Color color;
    
    float range = max_temp - min_temp;
    if (range < 0.01f) {
        color.r = 20; color.g = 20; color.b = 20; color.a = 255;
        return color;
    }
    
    float normalized;
    
    if (isolation_mode) {
        // Only show top 15% of temperature range (isolation mode)
        float threshold = max_temp - (range * ISOLATION_THRESHOLD);
        
        if (temp < threshold) {
            color.r = 0; color.g = 0; color.b = 0; color.a = 255;
            return color;
        }
        
        // Map within the top percentile range
        normalized = (temp - threshold) / (range * ISOLATION_THRESHOLD);
    } else {
        // Full relative range (no isolation)
        normalized = (temp - min_temp) / range;
    }
    
    // Apply steep curve for better visibility
    normalized = std::pow(normalized, 0.15f);
    
    // Blue → Cyan → Green → Yellow → Red
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

// Triangulation Distance Sensor
class TriangulationDistanceSensor {
private:
    // Physical setup
    const float BASELINE_ABOVE_CM = 2.5f;
    const float BASELINE_LEFT_CM = 3.3f;
    const float HFOV_DEGREES = 56.0f;
    const float SENSOR_WIDTH = 320.0f;
    const float SENSOR_HEIGHT = 240.0f;
    
    // Derived
    float VFOV_DEGREES;
    float focal_length_pixels;
    
    // Temporal filtering
    std::vector<float> distance_history;
    std::vector<float> yaw_history;
    std::vector<float> pitch_history;
    const size_t FILTER_SIZE;
    
    // EMA state
    float ema_distance;
    float ema_yaw;
    float ema_pitch;
    bool ema_init;
    
    // Deadband state
    float last_dist;
    float last_yaw;
    float last_pitch;
    
    float smoothValue(std::vector<float>& buffer, float new_val) {
        buffer.push_back(new_val);
        if (buffer.size() > FILTER_SIZE) {
            buffer.erase(buffer.begin());
        }
        float sum = 0;
        for (size_t i = 0; i < buffer.size(); i++) {
            sum += buffer[i];
        }
        return sum / buffer.size();
    }
    
    struct HotSpot {
        float x, y, temp;
        bool valid;
    };
    
    HotSpot findHotSpotCentroid(float* temps, int width, int height, 
                                 int center_x, int center_y, float max_temp) {
        HotSpot spot;
        spot.valid = false;
        spot.x = 0;
        spot.y = 0;
        spot.temp = 0;
        
        float threshold = max_temp - 2.0f;
        float sum_temp = 0.0f, sum_x = 0.0f, sum_y = 0.0f;
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
            spot.x = (float)center_x;
            spot.y = (float)center_y;
            spot.temp = max_temp;
            spot.valid = true;
        }
        
        return spot;
    }
    
public:
    TriangulationDistanceSensor() : FILTER_SIZE(8), ema_distance(0), ema_yaw(0), ema_pitch(0),
                                     ema_init(false), last_dist(0), last_yaw(0), last_pitch(0) {
        float aspect_ratio = SENSOR_HEIGHT / SENSOR_WIDTH;
        float hfov_half_rad = (HFOV_DEGREES / 2.0f) * (float)M_PI / 180.0f;
        float vfov_half_rad = atan(tan(hfov_half_rad) * aspect_ratio);
        VFOV_DEGREES = (vfov_half_rad * 180.0f / (float)M_PI) * 2.0f;
        focal_length_pixels = (SENSOR_WIDTH / 2.0f) / tan(hfov_half_rad);
    }
    
    struct DistanceResult {
        bool detected;
        float distance_cm;
        float camera_yaw_deg;    // Camera pointing direction (horizontal) - positive = camera points right
        float camera_pitch_deg;  // Camera pointing direction (vertical) - positive = camera points up
        
        // Spot positions for visualization
        int spot1_x, spot1_y;  // ABOVE emitter spot
        int spot2_x, spot2_y;  // LEFT emitter spot
        float spot1_temp, spot2_temp;
    };
    
    DistanceResult calculate(float* temps, int width, int height) {
        DistanceResult result;
        result.detected = false;
        result.distance_cm = 0;
        result.camera_yaw_deg = 0;
        result.camera_pitch_deg = 0;
        result.spot1_x = 0;
        result.spot1_y = 0;
        result.spot2_x = 0;
        result.spot2_y = 0;
        result.spot1_temp = 0;
        result.spot2_temp = 0;
        
        // Calculate background and thresholds
        std::vector<float> all_temps;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                all_temps.push_back(temps[y * width + x]);
            }
        }
        std::sort(all_temps.begin(), all_temps.end());
        float background_temp = all_temps[all_temps.size() / 2];
        float temp_80th = all_temps[(all_temps.size() * 80) / 100];
        
        // Detection threshold
        const float MIN_TEMP_ABOVE_BG = 0.1f;
        const int NEIGHBORHOOD = 2;
        float detection_threshold = std::min(background_temp + MIN_TEMP_ABOVE_BG, temp_80th);
        
        // Find local maxima
        struct LocalMax {
            int x, y;
            float temp;
        };
        
        std::vector<LocalMax> local_maxima;
        
        for (int y = NEIGHBORHOOD; y < height - NEIGHBORHOOD; y++) {
            for (int x = NEIGHBORHOOD; x < width - NEIGHBORHOOD; x++) {
                float center_temp = temps[y * width + x];
                if (center_temp < detection_threshold) continue;
                
                bool is_local_max = true;
                for (int dy = -NEIGHBORHOOD; dy <= NEIGHBORHOOD && is_local_max; dy++) {
                    for (int dx = -NEIGHBORHOOD; dx <= NEIGHBORHOOD; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        if (temps[(y+dy) * width + (x+dx)] > center_temp) {
                            is_local_max = false;
                            break;
                        }
                    }
                }
                
                if (is_local_max) {
                    LocalMax lm;
                    lm.x = x;
                    lm.y = y;
                    lm.temp = center_temp;
                    local_maxima.push_back(lm);
                }
            }
        }
        
        // Sort by temperature (descending)
        for (size_t i = 0; i < local_maxima.size(); i++) {
            for (size_t j = i + 1; j < local_maxima.size(); j++) {
                if (local_maxima[j].temp > local_maxima[i].temp) {
                    LocalMax tmp = local_maxima[i];
                    local_maxima[i] = local_maxima[j];
                    local_maxima[j] = tmp;
                }
            }
        }
        
        if (local_maxima.size() < 2) {
            return result;
        }
        
        // Find two spots with sufficient separation
        const float MIN_SEPARATION = 10.0f;
        LocalMax spot1 = local_maxima[0];
        LocalMax spot2;
        bool found_second = false;
        
        for (size_t i = 1; i < local_maxima.size(); i++) {
            float dx = (float)(local_maxima[i].x - spot1.x);
            float dy = (float)(local_maxima[i].y - spot1.y);
            float sep = sqrt(dx*dx + dy*dy);
            
            if (sep >= MIN_SEPARATION) {
                spot2 = local_maxima[i];
                found_second = true;
                break;
            }
        }
        
        if (!found_second) return result;
        
        // Get sub-pixel centroids
        HotSpot hotspot1 = findHotSpotCentroid(temps, width, height, spot1.x, spot1.y, spot1.temp);
        HotSpot hotspot2 = findHotSpotCentroid(temps, width, height, spot2.x, spot2.y, spot2.temp);
        
        if (!hotspot1.valid || !hotspot2.valid) return result;
        
        // Identify spots by relative position
        HotSpot spot_above, spot_left;
        float dx = hotspot2.x - hotspot1.x;
        float dy = hotspot2.y - hotspot1.y;
        
        if (fabs(dy) > fabs(dx)) {
            if (hotspot1.y <= hotspot2.y) {
                spot_above = hotspot1; spot_left = hotspot2;
            } else {
                spot_above = hotspot2; spot_left = hotspot1;
            }
        } else {
            if (hotspot1.x <= hotspot2.x) {
                spot_left = hotspot1; spot_above = hotspot2;
            } else {
                spot_left = hotspot2; spot_above = hotspot1;
            }
        }
        
        // Calculate angles - these show where SPOT appears relative to center
        float center_x = 160.0f, center_y = 120.0f;
        
        // Spot offsets from center (in pixels)
        float spot_above_offset_x = spot_above.x - center_x;  // Positive = spot is right of center
        float spot_above_offset_y = spot_above.y - center_y;  // Positive = spot is below center
        float spot_left_offset_x = spot_left.x - center_x;
        float spot_left_offset_y = spot_left.y - center_y;
        
        // Convert to angles (degrees) - using FOV scaling
        // HFOV = 56 degrees across 320 pixels, so 28 degrees per 160 pixels from center
        // VFOV ~= 44 degrees across 240 pixels, so 22 degrees per 120 pixels from center
        float spot_yaw_deg = (spot_above_offset_x / 160.0f) * 28.0f;   // Where spot appears horizontally
        float spot_pitch_deg = (spot_left_offset_y / 120.0f) * 22.0f;  // Where spot appears vertically
        
        // CAMERA direction is OPPOSITE to spot position (both axes inverted)
        // If spot appears on RIGHT (positive), camera is pointing LEFT (negative yaw)
        // If spot appears BELOW center (positive offset), camera is pointing UP (positive pitch)
        float camera_yaw_raw = -spot_yaw_deg;    // Invert: spot right = camera left
        float camera_pitch_raw = spot_pitch_deg; // spot below (positive) = camera up (positive)
        
        // Distance from pixel separation
        float dx_pixels = spot_above.x - spot_left.x;
        float dy_pixels = spot_above.y - spot_left.y;
        float separation_pixels = sqrt(dx_pixels * dx_pixels + dy_pixels * dy_pixels);
        
        const float K_CALIBRATION = 1335.0f;
        float d_separation = K_CALIBRATION / separation_pixels;
        
        // Account for reflection
        float d_unified = d_separation / 2.0f;
        
        // Filtering
        float d_ma = smoothValue(distance_history, d_unified);
        float yaw_ma = smoothValue(yaw_history, camera_yaw_raw);
        float pitch_ma = smoothValue(pitch_history, camera_pitch_raw);
        
        // Exponential smoothing
        const float ALPHA = 0.5f;
        
        if (!ema_init) {
            ema_distance = d_ma;
            ema_yaw = yaw_ma;
            ema_pitch = pitch_ma;
            ema_init = true;
        } else {
            ema_distance = ALPHA * d_ma + (1.0f - ALPHA) * ema_distance;
            ema_yaw = ALPHA * yaw_ma + (1.0f - ALPHA) * ema_yaw;
            ema_pitch = ALPHA * pitch_ma + (1.0f - ALPHA) * ema_pitch;
        }
        
        // Deadband
        const float DEADBAND_DIST = 0.15f;
        const float DEADBAND_ANGLE = 0.3f;
        
        float final_dist = (fabs(ema_distance - last_dist) > DEADBAND_DIST) ? ema_distance : last_dist;
        float final_yaw = (fabs(ema_yaw - last_yaw) > DEADBAND_ANGLE) ? ema_yaw : last_yaw;
        float final_pitch = (fabs(ema_pitch - last_pitch) > DEADBAND_ANGLE) ? ema_pitch : last_pitch;
        
        last_dist = final_dist;
        last_yaw = final_yaw;
        last_pitch = final_pitch;
        
        // Populate result
        result.detected = true;
        result.distance_cm = final_dist;
        result.camera_yaw_deg = final_yaw;
        result.camera_pitch_deg = final_pitch;
        result.spot1_x = (int)round(spot_above.x);
        result.spot1_y = (int)round(spot_above.y);
        result.spot1_temp = spot_above.temp;
        result.spot2_x = (int)round(spot_left.x);
        result.spot2_y = (int)round(spot_left.y);
        result.spot2_temp = spot_left.temp;
        
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

                    const int ddx[] = {-1, 1, 0, 0};
                    const int ddy[] = {0, 0, -1, 1};

                    for (int i = 0; i < 4; i++) {
                        int nx = cx + ddx[i];
                        int ny = cy + ddy[i];

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

// Signature pattern analysis
struct SignatureScore {
    float gradient_quality, compactness, peak_centrality;
    float aspect_ratio_score, circularity, overall_score;
};

SignatureScore analyzeSignaturePattern(float* temps, int width, int height, const ThermalObject& obj) {
    SignatureScore score;
    score.gradient_quality = 0;
    score.compactness = 0;
    score.peak_centrality = 0;
    score.aspect_ratio_score = 0;
    score.circularity = 0;
    score.overall_score = 0;
    
    float aspect_ratio = (float)obj.width / (float)obj.height;
    if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) return score;
    
    // Find peak
    int peak_x = obj.x, peak_y = obj.y;
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
    
    // Aspect ratio score
    score.aspect_ratio_score = std::max(0.0f, 1.0f - (float)fabs(aspect_ratio - 1.0f) * 2.0f);
    
    // Peak centrality
    float center_x = obj.x + obj.width / 2.0f;
    float center_y = obj.y + obj.height / 2.0f;
    float peak_offset = sqrt(pow((float)peak_x - center_x, 2.0f) + pow((float)peak_y - center_y, 2.0f));
    float max_offset = sqrt(pow(obj.width / 2.0f, 2.0f) + pow(obj.height / 2.0f, 2.0f));
    score.peak_centrality = max_offset > 0 ? (1.0f - (peak_offset / max_offset)) : 1.0f;
    
    if (score.peak_centrality < 0.5f) return score;
    
    // Gradient quality
    int directions[8][2] = {{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};
    float gradient_scores[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int valid_directions = 0;
    
    for (int dir = 0; dir < 8; dir++) {
        int ddx = directions[dir][0];
        int ddy = directions[dir][1];
        float last_temp = peak_temp;
        int consecutive_declines = 0, total_samples = 0;
        float temp_drop_total = 0.0f;
        
        for (int step = 1; step <= 5; step++) {
            int sample_x = peak_x + ddx * step;
            int sample_y = peak_y + ddy * step;
            
            if (sample_x < 0 || sample_x >= width || sample_y < 0 || sample_y >= height) break;
            if (sample_x < obj.x || sample_x >= obj.x + obj.width) break;
            if (sample_y < obj.y || sample_y >= obj.y + obj.height) break;
            
            float sample_temp = temps[sample_y * width + sample_x];
            if (sample_temp < last_temp) {
                consecutive_declines++;
                temp_drop_total += last_temp - sample_temp;
            }
            last_temp = sample_temp;
            total_samples++;
        }
        
        if (total_samples > 0) {
            float decline_ratio = (float)consecutive_declines / total_samples;
            float avg_drop = temp_drop_total / total_samples;
            gradient_scores[dir] = decline_ratio * 0.7f + std::min(avg_drop / 2.0f, 1.0f) * 0.3f;
            valid_directions++;
        }
    }
    
    if (valid_directions > 0) {
        float sum = 0;
        for (int i = 0; i < 8; i++) sum += gradient_scores[i];
        score.gradient_quality = sum / valid_directions;
    }
    
    if (score.gradient_quality < 0.4f) return score;
    
    // Circularity and compactness
    int area = obj.width * obj.height;
    int perimeter = 2 * (obj.width + obj.height);
    score.circularity = std::min((4.0f * 3.14159f * area) / (float)(perimeter * perimeter), 1.0f);
    
    if (area < 9) score.compactness = 0.0f;
    else if (area > 400) score.compactness = 0.3f;
    else score.compactness = std::max(0.3f, 1.0f - (float)fabs(area - 100.0f) / 100.0f);
    
    score.overall_score = score.gradient_quality * 0.40f + score.peak_centrality * 0.25f +
                          score.aspect_ratio_score * 0.15f + score.circularity * 0.10f +
                          score.compactness * 0.10f;
    
    return score;
}

std::vector<ThermalObject> detectThermalObjects(float* temps, int width, int height, int& next_id) {
    std::vector<ThermalObject> objects;
    
    float min_temp = 1000.0f, max_temp = -1000.0f;
    for (int i = 0; i < width * height; i++) {
        min_temp = std::min(min_temp, temps[i]);
        max_temp = std::max(max_temp, temps[i]);
    }
    
    float range = max_temp - min_temp;
    if (range < 0.01f) return objects;
    
    float threshold = max_temp - (range * ISOLATION_THRESHOLD);
    
    // Create binary map
    std::vector<std::vector<bool> > hot_pixels(height, std::vector<bool>(width, false));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] >= threshold) {
                hot_pixels[y][x] = true;
            }
        }
    }
    
    std::vector<std::vector<int> > labels = findConnectedComponents(hot_pixels, width, height);
    
    // Build objects from labels
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
    
    // Score and filter
    std::vector<std::pair<ThermalObject, SignatureScore> > scored;
    
    for (std::map<int, ThermalObject>::iterator it = object_map.begin(); it != object_map.end(); ++it) {
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
            scored.push_back(std::make_pair(obj, sig_score));
        }
    }
    
    // Sort by score (descending)
    for (size_t i = 0; i < scored.size(); i++) {
        for (size_t j = i + 1; j < scored.size(); j++) {
            if (scored[j].second.overall_score > scored[i].second.overall_score) {
                std::pair<ThermalObject, SignatureScore> tmp = scored[i];
                scored[i] = scored[j];
                scored[j] = tmp;
            }
        }
    }
    
    // Return top 2
    for (size_t i = 0; i < scored.size() && i < 2; i++) {
        ThermalObject obj = scored[i].first;
        float score_val = scored[i].second.overall_score;
        
        if (score_val > 0.70f) {
            obj.box_color.r = 0;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
            obj.box_color.a = 255;
        } else if (score_val > 0.60f) {
            obj.box_color.r = 255;
            obj.box_color.g = 255;
            obj.box_color.b = 0;
            obj.box_color.a = 255;
        } else {
            obj.box_color.r = 255;
            obj.box_color.g = 150;
            obj.box_color.b = 0;
            obj.box_color.a = 255;
        }
        
        objects.push_back(obj);
    }
    
    return objects;
}

// Drawing functions
void renderText(SDL_Renderer* renderer, TTF_Font* font, const char* text, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Blended(font, text, color);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect rect;
            rect.x = x;
            rect.y = y;
            rect.w = surface->w;
            rect.h = surface->h;
            SDL_RenderCopy(renderer, texture, NULL, &rect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

void drawCrosshair(SDL_Renderer* renderer, int width, int height) {
    int cx = width / 2, cy = height / 2;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    // Horizontal lines (2 pixels thick)
    SDL_RenderDrawLine(renderer, cx - 6, cy, cx + 6, cy);
    SDL_RenderDrawLine(renderer, cx - 6, cy + 1, cx + 6, cy + 1);
    // Vertical lines (2 pixels thick)
    SDL_RenderDrawLine(renderer, cx, cy - 6, cx, cy + 6);
    SDL_RenderDrawLine(renderer, cx + 1, cy - 6, cx + 1, cy + 6);
}

void drawSpotMarker(SDL_Renderer* renderer, TTF_Font* font, int x, int y, 
                    float temp, SDL_Color color, int scale, const char* label) {
    int dx = x * scale, dy = y * scale;
    
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
    SDL_Rect box;
    box.x = dx - 3;
    box.y = dy - 3;
    box.w = 6;
    box.h = 6;
    SDL_RenderDrawRect(renderer, &box);
    
    if (font && label) {
        // Create label with temperature
        char full_label[64];
        snprintf(full_label, sizeof(full_label), "%s %.1fC", label, temp);
        
        SDL_Surface* surface = TTF_RenderText_Solid(font, full_label, color);
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            if (texture) {
                SDL_Rect rect;
                rect.x = dx + 6;
                rect.y = dy - 8;
                rect.w = surface->w;
                rect.h = surface->h;
                SDL_RenderCopy(renderer, texture, NULL, &rect);
                SDL_DestroyTexture(texture);
            }
            SDL_FreeSurface(surface);
        }
    }
}

void drawCameraMarker(SDL_Renderer* renderer, TTF_Font* font, 
                      int spot1_x, int spot1_y, int spot2_x, int spot2_y, int scale) {
    // Camera is pointing at: X from ABOVE spot, Y from LEFT spot
    // This gives us the intersection point (below ABOVE, right of LEFT)
    int camera_point_x = spot1_x * scale;  // X coordinate from ABOVE spot
    int camera_point_y = spot2_y * scale;  // Y coordinate from LEFT spot
    
    // Draw small red dot at where camera is pointing
    SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    SDL_Rect center_dot;
    center_dot.x = camera_point_x - 2;
    center_dot.y = camera_point_y - 2;
    center_dot.w = 4;
    center_dot.h = 4;
    SDL_RenderFillRect(renderer, &center_dot);
}

void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_large,
                 bool detected, float distance_cm, float camera_yaw_deg, float camera_pitch_deg,
                 float spot1_temp, float spot2_temp, bool isolation_mode,
                 int window_width, int window_height) {
    
    int sidebar_x = window_width - SIDEBAR_WIDTH;
    
    // Background
    SDL_SetRenderDrawColor(renderer, 25, 25, 25, 255);
    SDL_Rect sidebar;
    sidebar.x = sidebar_x;
    sidebar.y = 0;
    sidebar.w = SIDEBAR_WIDTH;
    sidebar.h = window_height;
    SDL_RenderFillRect(renderer, &sidebar);
    
    SDL_Color white;
    white.r = 255; white.g = 255; white.b = 255; white.a = 255;
    SDL_Color green;
    green.r = 0; green.g = 255; green.b = 100; green.a = 255;
    SDL_Color red;
    red.r = 255; red.g = 100; red.b = 100; red.a = 255;
    SDL_Color yellow;
    yellow.r = 255; yellow.g = 200; yellow.b = 0; yellow.a = 255;
    SDL_Color gray;
    gray.r = 80; gray.g = 80; gray.b = 80; gray.a = 255;
    SDL_Color dark_gray;
    dark_gray.r = 120; dark_gray.g = 120; dark_gray.b = 120; dark_gray.a = 255;
    SDL_Color cyan;
    cyan.r = 0; cyan.g = 255; cyan.b = 255; cyan.a = 255;
    
    int y = 15;
    
    if (detected) {
        // Distance - large display (2 decimal places)
        char dist_text[32];
        snprintf(dist_text, sizeof(dist_text), "%.2f cm", distance_cm);
        renderText(renderer, font_large, dist_text, sidebar_x + 15, y, green);
        y += 45;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Spot temperatures
        renderText(renderer, font, "SPOT TEMPERATURES", sidebar_x + 15, y, white);
        y += 22;
        
        char temp_text[48];
        snprintf(temp_text, sizeof(temp_text), "Above:  %.1f C", spot1_temp);
        renderText(renderer, font, temp_text, sidebar_x + 20, y, green);
        y += 18;
        
        snprintf(temp_text, sizeof(temp_text), "Left:   %.1f C", spot2_temp);
        renderText(renderer, font, temp_text, sidebar_x + 20, y, cyan);
        y += 25;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Camera orientation (tilt angles) - 2 decimal places
        renderText(renderer, font, "CAMERA ORIENTATION", sidebar_x + 15, y, white);
        y += 25;
        
        // Yaw (camera horizontal pointing direction)
        char yaw_text[48];
        snprintf(yaw_text, sizeof(yaw_text), "Yaw:   %+7.2f deg", camera_yaw_deg);
        SDL_Color yaw_color = (fabs(camera_yaw_deg) < 2.0f) ? green : yellow;
        renderText(renderer, font, yaw_text, sidebar_x + 20, y, yaw_color);
        y += 22;
        
        // Pitch (camera vertical pointing direction)
        char pitch_text[48];
        snprintf(pitch_text, sizeof(pitch_text), "Pitch: %+7.2f deg", camera_pitch_deg);
        SDL_Color pitch_color = (fabs(camera_pitch_deg) < 2.0f) ? green : yellow;
        renderText(renderer, font, pitch_text, sidebar_x + 20, y, pitch_color);
        y += 30;
        
        // Visual tilt indicator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        renderText(renderer, font, "CAMERA TILT", sidebar_x + 15, y, white);
        y += 25;
        
        // Draw tilt crosshair box
        int box_size = 100;
        int box_x = sidebar_x + (SIDEBAR_WIDTH - box_size) / 2;
        int box_y = y;
        
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_Rect tilt_box;
        tilt_box.x = box_x;
        tilt_box.y = box_y;
        tilt_box.w = box_size;
        tilt_box.h = box_size;
        SDL_RenderFillRect(renderer, &tilt_box);
        
        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawRect(renderer, &tilt_box);
        
        // Center lines
        int bcx = box_x + box_size/2, bcy = box_y + box_size/2;
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, box_x, bcy, box_x + box_size, bcy);
        SDL_RenderDrawLine(renderer, bcx, box_y, bcx, box_y + box_size);
        
        // Camera direction indicator dot
        // Full FOV: HFOV = 56 deg (±28), VFOV ~= 44 deg (±22)
        // Camera tilts left = negative yaw = dot goes left
        // Camera tilts up = positive pitch = dot goes up (subtract in screen coords since Y increases down)
        float max_yaw = 28.0f;   // Half of HFOV
        float max_pitch = 22.0f; // Half of VFOV
        int dot_x = bcx + (int)(camera_yaw_deg / max_yaw * (box_size/2));
        int dot_y = bcy - (int)(camera_pitch_deg / max_pitch * (box_size/2));  // Subtract: positive pitch (up) = lower screen Y (up)
        dot_x = std::max(box_x + 2, std::min(box_x + box_size - 2, dot_x));
        dot_y = std::max(box_y + 2, std::min(box_y + box_size - 2, dot_y));
        
        SDL_Color dot_color = (fabs(camera_yaw_deg) < 2.0f && fabs(camera_pitch_deg) < 2.0f) ? green : yellow;
        SDL_SetRenderDrawColor(renderer, dot_color.r, dot_color.g, dot_color.b, 255);
        SDL_Rect dot;
        dot.x = dot_x - 1;
        dot.y = dot_y - 1;
        dot.w = 3;
        dot.h = 3;
        SDL_RenderFillRect(renderer, &dot);
        
        // Labels for the tilt box
        renderText(renderer, font, "L", box_x - 12, bcy - 7, gray);
        renderText(renderer, font, "R", box_x + box_size + 4, bcy - 7, gray);
        renderText(renderer, font, "U", bcx - 4, box_y - 16, gray);
        renderText(renderer, font, "D", bcx - 4, box_y + box_size + 2, gray);
        
    } else {
        renderText(renderer, font, "Searching...", sidebar_x + 15, y, red);
        y += 25;
        renderText(renderer, font, "No heat spots detected", sidebar_x + 15, y, dark_gray);
    }
    
    // Footer - mode and controls
    y = window_height - 55;
    
    // Isolation mode indicator
    const char* mode_text = isolation_mode ? "ISOLATION: ON (top 15%)" : "ISOLATION: OFF (full range)";
    SDL_Color mode_color = isolation_mode ? green : yellow;
    renderText(renderer, font, mode_text, sidebar_x + 15, y, mode_color);
    y += 18;
    
    renderText(renderer, font, "Press I to toggle isolation", sidebar_x + 15, y, gray);
    y += 18;
    renderText(renderer, font, "Press Q to quit", sidebar_x + 15, y, gray);
}

// Camera callbacks
void handle_camera_frame_available(seekcamera_t* camera, seekcamera_frame_t* camera_frame, void* user_data) {
    (void)camera;
    seekrenderer_t* renderer = (seekrenderer_t*)user_data;

    std::lock_guard<std::mutex> guard(renderer->the_mutex);

    seekcamera_frame_lock(camera_frame);
    renderer->is_dirty.store(true);
    renderer->frame = camera_frame;

    seekframe_t* thermal_frame = NULL;
    if (seekcamera_frame_get_frame_by_format(camera_frame,
        SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame) == SEEKCAMERA_SUCCESS) {

        float* temps = (float*)seekframe_get_data(thermal_frame);
        int width = seekframe_get_width(thermal_frame);
        int height = seekframe_get_height(thermal_frame);

        if (temps && width > 0 && height > 0) {
            renderer->detected_objects = detectThermalObjects(temps, width, height, renderer->next_object_id);
            renderer->center_pixel_temp = temps[(height/2) * width + (width/2)];
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

    seekrenderer_t* renderer = g_renderers[chipID] ? g_renderers[chipID] : new seekrenderer_t();
    renderer->is_active.store(true);
    renderer->camera = camera;

    seekcamera_register_frame_available_callback(camera, handle_camera_frame_available, renderer);
    seekcamera_set_color_palette(camera, SEEKCAMERA_COLOR_PALETTE_WHITE_HOT);
    
    uint32_t formats = SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
    seekcamera_capture_session_start(camera, formats);

    g_renderers[chipID] = renderer;
    std::cout << "Camera connected" << std::endl;
}

void handle_camera_disconnect(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID((char*)&cid);
    
    seekcamera_capture_session_stop(camera);
    
    if (g_renderers[chipID]) {
        g_renderers[chipID]->is_active.store(false);
        g_is_dirty.store(true);
    }
    std::cout << "Camera disconnected" << std::endl;
}

void handle_camera_ready_to_pair(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    seekcamera_store_calibration_data(camera, NULL, NULL, NULL);
    handle_camera_connect(camera, event_status, user_data);
}

void camera_event_callback(seekcamera_t* camera, seekcamera_manager_event_t event, 
                           seekcamera_error_t event_status, void* user_data) {
    switch (event) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT:
            handle_camera_connect(camera, event_status, user_data);
            break;
        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
            handle_camera_disconnect(camera, event_status, user_data);
            break;
        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            handle_camera_ready_to_pair(camera, event_status, user_data);
            break;
        default:
            break;
    }
}

void cleanup_renderer(seekrenderer_t* renderer) {
    if (!renderer) return;
    
    if (renderer->is_active.load()) {
        seekcamera_capture_session_stop(renderer->camera);
    }
    renderer->is_active.store(false);
    
    if (renderer->font) {
        TTF_CloseFont(renderer->font);
        renderer->font = NULL;
    }
    if (renderer->font_large) {
        TTF_CloseFont(renderer->font_large);
        renderer->font_large = NULL;
    }
    if (renderer->texture) {
        SDL_DestroyTexture(renderer->texture);
        renderer->texture = NULL;
    }
    if (renderer->renderer) {
        SDL_DestroyRenderer(renderer->renderer);
        renderer->renderer = NULL;
    }
    if (renderer->window) {
        SDL_DestroyWindow(renderer->window);
        renderer->window = NULL;
    }
}

static void signal_callback(int signum) {
    (void)signum;
    g_exit_requested.store(true);
}

int main() {
    g_exit_requested.store(false);
    g_is_dirty.store(false);

    signal(SIGINT, signal_callback);
    signal(SIGTERM, signal_callback);

    std::cout << "Thermal Distance Sensor - Dual Emitter Triangulation" << std::endl;
    std::cout << "Press I to toggle isolation mode" << std::endl;
    std::cout << "Press Q to quit" << std::endl << std::endl;

    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    seekcamera_manager_t* manager = NULL;
    if (seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager" << std::endl;
        return 1;
    }
    seekcamera_manager_register_event_callback(manager, camera_event_callback, NULL);

    TriangulationDistanceSensor distance_sensor;
    
    // For terminal output rate limiting
    std::chrono::steady_clock::time_point last_print = std::chrono::steady_clock::now();
    const std::chrono::milliseconds print_interval(200);

    while (!g_exit_requested.load()) {
        std::unique_lock<std::mutex> lock(g_mutex);

        if (g_condition_variable.wait_for(lock, std::chrono::milliseconds(150),
            []() { return g_is_dirty.load(); })) {
            g_is_dirty.store(false);

            for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                 it != g_renderers.end(); ++it) {
                seekrenderer_t* renderer = it->second;
                if (!renderer || !renderer->is_active.load()) continue;

                // Create window if needed
                if (!renderer->window) {
                    renderer->window = SDL_CreateWindow("Thermal Distance Sensor", 
                        100, 100, 0, 0, SDL_WINDOW_HIDDEN);
                    renderer->renderer = SDL_CreateRenderer(renderer->window, -1, 
                        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
                    
                    renderer->font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
                    if (!renderer->font) renderer->font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
                    
                    renderer->font_large = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28);
                    if (!renderer->font_large) renderer->font_large = TTF_OpenFont("C:\\Windows\\Fonts\\arialbd.ttf", 28);
                }

                std::lock_guard<std::mutex> guard(renderer->the_mutex);

                if (!renderer->is_dirty.load() || !renderer->frame) continue;

                seekframe_t* thermal_frame = NULL;
                if (seekcamera_frame_get_frame_by_format(renderer->frame,
                    SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame) != SEEKCAMERA_SUCCESS) {
                    seekcamera_frame_unlock(renderer->frame);
                    continue;
                }

                int fw = seekframe_get_width(thermal_frame);
                int fh = seekframe_get_height(thermal_frame);
                float* temps = (float*)seekframe_get_data(thermal_frame);

                // Create texture if needed
                if (!renderer->texture) {
                    const int scale = 2;
                    int ww = fw * scale + SIDEBAR_WIDTH;
                    int wh = fh * scale;
                    
                    SDL_RenderSetLogicalSize(renderer->renderer, ww, wh);
                    renderer->texture = SDL_CreateTexture(renderer->renderer, 
                        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, fw, fh);
                    SDL_SetWindowSize(renderer->window, ww, wh);
                    SDL_ShowWindow(renderer->window);
                }

                // Find temp range
                float min_t = 1000.0f, max_t = -1000.0f;
                for (int i = 0; i < fw * fh; i++) {
                    min_t = std::min(min_t, temps[i]);
                    max_t = std::max(max_t, temps[i]);
                }

                // Render thermal image
                Uint32* pixels;
                int pitch;
                SDL_LockTexture(renderer->texture, NULL, (void**)&pixels, &pitch);

                for (int y = 0; y < fh; y++) {
                    for (int x = 0; x < fw; x++) {
                        SDL_Color c = mapTemperature(temps[y * fw + x], min_t, max_t, renderer->isolation_mode);
                        pixels[y * (pitch/4) + x] = (255 << 24) | (c.r << 16) | (c.g << 8) | c.b;
                    }
                }
                SDL_UnlockTexture(renderer->texture);

                int ww, wh;
                SDL_GetWindowSize(renderer->window, &ww, &wh);

                SDL_Rect thermal_rect;
                thermal_rect.x = 0;
                thermal_rect.y = 0;
                thermal_rect.w = ww - SIDEBAR_WIDTH;
                thermal_rect.h = wh;
                SDL_RenderCopy(renderer->renderer, renderer->texture, NULL, &thermal_rect);

                const int scale = 2;
                
                // Draw crosshair
                drawCrosshair(renderer->renderer, ww - SIDEBAR_WIDTH, wh);

                // Calculate distance
                TriangulationDistanceSensor::DistanceResult result = distance_sensor.calculate(temps, fw, fh);
                
                if (result.detected) {
                    // Draw spot markers with temperatures
                    SDL_Color spot1_color;
                    spot1_color.r = 0; spot1_color.g = 255; spot1_color.b = 0; spot1_color.a = 255;
                    drawSpotMarker(renderer->renderer, renderer->font, 
                        result.spot1_x, result.spot1_y, result.spot1_temp,
                        spot1_color, scale, "ABOVE");
                    
                    SDL_Color spot2_color;
                    spot2_color.r = 0; spot2_color.g = 255; spot2_color.b = 255; spot2_color.a = 255;
                    drawSpotMarker(renderer->renderer, renderer->font,
                        result.spot2_x, result.spot2_y, result.spot2_temp,
                        spot2_color, scale, "LEFT");
                    
                    // Draw camera position marker (red) showing where camera points
                    drawCameraMarker(renderer->renderer, renderer->font,
                        result.spot1_x, result.spot1_y,
                        result.spot2_x, result.spot2_y, scale);
                    
                    // Terminal output (rate limited)
                    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                    if (now - last_print >= print_interval) {
                        std::cout << "\rDist: " << std::fixed << std::setprecision(2) << result.distance_cm 
                                  << " cm | Yaw: " << std::setw(7) << result.camera_yaw_deg 
                                  << " deg | Pitch: " << std::setw(7) << result.camera_pitch_deg << " deg   " << std::flush;
                        last_print = now;
                    }
                }

                // Draw sidebar with temperatures
                drawSidebar(renderer->renderer, renderer->font, renderer->font_large,
                    result.detected, result.distance_cm, result.camera_yaw_deg, result.camera_pitch_deg,
                    result.spot1_temp, result.spot2_temp, renderer->isolation_mode,
                    ww, wh);

                SDL_RenderPresent(renderer->renderer);

                seekcamera_frame_unlock(renderer->frame);
                renderer->is_dirty.store(false);
                renderer->frame = NULL;
            }
        }

        // Handle events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_i) {
                // Toggle isolation mode
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                     it != g_renderers.end(); ++it) {
                    if (it->second != NULL) {
                        it->second->isolation_mode = !it->second->isolation_mode;
                        std::cout << "\nIsolation mode: " << (it->second->isolation_mode ? "ON" : "OFF") << std::endl;
                    }
                }
            }
        }
    }

    std::cout << std::endl << "Shutting down..." << std::endl;

    seekcamera_manager_destroy(&manager);

    for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
         it != g_renderers.end(); ++it) {
        cleanup_renderer(it->second);
        delete it->second;
    }
    g_renderers.clear();

    TTF_Quit();
    SDL_Quit();

    return 0;
}