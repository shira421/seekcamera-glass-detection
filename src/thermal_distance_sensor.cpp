/**
 * @file thermal_distance_sensor.cpp
 * @brief Implementation of thermal triangulation distance sensor
 */

#include "thermal_distance_sensor.h"
#include <queue>
#include <map>
#include <cmath>

namespace thermal {

TriangulationDistanceSensor::TriangulationDistanceSensor()
    : filtered_dist_(0), filtered_yaw_(0), filtered_pitch_(0)
    , filtered_spot1_x_(0), filtered_spot1_y_(0)
    , filtered_spot2_x_(0), filtered_spot2_y_(0)
    , filter_initialized_(false)
    , prev_dist_(0), prev_yaw_(0), prev_pitch_(0)
    , prev_spot1_x_(0), prev_spot1_y_(0)
    , prev_spot2_x_(0), prev_spot2_y_(0)
{
    // Calculate vertical FOV from horizontal FOV using pinhole camera model
    float aspect_ratio = SENSOR_HEIGHT / SENSOR_WIDTH;
    float hfov_half_rad = (HFOV_DEGREES / 2.0f) * static_cast<float>(M_PI) / 180.0f;
    float vfov_half_rad = std::atan(std::tan(hfov_half_rad) * aspect_ratio);
    vfov_degrees_ = (vfov_half_rad * 180.0f / static_cast<float>(M_PI)) * 2.0f;
    
    // Calculate focal length in pixels
    focal_length_pixels_ = (SENSOR_WIDTH / 2.0f) / std::tan(hfov_half_rad);
    
    // Reserve space for median buffers
    dist_buffer_.reserve(MEDIAN_WINDOW);
    yaw_buffer_.reserve(MEDIAN_WINDOW);
    pitch_buffer_.reserve(MEDIAN_WINDOW);
}

float TriangulationDistanceSensor::getMedian(std::vector<float>& buffer, float new_val) {
    buffer.push_back(new_val);
    if (buffer.size() > MEDIAN_WINDOW) {
        buffer.erase(buffer.begin());
    }
    
    // Copy and sort to find median
    std::vector<float> sorted = buffer;
    std::sort(sorted.begin(), sorted.end());
    
    size_t mid = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
        return (sorted[mid - 1] + sorted[mid]) / 2.0f;
    } else {
        return sorted[mid];
    }
}

TriangulationDistanceSensor::HotSpot 
TriangulationDistanceSensor::findHotSpotCentroid(
    float* temps, int width, int height,
    int center_x, int center_y, float max_temp)
{
    HotSpot spot;
    spot.valid = false;
    spot.x = 0;
    spot.y = 0;
    spot.temp = 0;
    
    // 7x7 Gaussian kernel (sigma ≈ 1.5)
    // Provides spatial weighting to reduce edge noise
    static const float gauss[7][7] = {
        {0.01f, 0.02f, 0.03f, 0.04f, 0.03f, 0.02f, 0.01f},
        {0.02f, 0.04f, 0.06f, 0.07f, 0.06f, 0.04f, 0.02f},
        {0.03f, 0.06f, 0.09f, 0.12f, 0.09f, 0.06f, 0.03f},
        {0.04f, 0.07f, 0.12f, 0.15f, 0.12f, 0.07f, 0.04f},
        {0.03f, 0.06f, 0.09f, 0.12f, 0.09f, 0.06f, 0.03f},
        {0.02f, 0.04f, 0.06f, 0.07f, 0.06f, 0.04f, 0.02f},
        {0.01f, 0.02f, 0.03f, 0.04f, 0.03f, 0.02f, 0.01f}
    };
    
    // Include pixels within 3°C of peak
    float threshold = max_temp - 3.0f;
    float sum_weight = 0.0f, sum_x = 0.0f, sum_y = 0.0f;
    
    // Iterate over 7x7 window
    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            int px = center_x + dx;
            int py = center_y + dy;
            
            if (px >= 0 && px < width && py >= 0 && py < height) {
                float temp = temps[py * width + px];
                if (temp > threshold) {
                    // Combined weight: temperature³ × Gaussian spatial weight
                    // Cubic weighting strongly emphasizes the peak
                    float temp_weight = (temp - threshold);
                    temp_weight = temp_weight * temp_weight * temp_weight;
                    float spatial_weight = gauss[dy + 3][dx + 3];
                    float weight = temp_weight * spatial_weight;
                    
                    sum_weight += weight;
                    sum_x += weight * static_cast<float>(px);
                    sum_y += weight * static_cast<float>(py);
                }
            }
        }
    }
    
    if (sum_weight > 0) {
        spot.x = sum_x / sum_weight;
        spot.y = sum_y / sum_weight;
        spot.temp = max_temp;
        spot.valid = true;
    } else {
        // Fallback to integer position if weighting fails
        spot.x = static_cast<float>(center_x);
        spot.y = static_cast<float>(center_y);
        spot.temp = max_temp;
        spot.valid = true;
    }
    
    return spot;
}

DistanceResult TriangulationDistanceSensor::calculate(
    float* temps, int width, int height)
{
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
    
    std::vector<float> all_temps;
    all_temps.reserve(width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            all_temps.push_back(temps[y * width + x]);
        }
    }
    std::sort(all_temps.begin(), all_temps.end());
    
    float background_temp = all_temps[all_temps.size() / 2];  // Median
    float temp_80th = all_temps[(all_temps.size() * 80) / 100];
    
    // Detection threshold: at least 0.1°C above background, or 80th percentile
    constexpr float MIN_TEMP_ABOVE_BG = 0.1f;
    constexpr int NEIGHBORHOOD = 2;  // 5x5 window for local max detection
    float detection_threshold = std::min(background_temp + MIN_TEMP_ABOVE_BG, temp_80th);
    
    struct LocalMax {
        int x, y;
        float temp;
    };
    
    std::vector<LocalMax> local_maxima;
    
    for (int y = NEIGHBORHOOD; y < height - NEIGHBORHOOD; y++) {
        for (int x = NEIGHBORHOOD; x < width - NEIGHBORHOOD; x++) {
            float center_temp = temps[y * width + x];
            if (center_temp < detection_threshold) continue;
            
            // Check if this pixel is the maximum in its neighborhood
            bool is_local_max = true;
            for (int dy = -NEIGHBORHOOD; dy <= NEIGHBORHOOD && is_local_max; dy++) {
                for (int dx = -NEIGHBORHOOD; dx <= NEIGHBORHOOD; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (temps[(y + dy) * width + (x + dx)] > center_temp) {
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
    
    // Sort by temperature (descending) using simple bubble sort for C++11 compat
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
        return result;  // Need at least 2 spots
    }
    
    constexpr float MIN_SEPARATION = 10.0f;  // Minimum pixel separation
    LocalMax spot1 = local_maxima[0];  // Hottest spot
    LocalMax spot2;
    bool found_second = false;
    
    for (size_t i = 1; i < local_maxima.size(); i++) {
        float dx = static_cast<float>(local_maxima[i].x - spot1.x);
        float dy = static_cast<float>(local_maxima[i].y - spot1.y);
        float sep = std::sqrt(dx * dx + dy * dy);
        
        if (sep >= MIN_SEPARATION) {
            spot2 = local_maxima[i];
            found_second = true;
            break;
        }
    }
    
    if (!found_second) return result;
    
    HotSpot hotspot1 = findHotSpotCentroid(temps, width, height, 
                                           spot1.x, spot1.y, spot1.temp);
    HotSpot hotspot2 = findHotSpotCentroid(temps, width, height, 
                                           spot2.x, spot2.y, spot2.temp);
    
    if (!hotspot1.valid || !hotspot2.valid) return result;
    
    HotSpot spot_above, spot_left;
    float dx = hotspot2.x - hotspot1.x;
    float dy = hotspot2.y - hotspot1.y;
    
    if (std::fabs(dy) > std::fabs(dx)) {
        // Vertical separation dominates - use Y position
        if (hotspot1.y <= hotspot2.y) {
            spot_above = hotspot1;
            spot_left = hotspot2;
        } else {
            spot_above = hotspot2;
            spot_left = hotspot1;
        }
    } else {
        // Horizontal separation dominates - use X position
        if (hotspot1.x <= hotspot2.x) {
            spot_left = hotspot1;
            spot_above = hotspot2;
        } else {
            spot_left = hotspot2;
            spot_above = hotspot1;
        }
    }
    
    constexpr float center_x = 160.0f;
    constexpr float center_y = 120.0f;
    
    // Spot offsets from image center (in pixels)
    float spot_above_offset_x = spot_above.x - center_x;
    float spot_left_offset_y = spot_left.y - center_y;
    
    // Convert to degrees using FOV scaling
    // HFOV = 56° across 320px → 28° per 160px from center
    // VFOV ≈ 44° across 240px → 22° per 120px from center
    float spot_yaw_deg = (spot_above_offset_x / 160.0f) * 28.0f;
    float spot_pitch_deg = (spot_left_offset_y / 120.0f) * 22.0f;
    
    // Camera direction is OPPOSITE to spot position
    // Spot moves right → camera points left (negative yaw)
    // Spot moves down → camera points up (positive pitch)
    float camera_yaw_raw = -spot_yaw_deg;
    float camera_pitch_raw = spot_pitch_deg;
    
    float dx_pixels = spot_above.x - spot_left.x;
    float dy_pixels = spot_above.y - spot_left.y;
    float separation_pixels = std::sqrt(dx_pixels * dx_pixels + dy_pixels * dy_pixels);
    
    // Distance formula: d = K / separation
    float d_separation = K_CALIBRATION / separation_pixels;
    
    // Account for reflection (light travels to surface AND back)
    float raw_dist = d_separation / 2.0f;
    
    // Raw spot positions
    float raw_spot1_x = spot_above.x;
    float raw_spot1_y = spot_above.y;
    float raw_spot2_x = spot_left.x;
    float raw_spot2_y = spot_left.y;
    
    float final_dist, final_yaw, final_pitch;
    float final_spot1_x, final_spot1_y, final_spot2_x, final_spot2_y;
    
    if (!filter_initialized_) {
        // First frame - initialize all filter state
        filtered_dist_ = raw_dist;
        filtered_yaw_ = camera_yaw_raw;
        filtered_pitch_ = camera_pitch_raw;
        filtered_spot1_x_ = raw_spot1_x;
        filtered_spot1_y_ = raw_spot1_y;
        filtered_spot2_x_ = raw_spot2_x;
        filtered_spot2_y_ = raw_spot2_y;
        
        prev_dist_ = raw_dist;
        prev_yaw_ = camera_yaw_raw;
        prev_pitch_ = camera_pitch_raw;
        prev_spot1_x_ = raw_spot1_x;
        prev_spot1_y_ = raw_spot1_y;
        prev_spot2_x_ = raw_spot2_x;
        prev_spot2_y_ = raw_spot2_y;
        
        filter_initialized_ = true;
        
        final_dist = raw_dist;
        final_yaw = camera_yaw_raw;
        final_pitch = camera_pitch_raw;
        final_spot1_x = raw_spot1_x;
        final_spot1_y = raw_spot1_y;
        final_spot2_x = raw_spot2_x;
        final_spot2_y = raw_spot2_y;
    } else {
        // Calculate speed of change for adaptive alpha
        float dist_speed = std::fabs(raw_dist - prev_dist_);
        float yaw_speed = std::fabs(camera_yaw_raw - prev_yaw_);
        float pitch_speed = std::fabs(camera_pitch_raw - prev_pitch_);
        float spot1_speed = std::sqrt(
            std::pow(raw_spot1_x - prev_spot1_x_, 2.0f) + 
            std::pow(raw_spot1_y - prev_spot1_y_, 2.0f));
        float spot2_speed = std::sqrt(
            std::pow(raw_spot2_x - prev_spot2_x_, 2.0f) + 
            std::pow(raw_spot2_y - prev_spot2_y_, 2.0f));
        
        // Update previous values BEFORE filtering (prevents drift)
        prev_dist_ = raw_dist;
        prev_yaw_ = camera_yaw_raw;
        prev_pitch_ = camera_pitch_raw;
        prev_spot1_x_ = raw_spot1_x;
        prev_spot1_y_ = raw_spot1_y;
        prev_spot2_x_ = raw_spot2_x;
        prev_spot2_y_ = raw_spot2_y;
        
        // Adaptive alpha: squared speed for aggressive noise rejection
        // Base alpha = 0.01 (smooth when still), max = 0.5 (responsive when moving)
        float dist_alpha = std::min(0.01f + dist_speed * dist_speed * 2.0f, 0.5f);
        float angle_alpha = std::min(
            0.01f + std::max(yaw_speed, pitch_speed) * 
                    std::max(yaw_speed, pitch_speed) * 1.0f, 0.5f);
        float spot_alpha = std::min(
            0.01f + std::max(spot1_speed, spot2_speed) * 
                    std::max(spot1_speed, spot2_speed) * 0.5f, 0.5f);
        
        // Low-pass filter toward raw values
        filtered_dist_ = dist_alpha * raw_dist + (1.0f - dist_alpha) * filtered_dist_;
        filtered_yaw_ = angle_alpha * camera_yaw_raw + (1.0f - angle_alpha) * filtered_yaw_;
        filtered_pitch_ = angle_alpha * camera_pitch_raw + (1.0f - angle_alpha) * filtered_pitch_;
        filtered_spot1_x_ = spot_alpha * raw_spot1_x + (1.0f - spot_alpha) * filtered_spot1_x_;
        filtered_spot1_y_ = spot_alpha * raw_spot1_y + (1.0f - spot_alpha) * filtered_spot1_y_;
        filtered_spot2_x_ = spot_alpha * raw_spot2_x + (1.0f - spot_alpha) * filtered_spot2_x_;
        filtered_spot2_y_ = spot_alpha * raw_spot2_y + (1.0f - spot_alpha) * filtered_spot2_y_;
        
        // Bias correction: slowly pull toward long-term median
        // Prevents systematic drift while maintaining smoothing
        float raw_avg_dist = getMedian(dist_buffer_, raw_dist);
        float bias = filtered_dist_ - raw_avg_dist;
        filtered_dist_ -= bias * 0.01f;
        
        float raw_avg_yaw = getMedian(yaw_buffer_, camera_yaw_raw);
        float yaw_bias = filtered_yaw_ - raw_avg_yaw;
        filtered_yaw_ -= yaw_bias * 0.01f;
        
        float raw_avg_pitch = getMedian(pitch_buffer_, camera_pitch_raw);
        float pitch_bias = filtered_pitch_ - raw_avg_pitch;
        filtered_pitch_ -= pitch_bias * 0.01f;
        
        final_dist = filtered_dist_;
        final_yaw = filtered_yaw_;
        final_pitch = filtered_pitch_;
        final_spot1_x = filtered_spot1_x_;
        final_spot1_y = filtered_spot1_y_;
        final_spot2_x = filtered_spot2_x_;
        final_spot2_y = filtered_spot2_y_;
    }
    
    result.detected = true;
    result.distance_cm = final_dist;
    result.camera_yaw_deg = final_yaw;
    result.camera_pitch_deg = final_pitch;
    result.spot1_x = static_cast<int>(std::round(final_spot1_x));
    result.spot1_y = static_cast<int>(std::round(final_spot1_y));
    result.spot1_temp = spot_above.temp;
    result.spot2_x = static_cast<int>(std::round(final_spot2_x));
    result.spot2_y = static_cast<int>(std::round(final_spot2_y));
    result.spot2_temp = spot_left.temp;
    
    return result;
}

SDL_Color mapTemperature(float temp, float min_temp, float max_temp, 
                         bool isolation_mode) {
    SDL_Color color;
    
    float range = max_temp - min_temp;
    if (range < 0.01f) {
        color.r = 20; color.g = 20; color.b = 20; color.a = 255;
        return color;
    }
    
    float normalized;
    
    if (isolation_mode) {
        // Only show top 15% of temperature range
        float threshold = max_temp - (range * ISOLATION_THRESHOLD);
        
        if (temp < threshold) {
            color.r = 0; color.g = 0; color.b = 0; color.a = 255;
            return color;
        }
        
        normalized = (temp - threshold) / (range * ISOLATION_THRESHOLD);
    } else {
        // Full relative range
        normalized = (temp - min_temp) / range;
    }
    
    // Apply power curve to expand lower values
    normalized = std::pow(normalized, 0.15f);
    
    // Map to color gradient: Blue → Cyan → Green → Yellow → Red
    if (normalized < 0.25f) {
        float t = normalized * 4.0f;
        color.r = 0;
        color.g = static_cast<Uint8>(t * 255);
        color.b = 255;
    } else if (normalized < 0.5f) {
        float t = (normalized - 0.25f) * 4.0f;
        color.r = 0;
        color.g = 255;
        color.b = static_cast<Uint8>((1.0f - t) * 255);
    } else if (normalized < 0.75f) {
        float t = (normalized - 0.5f) * 4.0f;
        color.r = static_cast<Uint8>(t * 255);
        color.g = 255;
        color.b = 0;
    } else {
        float t = (normalized - 0.75f) * 4.0f;
        color.r = 255;
        color.g = static_cast<Uint8>((1.0f - t) * 255);
        color.b = 0;
    }
    
    color.a = 255;
    return color;
}

std::vector<std::vector<int>> findConnectedComponents(
    std::vector<std::vector<bool>>& binary_map, int width, int height)
{
    std::vector<std::vector<int>> labels(height, std::vector<int>(width, 0));
    int current_label = 1;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (binary_map[y][x] && labels[y][x] == 0) {
                // BFS flood fill
                std::queue<std::pair<int, int>> queue;
                queue.push(std::make_pair(x, y));
                labels[y][x] = current_label;

                while (!queue.empty()) {
                    std::pair<int, int> current = queue.front();
                    queue.pop();
                    int cx = current.first;
                    int cy = current.second;

                    // 4-connected neighbors
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

SignatureScore analyzeSignaturePattern(float* temps, int width, int height,
                                       const ThermalObject& obj) {
    SignatureScore score;
    score.gradient_quality = 0;
    score.compactness = 0;
    score.peak_centrality = 0;
    score.aspect_ratio_score = 0;
    score.circularity = 0;
    score.overall_score = 0;
    
    // Reject extreme aspect ratios
    float aspect_ratio = static_cast<float>(obj.width) / static_cast<float>(obj.height);
    if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) return score;
    
    // Find peak temperature location
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
    
    // Aspect ratio score (prefer square-ish)
    score.aspect_ratio_score = std::max(0.0f, 1.0f - std::fabs(aspect_ratio - 1.0f) * 2.0f);
    
    // Peak centrality score
    float center_x = obj.x + obj.width / 2.0f;
    float center_y = obj.y + obj.height / 2.0f;
    float peak_offset = std::sqrt(
        std::pow(static_cast<float>(peak_x) - center_x, 2.0f) + 
        std::pow(static_cast<float>(peak_y) - center_y, 2.0f));
    float max_offset = std::sqrt(
        std::pow(obj.width / 2.0f, 2.0f) + 
        std::pow(obj.height / 2.0f, 2.0f));
    score.peak_centrality = max_offset > 0 ? (1.0f - (peak_offset / max_offset)) : 1.0f;
    
    if (score.peak_centrality < 0.5f) return score;
    
    // Gradient quality - check temperature falloff in 8 directions
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
            float decline_ratio = static_cast<float>(consecutive_declines) / total_samples;
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
    score.circularity = std::min(
        (4.0f * 3.14159f * area) / static_cast<float>(perimeter * perimeter), 1.0f);
    
    if (area < 9) score.compactness = 0.0f;
    else if (area > 400) score.compactness = 0.3f;
    else score.compactness = std::max(0.3f, 1.0f - std::fabs(area - 100.0f) / 100.0f);
    
    // Calculate overall weighted score
    score.overall_score = score.gradient_quality * 0.40f + 
                          score.peak_centrality * 0.25f +
                          score.aspect_ratio_score * 0.15f + 
                          score.circularity * 0.10f +
                          score.compactness * 0.10f;
    
    return score;
}

std::vector<ThermalObject> detectThermalObjects(float* temps, int width, 
                                                 int height, int& next_id) {
    std::vector<ThermalObject> objects;
    
    // Find temperature range
    float min_temp = 1000.0f, max_temp = -1000.0f;
    for (int i = 0; i < width * height; i++) {
        min_temp = std::min(min_temp, temps[i]);
        max_temp = std::max(max_temp, temps[i]);
    }
    
    float range = max_temp - min_temp;
    if (range < 0.01f) return objects;
    
    // Threshold at top 15%
    float threshold = max_temp - (range * ISOLATION_THRESHOLD);
    
    // Create binary map
    std::vector<std::vector<bool>> hot_pixels(height, std::vector<bool>(width, false));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] >= threshold) {
                hot_pixels[y][x] = true;
            }
        }
    }
    
    // Find connected components
    std::vector<std::vector<int>> labels = findConnectedComponents(hot_pixels, width, height);
    
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
    
    // Score and filter objects
    std::vector<std::pair<ThermalObject, SignatureScore>> scored;
    
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
    
    // Return top 2 objects with color coding
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

} // namespace thermal