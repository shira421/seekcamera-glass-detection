/**
 * @file thermal_distance_sensor.cpp
 * @brief Implementation of core detection and ranging functions
 */

#include "thermal_distance_sensor.h"
#include <cmath>
#include <algorithm>
#include <numeric>

static const float PI_F = 3.14159265f;

// Physical grid dimensions (known constants)
static const float GRID_PHYSICAL_WIDTH_CM = 9.6f;
static const float GRID_PHYSICAL_HEIGHT_CM = 2.4f;
static const float CAMERA_HOLE_DIAMETER_CM = 0.8f;

namespace thermal {

PCBGridSensor::PCBGridSensor()
    : initialized_(false)
    , filt_dist_(0), filt_yaw_(0), filt_pitch_(0)
    , filt_center_x_(0), filt_center_y_(0)
    , filt_scale_(0)
    , smooth_circle_x_(0), smooth_circle_y_(0), circle_initialized_(false)
    , filt_x_min_(0), filt_x_max_(0), filt_y_min_(0), filt_y_max_(0)
    , prev_center_x_(0), prev_center_y_(0)
    , velocity_(0)
    , frames_since_detection_(999)
    , detection_confidence_(0)
{
    float aspect = static_cast<float>(SENSOR_HEIGHT) / static_cast<float>(SENSOR_WIDTH);
    float hfov_rad = (HFOV_DEGREES / 2.0f) * PI_F / 180.0f;
    float vfov_rad = std::atan(std::tan(hfov_rad) * aspect);
    vfov_degrees_ = vfov_rad * 2.0f * 180.0f / PI_F;
    focal_length_px_ = (SENSOR_WIDTH / 2.0f) / std::tan(hfov_rad);
    
    initializeExpectedGrid();
    
    smoothed_x_.resize(PCBGrid::TOTAL_DOTS, 0);
    smoothed_y_.resize(PCBGrid::TOTAL_DOTS, 0);
    
    dist_buffer_.reserve(MEDIAN_SIZE);
    yaw_buffer_.reserve(MEDIAN_SIZE);
    pitch_buffer_.reserve(MEDIAN_SIZE);
}

void PCBGridSensor::initializeExpectedGrid() {
    expected_dots_.clear();
    expected_dots_.reserve(PCBGrid::TOTAL_DOTS);
    
    float cam_col = 6.0f;
    float cam_row = 1.0f;
    
    // Row 0: 13 dots
    for (int col = 0; col < 13; col++) {
        GridDot dot;
        dot.row = 0; dot.col = col;
        dot.anchor_id = -1; dot.anchor_pos = -1;
        dot.expected_x_cm = (col - cam_col) * PCBGrid::H_SPACING_CM;
        dot.expected_y_cm = (0 - cam_row) * PCBGrid::V_SPACING_CM;
        dot.pixel_x = 0; dot.pixel_y = 0;
        dot.temperature = 0; dot.detected = false;
        expected_dots_.push_back(dot);
    }
    
    // Row 1: 9 dots (skip camera hole)
    for (int col = 0; col < 13; col++) {
        if (col == 0) continue;
        if (col >= 5 && col <= 7) continue;
        GridDot dot;
        dot.row = 1; dot.col = col;
        dot.anchor_id = -1; dot.anchor_pos = -1;
        dot.expected_x_cm = (col - cam_col) * PCBGrid::H_SPACING_CM;
        dot.expected_y_cm = 0;
        dot.pixel_x = 0; dot.pixel_y = 0;
        dot.temperature = 0; dot.detected = false;
        expected_dots_.push_back(dot);
    }
    
    // Row 2: 13 dots
    for (int col = 0; col < 13; col++) {
        GridDot dot;
        dot.row = 2; dot.col = col;
        dot.anchor_id = -1; dot.anchor_pos = -1;
        dot.expected_x_cm = (col - cam_col) * PCBGrid::H_SPACING_CM;
        dot.expected_y_cm = (2 - cam_row) * PCBGrid::V_SPACING_CM;
        dot.pixel_x = 0; dot.pixel_y = 0;
        dot.temperature = 0; dot.detected = false;
        expected_dots_.push_back(dot);
    }
}

struct ThermalBlob {
    float center_x, center_y;
    float min_x, max_x, min_y, max_y;
    float width, height;
    float peak_temp;
    int pixel_count;
    bool valid;
    
    float cold_center_x, cold_center_y;
    float cold_radius;
    bool cold_circle_found;
};

bool validatePCBShape(const ThermalBlob& blob, float* temps, int width, int height, 
                      float avg_temp, float max_temp) {
    if (!blob.valid) return false;
    
    if (blob.width < 15 || blob.height < 6) return false;
    
    float temp_range = max_temp - avg_temp;
    if (temp_range < 3.0f) return false;  // Need at least 3°C above ambient
    
    float hot_thresh = avg_temp + temp_range * 0.45f;
    int peak_count = 0;
    const int peak_radius = 3;
    
    int bx_min = static_cast<int>(blob.min_x) + peak_radius;
    int bx_max = static_cast<int>(blob.max_x) - peak_radius;
    int by_min = static_cast<int>(blob.min_y) + peak_radius;
    int by_max = static_cast<int>(blob.max_y) - peak_radius;
    
    // Clamp to image bounds
    bx_min = std::max(peak_radius, bx_min);
    bx_max = std::min(width - peak_radius - 1, bx_max);
    by_min = std::max(peak_radius, by_min);
    by_max = std::min(height - peak_radius - 1, by_max);
    
    if (bx_max <= bx_min || by_max <= by_min) return false;
    
    for (int y = by_min; y <= by_max; y += 2) {  // Sample every 2 pixels
        for (int x = bx_min; x <= bx_max; x += 2) {
            float t = temps[y * width + x];
            if (t < hot_thresh) continue;
            
            // Check if local maximum
            bool is_peak = true;
            for (int dy = -peak_radius; dy <= peak_radius && is_peak; dy++) {
                for (int dx = -peak_radius; dx <= peak_radius; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (temps[(y + dy) * width + (x + dx)] > t) {
                        is_peak = false;
                        break;
                    }
                }
            }
            if (is_peak) peak_count++;
        }
    }
    
    if (peak_count < 3) return false;
    
    float center_x = (blob.min_x + blob.max_x) / 2.0f;
    float center_y = (blob.min_y + blob.max_y) / 2.0f;
    int cx = static_cast<int>(center_x);
    int cy = static_cast<int>(center_y);
    
    // Find minimum temperature in the central region
    float min_center_temp = max_temp;
    int sample_r = static_cast<int>(std::min(blob.width, blob.height) * 0.25f);
    sample_r = std::max(3, std::min(sample_r, 20));
    
    for (int dy = -sample_r; dy <= sample_r; dy++) {
        for (int dx = -sample_r; dx <= sample_r; dx++) {
            int px = cx + dx, py = cy + dy;
            if (px >= 0 && px < width && py >= 0 && py < height) {
                min_center_temp = std::min(min_center_temp, temps[py * width + px]);
            }
        }
    }
    
    if (min_center_temp > max_temp - 2.0f) return false;
    
    return true;
}

// Find the main thermal blob using flood-fill from peak temperature
ThermalBlob findMainBlob(float* temps, int width, int height, float min_temp, float max_temp) {
    ThermalBlob blob = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, false};
    
    // Find peak temperature location
    int peak_x = 0, peak_y = 0;
    float peak_val = min_temp;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] > peak_val) {
                peak_val = temps[y * width + x];
                peak_x = x;
                peak_y = y;
            }
        }
    }
    
    blob.peak_temp = peak_val;
    
    // Threshold: midpoint between average and peak
    float avg_temp = 0;
    for (int i = 0; i < width * height; i++) avg_temp += temps[i];
    avg_temp /= (width * height);
    
    float threshold = avg_temp + (peak_val - avg_temp) * 0.25f;
    
    // Find all connected hot pixels using simple region growing
    float sum_x = 0, sum_y = 0, sum_w = 0;
    float blob_min_x = static_cast<float>(width), blob_max_x = 0;
    float blob_min_y = static_cast<float>(height), blob_max_y = 0;
    int count = 0;
    
    std::vector<bool> visited(width * height, false);
    std::vector<std::pair<int,int>> queue;
    queue.push_back({peak_x, peak_y});
    visited[peak_y * width + peak_x] = true;
    
    while (!queue.empty()) {
        std::pair<int,int> p = queue.back();
        queue.pop_back();
        int x = p.first;
        int y = p.second;
        
        float t = temps[y * width + x];
        if (t < threshold) continue;
        
        float w = (t - threshold) * (t - threshold);
        sum_x += x * w;
        sum_y += y * w;
        sum_w += w;
        count++;
        
        blob_min_x = std::min(blob_min_x, static_cast<float>(x));
        blob_max_x = std::max(blob_max_x, static_cast<float>(x));
        blob_min_y = std::min(blob_min_y, static_cast<float>(y));
        blob_max_y = std::max(blob_max_y, static_cast<float>(y));
        
        // Add neighbors
        const int dx[] = {-1, 1, 0, 0};
        const int dy[] = {0, 0, -1, 1};
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (!visited[ny * width + nx]) {
                    visited[ny * width + nx] = true;
                    if (temps[ny * width + nx] >= threshold) {
                        queue.push_back({nx, ny});
                    }
                }
            }
        }
    }
    
    if (sum_w > 0 && count > 10) {
        blob.center_x = sum_x / sum_w;
        blob.center_y = sum_y / sum_w;
        blob.min_x = blob_min_x;
        blob.max_x = blob_max_x;
        blob.min_y = blob_min_y;
        blob.max_y = blob_max_y;
        blob.width = blob_max_x - blob_min_x;
        blob.height = blob_max_y - blob_min_y;
        blob.pixel_count = count;
        blob.valid = true;
    }
    
    return blob;
}

float findMainGridBottom(float* temps, int width, int height, const ThermalBlob& blob, float avg_temp, float max_temp) {
    if (!blob.valid) return blob.max_y;
    
    int y_min = static_cast<int>(blob.min_y);
    int y_max = static_cast<int>(blob.max_y);
    int x_min = static_cast<int>(blob.min_x);
    int x_max = static_cast<int>(blob.max_x);
    
    int blob_height = y_max - y_min;
    if (blob_height < 15) return blob.max_y;  // Too small
    
    // Calculate heat density per row
    float hot_threshold = avg_temp + (max_temp - avg_temp) * 0.3f;
    std::vector<int> row_hot_pixels(blob_height + 1, 0);
    
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            if (temps[y * width + x] >= hot_threshold) {
                row_hot_pixels[y - y_min]++;
            }
        }
    }
    
    const int window = 3;
    std::vector<float> smoothed(blob_height + 1, 0);
    for (int i = 0; i <= blob_height; i++) {
        float sum = 0;
        int cnt = 0;
        for (int j = std::max(0, i - window); j <= std::min(blob_height, i + window); j++) {
            sum += row_hot_pixels[j];
            cnt++;
        }
        smoothed[i] = sum / cnt;
    }
    
    // Find significant drops in heat density from top
    // A gap between main grid and anchors will show as a local minimum
    float max_heat = 0;
    for (int i = 0; i < static_cast<int>(blob_height * 0.6f); i++) {
        max_heat = std::max(max_heat, smoothed[i]);
    }
    
    if (max_heat < 3) return blob.max_y;  // No significant heat pattern
    
    int start_scan = static_cast<int>(blob_height * 0.5f);
    float min_in_gap = max_heat;
    int gap_row = -1;
    
    for (int i = start_scan; i <= blob_height; i++) {
        if (smoothed[i] < max_heat * 0.25f) {
            if (smoothed[i] < min_in_gap) {
                min_in_gap = smoothed[i];
                gap_row = i;
            }
        }
    }
    
    if (gap_row > 0) {
        bool found_heat_after_gap = false;
        for (int i = gap_row; i <= blob_height; i++) {
            if (smoothed[i] > max_heat * 0.4f) {
                found_heat_after_gap = true;
                break;
            }
        }
        
        if (found_heat_after_gap) {
            return static_cast<float>(y_min + gap_row - 1);
        }
    }
    
    return static_cast<float>(y_min + static_cast<int>(blob_height * 0.8f));
}

void findCameraCircle(float* temps, int width, int height, ThermalBlob& blob, 
                      float scale, float avg_temp, float max_temp) {
    blob.cold_circle_found = false;
    blob.cold_radius = 0;
    
    if (!blob.valid) return;
    
    // Expected radius based on physical size
    float expected_radius = (CAMERA_HOLE_DIAMETER_CM / 2.0f) * scale;
    expected_radius = std::max(3.0f, std::min(expected_radius, 25.0f));
    
    // Bounding box center
    float bbox_center_x = (blob.min_x + blob.max_x) / 2.0f;
    float bbox_center_y = (blob.min_y + blob.max_y) / 2.0f;
    
    // Temperature threshold
    float temp_range = max_temp - avg_temp;
    float hot_thresh = avg_temp + temp_range * 0.45f;
    
    float margin_x = blob.width * 0.25f;
    float margin_y = blob.height * 0.25f;
    
    int sx_min = static_cast<int>(blob.min_x + margin_x);
    int sx_max = static_cast<int>(blob.max_x - margin_x);
    int sy_min = static_cast<int>(blob.min_y + margin_y);
    int sy_max = static_cast<int>(blob.max_y - margin_y);
    
    // Ensure valid search region
    sx_min = std::max(0, sx_min);
    sx_max = std::min(width - 1, sx_max);
    sy_min = std::max(0, sy_min);
    sy_max = std::min(height - 1, sy_max);
    
    if (sx_max <= sx_min || sy_max <= sy_min) {
        blob.cold_center_x = bbox_center_x;
        blob.cold_center_y = bbox_center_y;
        blob.cold_radius = expected_radius;
        blob.cold_circle_found = true;
        return;
    }
    
    float coldest_temp = temps[sy_min * width + sx_min];
    int coldest_x = static_cast<int>(bbox_center_x);
    int coldest_y = static_cast<int>(bbox_center_y);
    
    for (int y = sy_min; y <= sy_max; y++) {
        for (int x = sx_min; x <= sx_max; x++) {
            float t = temps[y * width + x];
            if (t < coldest_temp) {
                coldest_temp = t;
                coldest_x = x;
                coldest_y = y;
            }
        }
    }
    
    // Expand concentrically from coldest point to find circle edge
    int cx = coldest_x;
    int cy = coldest_y;
    
    int max_search_radius = static_cast<int>(expected_radius * 2.5f);
    max_search_radius = std::min(max_search_radius, 25);
    
    int found_radius = 0;
    
    for (int r = 1; r <= max_search_radius; r++) {
        int num_samples = std::max(8, r * 4);
        int hot_count = 0;
        int valid_count = 0;
        
        for (int i = 0; i < num_samples; i++) {
            float angle = 2.0f * PI_F * i / num_samples;
            int px = cx + static_cast<int>(r * std::cos(angle));
            int py = cy + static_cast<int>(r * std::sin(angle));
            
            if (px < 0 || px >= width || py < 0 || py >= height) continue;
            
            valid_count++;
            if (temps[py * width + px] >= hot_thresh) {
                hot_count++;
            }
        }
        
        if (valid_count > 0 && hot_count > valid_count / 2) {
            found_radius = r;
            break;
        }
    }
    
    float final_radius = (found_radius > 2) ? static_cast<float>(found_radius) : expected_radius;
    
    // Refine center using weighted centroid of cold region
    int search_r = static_cast<int>(final_radius * 1.2f);
    float cold_thresh = coldest_temp + (hot_thresh - coldest_temp) * 0.4f;
    
    float sum_x = 0, sum_y = 0, sum_w = 0;
    
    for (int dy = -search_r; dy <= search_r; dy++) {
        for (int dx = -search_r; dx <= search_r; dx++) {
            if (dx*dx + dy*dy > search_r*search_r) continue;
            int px = cx + dx, py = cy + dy;
            if (px < 0 || px >= width || py < 0 || py >= height) continue;
            
            float t = temps[py * width + px];
            if (t <= cold_thresh) {
                float w = cold_thresh - t + 0.1f;
                sum_x += px * w;
                sum_y += py * w;
                sum_w += w;
            }
        }
    }
    
    if (sum_w > 0) {
        blob.cold_center_x = sum_x / sum_w;
        blob.cold_center_y = sum_y / sum_w;
    } else {
        blob.cold_center_x = static_cast<float>(coldest_x);
        blob.cold_center_y = static_cast<float>(coldest_y);
    }
    
    float final_margin_x = blob.width * 0.20f;
    float final_margin_y = blob.height * 0.20f;
    
    blob.cold_center_x = std::max(blob.cold_center_x, blob.min_x + final_margin_x);
    blob.cold_center_x = std::min(blob.cold_center_x, blob.max_x - final_margin_x);
    blob.cold_center_y = std::max(blob.cold_center_y, blob.min_y + final_margin_y);
    blob.cold_center_y = std::min(blob.cold_center_y, blob.max_y - final_margin_y);
    
    blob.cold_radius = final_radius;
    blob.cold_circle_found = true;
}

std::vector<PCBGridSensor::HotSpot> PCBGridSensor::findHotSpots(
    float* temps, int w, int h, float threshold, float scale) {
    
    std::vector<HotSpot> spots;
    
    int radius = static_cast<int>(scale * 0.25f);
    radius = std::max(2, std::min(radius, 8));
    
    float min_separation = scale * 0.5f;  // Half the spacing to allow some overlap
    min_separation = std::max(3.0f, min_separation);
    
    for (int y = radius; y < h - radius; y++) {
        for (int x = radius; x < w - radius; x++) {
            float center_temp = temps[y * w + x];
            if (center_temp < threshold) continue;
            
            // Local maximum check with scale-dependent radius
            bool is_max = true;
            for (int dy = -radius; dy <= radius && is_max; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (temps[(y + dy) * w + (x + dx)] >= center_temp) {
                        is_max = false;
                        break;
                    }
                }
            }
            
            if (!is_max) continue;
            
            // Check minimum separation from already found spots
            bool too_close = false;
            for (const auto& existing : spots) {
                float dx = existing.x - x;
                float dy = existing.y - y;
                if (dx*dx + dy*dy < min_separation * min_separation) {
                    if (center_temp <= existing.temperature) {
                        too_close = true;
                        break;
                    }
                }
            }
            if (too_close) continue;
            
            HotSpot spot;
            spot.x = static_cast<float>(x);
            spot.y = static_cast<float>(y);
            spot.temperature = center_temp;
            spots.push_back(spot);
        }
    }
    
    std::sort(spots.begin(), spots.end(),
        [](const HotSpot& a, const HotSpot& b) { return a.temperature > b.temperature; });
    
    if (spots.size() > 60) spots.resize(60);
    
    return spots;
}

float PCBGridSensor::smooth(float raw, float& filtered, float alpha) {
    filtered = alpha * raw + (1.0f - alpha) * filtered;
    return filtered;
}

float PCBGridSensor::median(std::vector<float>& buffer, float new_val) {
    buffer.push_back(new_val);
    if (buffer.size() > MEDIAN_SIZE) buffer.erase(buffer.begin());
    std::vector<float> sorted = buffer;
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}

static void adaptiveSmoothCenter(float raw, float& filtered, float velocity) {
    float alpha;
    if (velocity < 0.5f) {
        alpha = 0.08f;
    } else if (velocity > 3.0f) {
        alpha = 0.6f;   // Very responsive when moving
    } else {
        alpha = 0.08f + (velocity - 0.5f) / 2.5f * 0.52f;
    }
    filtered = alpha * raw + (1.0f - alpha) * filtered;
}

static void adaptiveSmoothScale(float raw, float& filtered, float velocity) {
    float alpha;
    if (velocity < 1.0f) {
        alpha = 0.1f;
    } else if (velocity > 3.0f) {
        alpha = 0.4f;
    } else {
        alpha = 0.1f + (velocity - 1.0f) / 2.0f * 0.3f;
    }
    filtered = alpha * raw + (1.0f - alpha) * filtered;
}

void PCBGridSensor::matchDotsToGrid(const std::vector<HotSpot>& spots,
                                     float center_x, float center_y, float scale,
                                     std::vector<GridDot>& dots) {
    dots = expected_dots_;
    for (auto& d : dots) { 
        d.detected = false; 
        d.temperature = 0;
        d.pixel_x = center_x + d.expected_x_cm * scale;
        d.pixel_y = center_y + d.expected_y_cm * scale;
    }
    
    if (scale < 2.0f) return;
    
    std::vector<bool> used(spots.size(), false);
    float search_radius = std::max(6.0f, scale * 0.4f);
    
    for (size_t i = 0; i < dots.size(); i++) {
        if (dots[i].anchor_id >= 0) continue;
        
        float exp_x = dots[i].pixel_x;
        float exp_y = dots[i].pixel_y;
        
        float best_dist = search_radius;
        int best_idx = -1;
        
        for (size_t j = 0; j < spots.size(); j++) {
            if (used[j]) continue;
            float dx = spots[j].x - exp_x;
            float dy = spots[j].y - exp_y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(j);
            }
        }
        
        if (best_idx >= 0) {
            used[best_idx] = true;
            dots[i].detected = true;
            dots[i].temperature = spots[best_idx].temperature;
            dots[i].pixel_x = spots[best_idx].x;
            dots[i].pixel_y = spots[best_idx].y;
        }
    }
}

GridResult PCBGridSensor::detect(float* temps, int width, int height) {
    GridResult result = {};
    result.valid = false;
    
    float min_temp = temps[0], max_temp = temps[0];
    for (int i = 0; i < width * height; i++) {
        min_temp = std::min(min_temp, temps[i]);
        max_temp = std::max(max_temp, temps[i]);
    }
    result.max_temp = max_temp;
    
    // Calculate average temperature early
    float avg_temp = 0;
    for (int i = 0; i < width * height; i++) avg_temp += temps[i];
    avg_temp /= (width * height);
    
    bool detection_succeeded = false;
    ThermalBlob blob;
    
    if (max_temp - min_temp >= 3.0f) {
        blob = findMainBlob(temps, width, height, min_temp, max_temp);
        
        if (blob.valid && blob.width >= 10 && blob.height >= 5) {
            if (validatePCBShape(blob, temps, width, height, avg_temp, max_temp)) {
                detection_succeeded = true;
            }
        }
    }
    
    if (detection_succeeded) {
        frames_since_detection_ = 0;
        detection_confidence_ = 1.0f;
    } else {
        // No detection this frame
        frames_since_detection_++;
        
        // Decay confidence gradually
        detection_confidence_ *= 0.85f;  // Lose 15% confidence per frame
        
        if (initialized_ && frames_since_detection_ <= MAX_HOLDOVER_FRAMES && detection_confidence_ > 0.1f) {
            result.valid = true;
            result.confidence = detection_confidence_;  // Show we're in holdover
            result.distance_cm = filt_dist_;
            result.yaw_deg = filt_yaw_;
            result.pitch_deg = filt_pitch_;
            result.pixels_per_cm = filt_scale_;
            result.grid_center_x = filt_center_x_;
            result.grid_center_y = filt_center_y_;
            result.circle_found = true;
            result.circle_radius = (CAMERA_HOLE_DIAMETER_CM / 2.0f) * filt_scale_;
            result.main_grid_x_min = filt_x_min_;
            result.main_grid_x_max = filt_x_max_;
            result.main_grid_y_min = filt_y_min_;
            result.main_grid_y_max = filt_y_max_;
            result.grid_width_px = filt_x_max_ - filt_x_min_;
            result.grid_height_px = filt_y_max_ - filt_y_min_;
            result.hot_region_temp = max_temp;
            return result;
        }
        return result;
    }
    
    result.hot_region_temp = blob.peak_temp;

    // Find where the main grid ends (before anchors)
    float main_grid_bottom = findMainGridBottom(temps, width, height, blob, avg_temp, max_temp);
    
    if (main_grid_bottom < blob.max_y - 5) {
        blob.max_y = main_grid_bottom;
        blob.height = blob.max_y - blob.min_y;
        blob.center_y = (blob.min_y + blob.max_y) / 2.0f;
    }

    float raw_scale;
    float visible_width_cm;
    float blob_aspect = blob.width / std::max(1.0f, blob.height);
    
    if (blob_aspect > 2.5f) {
        visible_width_cm = GRID_PHYSICAL_WIDTH_CM * 0.85f;  // ~8.2cm
    } else if (blob_aspect > 1.5f) {
        visible_width_cm = GRID_PHYSICAL_WIDTH_CM * 0.7f;   // ~6.7cm
    } else {
        visible_width_cm = blob.height / (GRID_PHYSICAL_HEIGHT_CM / GRID_PHYSICAL_WIDTH_CM);
        visible_width_cm = std::min(visible_width_cm, GRID_PHYSICAL_WIDTH_CM);
    }
    
    raw_scale = blob.width / visible_width_cm;
    raw_scale = std::max(2.0f, std::min(raw_scale, 40.0f));
    
    float scale_for_circle = (initialized_ && filt_scale_ > 2.0f) ? filt_scale_ : raw_scale;
    
    findCameraCircle(temps, width, height, blob, scale_for_circle, avg_temp, max_temp);
    
    float raw_center_x, raw_center_y;
    
    if (blob.cold_circle_found) {
        raw_center_x = blob.cold_center_x;
        raw_center_y = blob.cold_center_y;
    } else {
        // Fallback to blob centroid
        raw_center_x = blob.center_x;
        raw_center_y = blob.center_y;
    }
    
    if (!initialized_) {
        filt_scale_ = raw_scale;
        filt_center_x_ = raw_center_x;
        filt_center_y_ = raw_center_y;
        prev_center_x_ = raw_center_x;
        prev_center_y_ = raw_center_y;
        velocity_ = 0;
        filt_dist_ = focal_length_px_ / raw_scale / 2.0f;
        filt_yaw_ = 0;
        filt_pitch_ = 0;
        // Initialize filtered bounds
        filt_x_min_ = blob.min_x;
        filt_x_max_ = blob.max_x;
        filt_y_min_ = blob.min_y;
        filt_y_max_ = blob.max_y;
        initialized_ = true;
    } else {
        // Calculate instantaneous velocity
        float dx = raw_center_x - prev_center_x_;
        float dy = raw_center_y - prev_center_y_;
        float instant_vel = std::sqrt(dx*dx + dy*dy);
        
        if (instant_vel > velocity_) {
            velocity_ = 0.4f * instant_vel + 0.6f * velocity_;
        } else {
            velocity_ = 0.02f * instant_vel + 0.98f * velocity_;  // Extremely slow decay
        }
        
        prev_center_x_ = raw_center_x;
        prev_center_y_ = raw_center_y;
        
        float scale_ratio = raw_scale / filt_scale_;
        if (scale_ratio < 0.75f || scale_ratio > 1.33f) {
            raw_scale = filt_scale_;  // Keep old value
        }
        
        raw_center_x = std::max(raw_center_x, blob.min_x + 5.0f);
        raw_center_x = std::min(raw_center_x, blob.max_x - 5.0f);
        raw_center_y = std::max(raw_center_y, blob.min_y + 3.0f);
        raw_center_y = std::min(raw_center_y, blob.max_y - 3.0f);
        
        float center_jump = std::sqrt(
            (raw_center_x - filt_center_x_) * (raw_center_x - filt_center_x_) +
            (raw_center_y - filt_center_y_) * (raw_center_y - filt_center_y_));
        
        float max_jump = (velocity_ < 1.0f) ? 5.0f : (velocity_ < 3.0f) ? 15.0f : 30.0f;
        
        if (center_jump > max_jump) {
            float blend = max_jump / center_jump;
            raw_center_x = filt_center_x_ + (raw_center_x - filt_center_x_) * blend;
            raw_center_y = filt_center_y_ + (raw_center_y - filt_center_y_) * blend;
        }
        
        // Apply adaptive smoothing - different for scale vs center
        adaptiveSmoothScale(raw_scale, filt_scale_, velocity_);
        
        float circle_alpha;
        if (velocity_ < 0.5f) {
            circle_alpha = 0.05f;
        } else if (velocity_ > 3.0f) {
            circle_alpha = 0.4f;
        } else {
            circle_alpha = 0.05f + (velocity_ - 0.5f) / 2.5f * 0.35f;
        }
        filt_center_x_ = circle_alpha * raw_center_x + (1.0f - circle_alpha) * filt_center_x_;
        filt_center_y_ = circle_alpha * raw_center_y + (1.0f - circle_alpha) * filt_center_y_;
        
        adaptiveSmoothScale(blob.min_x, filt_x_min_, velocity_);
        adaptiveSmoothScale(blob.max_x, filt_x_max_, velocity_);
        adaptiveSmoothScale(blob.min_y, filt_y_min_, velocity_);
        adaptiveSmoothScale(blob.max_y, filt_y_max_, velocity_);
    }
    static const float DISTANCE_CALIBRATION = 1.57273f;  // Adjust based on testing
    
    float optical_distance = focal_length_px_ / filt_scale_;
    float raw_distance = (optical_distance / 2.0f) * DISTANCE_CALIBRATION;
    
    // Smooth distance with median filter for extra stability
    float med_distance = median(dist_buffer_, raw_distance);
    adaptiveSmoothScale(med_distance, filt_dist_, velocity_);
    
    // Yaw: horizontal offset from center
    float offset_x = filt_center_x_ - (width / 2.0f);
    float raw_yaw = (offset_x / (width / 2.0f)) * (HFOV_DEGREES / 2.0f);
    float med_yaw = median(yaw_buffer_, raw_yaw);
    adaptiveSmoothCenter(med_yaw, filt_yaw_, velocity_);
    
    // Pitch: vertical offset from center
    float offset_y = filt_center_y_ - (height / 2.0f);
    float raw_pitch = (offset_y / (height / 2.0f)) * (vfov_degrees_ / 2.0f);
    float med_pitch = median(pitch_buffer_, raw_pitch);
    adaptiveSmoothCenter(med_pitch, filt_pitch_, velocity_);
    
    result.confidence = 1.0f;  // Fresh detection = full confidence
    result.distance_cm = filt_dist_;
    result.yaw_deg = filt_yaw_;
    result.pitch_deg = filt_pitch_;
    result.pixels_per_cm = filt_scale_;
    result.hot_region_temp = blob.peak_temp;
    
    result.grid_center_x = filt_center_x_;
    result.grid_center_y = filt_center_y_;
    
    result.circle_found = blob.cold_circle_found;
    result.circle_radius = (CAMERA_HOLE_DIAMETER_CM / 2.0f) * filt_scale_;
    
    result.main_grid_x_min = filt_x_min_;
    result.main_grid_x_max = filt_x_max_;
    result.main_grid_y_min = filt_y_min_;
    result.main_grid_y_max = filt_y_max_;
    result.grid_width_px = filt_x_max_ - filt_x_min_;
    result.grid_height_px = filt_y_max_ - filt_y_min_;
    
    float spot_threshold = avg_temp + (max_temp - avg_temp) * 0.15f;
    std::vector<HotSpot> spots = findHotSpots(temps, width, height, spot_threshold, filt_scale_);
    
    std::vector<HotSpot> blob_spots;
    float margin = 5.0f;
    for (const auto& s : spots) {
        if (s.x >= blob.min_x - margin && s.x <= blob.max_x + margin &&
            s.y >= blob.min_y - margin && s.y <= blob.max_y + margin) {
            blob_spots.push_back(s);
        }
    }
    
    matchDotsToGrid(blob_spots, filt_center_x_, filt_center_y_, filt_scale_, result.dots);
    
    int main_detected = 0;
    for (const auto& d : result.dots) {
        if (d.anchor_id < 0 && d.detected) main_detected++;
    }
    result.main_dots_detected = main_detected;
    result.total_dots_detected = main_detected;
    result.num_main_rows = 3;  // Assume 3 rows if blob valid
    
    result.valid = true;
    
    return result;
}

SDL_Color mapTemperature(float temp, float min_temp, float max_temp, bool isolation) {
    float range = max_temp - min_temp;
    if (range < 1.0f) range = 1.0f;
    float norm = (temp - min_temp) / range;
    norm = std::max(0.0f, std::min(1.0f, norm));
    
    float enhanced = norm * norm * (3.0f - 2.0f * norm);
    
    if (isolation) {
        if (enhanced < 0.7f) return {0, 0, static_cast<Uint8>(enhanced * 100), 255};
        float hot = (enhanced - 0.7f) / 0.3f;
        return {static_cast<Uint8>(255 * hot), static_cast<Uint8>(100 * hot), 0, 255};
    }
    
    SDL_Color c;
    if (enhanced < 0.25f) {
        float t = enhanced / 0.25f;
        c.r = static_cast<Uint8>(t * 80);
        c.g = 0;
        c.b = static_cast<Uint8>(50 + t * 130);
    } else if (enhanced < 0.5f) {
        float t = (enhanced - 0.25f) / 0.25f;
        c.r = static_cast<Uint8>(80 + t * 175);
        c.g = static_cast<Uint8>(t * 50);
        c.b = static_cast<Uint8>(180 - t * 100);
    } else if (enhanced < 0.75f) {
        float t = (enhanced - 0.5f) / 0.25f;
        c.r = 255;
        c.g = static_cast<Uint8>(50 + t * 150);
        c.b = static_cast<Uint8>(80 - t * 80);
    } else {
        float t = (enhanced - 0.75f) / 0.25f;
        c.r = 255;
        c.g = static_cast<Uint8>(200 + t * 55);
        c.b = static_cast<Uint8>(t * 150);
    }
    c.a = 255;
    return c;
}

SDL_Color mapTemperatureSharp(float temp, float sharpened, float min_temp, float max_temp, bool isolation) {
    return mapTemperature(sharpened, min_temp, max_temp, isolation);
}

void sharpenFrame(float* input, float* output, int width, int height, float strength) {
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            float center = input[y * width + x];
            float neighbors = (
                input[(y-1) * width + x] +
                input[(y+1) * width + x] +
                input[y * width + (x-1)] +
                input[y * width + (x+1)]
            ) / 4.0f;
            output[y * width + x] = center + strength * (center - neighbors);
        }
    }
    for (int x = 0; x < width; x++) {
        output[x] = input[x];
        output[(height-1) * width + x] = input[(height-1) * width + x];
    }
    for (int y = 0; y < height; y++) {
        output[y * width] = input[y * width];
        output[y * width + (width-1)] = input[y * width + (width-1)];
    }
}

} // namespace thermal