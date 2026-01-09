/**
 * @file map_view.cpp
 * @brief Implementation of 2D map visualization with glass surface mapping
 */

#include "map_view.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace thermal {

static const float PI_F = 3.14159265f;

MapView::MapView()
    : window_(nullptr)
    , renderer_(nullptr)
    , font_(nullptr)
    , font_small_(nullptr)
    , width_(500)
    , height_(500)
    , distance_mm_(0)
    , yaw_deg_(0)
    , pitch_deg_(0)
    , valid_(false)
    , sensor_x_(250)
    , sensor_y_(450)
    , max_distance_mm_(300.0f)  // Show up to 300mm (30cm)
    , pixels_per_mm_(1.2f)
    , last_edge_yaw_(999.0f)
    , last_recorded_yaw_(999.0f)  // Force first point to be recorded
{
}

MapView::~MapView() {
    cleanup();
}

bool MapView::init(int width, int height) {
    width_ = width;
    height_ = height;
    
    // Sensor at bottom center, with margin
    sensor_x_ = width_ / 2;
    sensor_y_ = height_ - 50;
    
    // Calculate scale to fit max distance in view
    // Leave room for sensor at bottom and labels at top
    int usable_height = sensor_y_ - 60;
    pixels_per_mm_ = static_cast<float>(usable_height) / max_distance_mm_;
    
    window_ = SDL_CreateWindow(
        "Surface Map",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width_, height_,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    
    if (!window_) {
        printf("MapView: Failed to create window: %s\n", SDL_GetError());
        return false;
    }
    
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        printf("MapView: Failed to create renderer: %s\n", SDL_GetError());
        cleanup();
        return false;
    }
    
    // Enable alpha blending
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    
    // Load fonts
#if defined(_WIN32)
    font_ = TTF_OpenFont("C:\\Windows\\Fonts\\consola.ttf", 16);
    font_small_ = TTF_OpenFont("C:\\Windows\\Fonts\\consola.ttf", 11);
#else
    font_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16);
    font_small_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11);
#endif
    
    if (!font_) {
        printf("MapView: Warning - could not load font\n");
    }
    
    return true;
}

void MapView::cleanup() {
    if (font_small_) {
        TTF_CloseFont(font_small_);
        font_small_ = nullptr;
    }
    if (font_) {
        TTF_CloseFont(font_);
        font_ = nullptr;
    }
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}

Uint32 MapView::getWindowID() const {
    if (window_) {
        return SDL_GetWindowID(window_);
    }
    return 0;
}

void MapView::clearMap() {
    mapped_points_.clear();
    edge_points_.clear();
    last_recorded_yaw_ = 999.0f;
    last_edge_yaw_ = 999.0f;
}

int MapView::worldToScreenX(float x_mm) {
    // X: positive = right, so add to sensor_x
    return sensor_x_ + static_cast<int>(x_mm * pixels_per_mm_);
}

int MapView::worldToScreenY(float y_mm) {
    // Y: positive = forward (up in top-down view), so subtract from sensor_y
    return sensor_y_ - static_cast<int>(y_mm * pixels_per_mm_);
}

void MapView::update(const GridResult& result) {
    valid_ = result.valid;
    
    if (valid_) {
        distance_mm_ = result.distance_mm;
        yaw_deg_ = result.yaw_deg;
        pitch_deg_ = result.pitch_deg;
        
        // Record a new point if yaw has changed enough
        float yaw_change = std::abs(yaw_deg_ - last_recorded_yaw_);
        if (yaw_change >= MIN_YAW_CHANGE || last_recorded_yaw_ > 900.0f) {
            
            // Calculate point position in world coordinates
            // The camera rotates around a pivot point 35mm behind it
            // So the camera position changes as it rotates
            static const float PIVOT_RADIUS_MM = 35.0f;
            
            // Negate yaw: positive yaw = glass on right of image = camera pointing left
            float yaw_rad = -yaw_deg_ * PI_F / 180.0f;
            
            // Camera position relative to pivot (pivot is at origin)
            // When yaw=0, camera is at (0, PIVOT_RADIUS) facing forward
            // When camera rotates, it moves in an arc
            float cam_x = PIVOT_RADIUS_MM * std::sin(yaw_rad);
            float cam_y = PIVOT_RADIUS_MM * std::cos(yaw_rad);
            
            // The measured point is distance_mm along the camera's viewing direction
            // Camera viewing direction is the same angle as its position from pivot
            float point_x = cam_x + distance_mm_ * std::sin(yaw_rad);
            float point_y = cam_y + distance_mm_ * std::cos(yaw_rad);
            
            MappedPoint point;
            point.distance_mm = distance_mm_;
            point.yaw_deg = yaw_deg_;
            point.x_mm = point_x;
            point.y_mm = point_y;
            point.timestamp = SDL_GetTicks();
            
            mapped_points_.push_back(point);
            last_recorded_yaw_ = yaw_deg_;
            
            // Remove old points if we have too many
            while (mapped_points_.size() > MAX_POINTS) {
                mapped_points_.pop_front();
            }
        }
    }
    
    // Check for edge detection (even when valid is false)
    if (result.edge_left || result.edge_right) {
        float edge_yaw_change = std::abs(result.edge_yaw_deg - last_edge_yaw_);
        if (edge_yaw_change >= MIN_YAW_CHANGE || last_edge_yaw_ > 900.0f) {
            
            static const float PIVOT_RADIUS_MM = 35.0f;
            float yaw_rad = -result.edge_yaw_deg * PI_F / 180.0f;
            
            // Camera position
            float cam_x = PIVOT_RADIUS_MM * std::sin(yaw_rad);
            float cam_y = PIVOT_RADIUS_MM * std::cos(yaw_rad);
            
            // Edge point is at the last known distance
            float point_x = cam_x + result.edge_distance_mm * std::sin(yaw_rad);
            float point_y = cam_y + result.edge_distance_mm * std::cos(yaw_rad);
            
            EdgePoint edge;
            edge.x_mm = point_x;
            edge.y_mm = point_y;
            edge.yaw_deg = result.edge_yaw_deg;
            edge.is_left_edge = result.edge_left;
            edge.timestamp = SDL_GetTicks();
            
            edge_points_.push_back(edge);
            last_edge_yaw_ = result.edge_yaw_deg;
            
            // Remove old edge points if we have too many
            while (edge_points_.size() > MAX_EDGE_POINTS) {
                edge_points_.pop_front();
            }
        }
    }
    
    // Remove expired points
    Uint32 now = SDL_GetTicks();
    while (!mapped_points_.empty() && 
           (now - mapped_points_.front().timestamp) > POINT_LIFETIME_MS) {
        mapped_points_.pop_front();
    }
    
    // Remove expired edge points
    while (!edge_points_.empty() && 
           (now - edge_points_.front().timestamp) > POINT_LIFETIME_MS) {
        edge_points_.pop_front();
    }
}

bool MapView::handleEvents() {
    return window_ != nullptr;
}

void MapView::render() {
    if (!renderer_) return;
    
    // Clear with dark background
    SDL_SetRenderDrawColor(renderer_, 20, 25, 30, 255);
    SDL_RenderClear(renderer_);
    
    // Draw components (order matters for layering)
    drawGrid();
    drawMappedPoints();
    drawEdgePoints();
    if (valid_) {
        drawCurrentRay();
    }
    drawSensor();
    drawLabels();
    
    SDL_RenderPresent(renderer_);
}

void MapView::drawGrid() {
    // Draw concentric distance arcs
    SDL_SetRenderDrawColor(renderer_, 40, 45, 50, 255);
    
    // Draw arcs at 50mm intervals
    for (float d = 50; d <= max_distance_mm_; d += 50) {
        int radius = static_cast<int>(d * pixels_per_mm_);
        
        // Draw arc using line segments
        int num_segments = 60;
        for (int i = 0; i < num_segments; i++) {
            // Arc from -90 to +90 degrees (forward hemisphere)
            float angle1 = (-90.0f + i * 180.0f / num_segments) * PI_F / 180.0f;
            float angle2 = (-90.0f + (i + 1) * 180.0f / num_segments) * PI_F / 180.0f;
            
            int x1 = sensor_x_ + static_cast<int>(radius * std::cos(angle1));
            int y1 = sensor_y_ + static_cast<int>(radius * std::sin(angle1));
            int x2 = sensor_x_ + static_cast<int>(radius * std::cos(angle2));
            int y2 = sensor_y_ + static_cast<int>(radius * std::sin(angle2));
            
            SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
        }
    }
    
    // Draw radial lines at angle intervals
    SDL_SetRenderDrawColor(renderer_, 35, 40, 45, 255);
    for (float angle = -60; angle <= 60; angle += 15) {
        float rad = angle * PI_F / 180.0f;
        int end_x = sensor_x_ + static_cast<int>(max_distance_mm_ * pixels_per_mm_ * std::sin(rad));
        int end_y = sensor_y_ - static_cast<int>(max_distance_mm_ * pixels_per_mm_ * std::cos(rad));
        SDL_RenderDrawLine(renderer_, sensor_x_, sensor_y_, end_x, end_y);
    }
    
    // Draw center line (0 degrees) more prominently
    SDL_SetRenderDrawColor(renderer_, 50, 55, 60, 255);
    SDL_RenderDrawLine(renderer_, sensor_x_, sensor_y_, sensor_x_, sensor_y_ - static_cast<int>(max_distance_mm_ * pixels_per_mm_));
}

void MapView::drawMappedPoints() {
    if (mapped_points_.empty()) return;
    
    Uint32 now = SDL_GetTicks();
    
    // Draw lines connecting adjacent points (to form surface outline)
    if (mapped_points_.size() >= 2) {
        // Sort points by yaw angle for connecting
        std::vector<MappedPoint> sorted_points(mapped_points_.begin(), mapped_points_.end());
        std::sort(sorted_points.begin(), sorted_points.end(),
            [](const MappedPoint& a, const MappedPoint& b) {
                return a.yaw_deg < b.yaw_deg;
            });
        
        for (size_t i = 1; i < sorted_points.size(); i++) {
            const MappedPoint& p1 = sorted_points[i - 1];
            const MappedPoint& p2 = sorted_points[i];
            
            // Only connect if yaw angles are close (< 5 degrees apart)
            if (std::abs(p2.yaw_deg - p1.yaw_deg) < 5.0f) {
                // Calculate alpha based on age (newer = brighter)
                Uint32 age1 = now - p1.timestamp;
                Uint32 age2 = now - p2.timestamp;
                Uint32 avg_age = (age1 + age2) / 2;
                float alpha = 1.0f - static_cast<float>(avg_age) / POINT_LIFETIME_MS;
                alpha = std::max(0.2f, alpha);
                
                int x1 = worldToScreenX(p1.x_mm);
                int y1 = worldToScreenY(p1.y_mm);
                int x2 = worldToScreenX(p2.x_mm);
                int y2 = worldToScreenY(p2.y_mm);
                
                // Draw line in cyan
                SDL_SetRenderDrawColor(renderer_, 0, 200, 255, static_cast<Uint8>(alpha * 200));
                SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
            }
        }
    }
    
    // Draw individual points
    for (const auto& point : mapped_points_) {
        int x = worldToScreenX(point.x_mm);
        int y = worldToScreenY(point.y_mm);
        
        // Calculate alpha based on age
        Uint32 age = now - point.timestamp;
        float alpha = 1.0f - static_cast<float>(age) / POINT_LIFETIME_MS;
        alpha = std::max(0.2f, alpha);
        
        // Draw point as small filled circle
        SDL_SetRenderDrawColor(renderer_, 0, 255, 200, static_cast<Uint8>(alpha * 255));
        
        // Draw 3x3 point
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx*dx + dy*dy <= 1) {
                    SDL_RenderDrawPoint(renderer_, x + dx, y + dy);
                }
            }
        }
    }
}

void MapView::drawEdgePoints() {
    if (edge_points_.empty()) return;
    
    Uint32 now = SDL_GetTicks();
    
    // Draw edge points as pink/magenta dots
    for (const auto& edge : edge_points_) {
        int x = worldToScreenX(edge.x_mm);
        int y = worldToScreenY(edge.y_mm);
        
        // Calculate alpha based on age
        Uint32 age = now - edge.timestamp;
        float alpha = 1.0f - static_cast<float>(age) / POINT_LIFETIME_MS;
        alpha = std::max(0.2f, alpha);
        
        // Pink/magenta color for edge points
        SDL_SetRenderDrawColor(renderer_, 255, 100, 200, static_cast<Uint8>(alpha * 255));
        
        // Draw slightly larger point (5x5) for edge markers
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                if (dx*dx + dy*dy <= 4) {
                    SDL_RenderDrawPoint(renderer_, x + dx, y + dy);
                }
            }
        }
    }
    
    // Connect consecutive edge points with lines
    if (edge_points_.size() >= 2) {
        for (size_t i = 1; i < edge_points_.size(); i++) {
            const auto& p1 = edge_points_[i - 1];
            const auto& p2 = edge_points_[i];
            
            // Only connect points that are close in yaw (continuous edge)
            float yaw_diff = std::abs(p1.yaw_deg - p2.yaw_deg);
            if (yaw_diff < 3.0f) {
                // Calculate alpha based on newer point
                Uint32 age = now - p2.timestamp;
                float alpha = 1.0f - static_cast<float>(age) / POINT_LIFETIME_MS;
                alpha = std::max(0.2f, alpha);
                
                int x1 = worldToScreenX(p1.x_mm);
                int y1 = worldToScreenY(p1.y_mm);
                int x2 = worldToScreenX(p2.x_mm);
                int y2 = worldToScreenY(p2.y_mm);
                
                // Draw line in pink
                SDL_SetRenderDrawColor(renderer_, 255, 100, 200, static_cast<Uint8>(alpha * 150));
                SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
            }
        }
    }
}

void MapView::drawCurrentRay() {
    // Draw the current measurement ray from sensor to detected point
    // The camera rotates around a pivot point 35mm behind it
    static const float PIVOT_RADIUS_MM = 35.0f;
    
    // Negate yaw: positive yaw = glass on right of image = camera pointing left
    float yaw_rad = -yaw_deg_ * PI_F / 180.0f;
    
    // Camera position relative to pivot
    float cam_x = PIVOT_RADIUS_MM * std::sin(yaw_rad);
    float cam_y = PIVOT_RADIUS_MM * std::cos(yaw_rad);
    
    // Detected point position
    float point_x = cam_x + distance_mm_ * std::sin(yaw_rad);
    float point_y = cam_y + distance_mm_ * std::cos(yaw_rad);
    
    int cam_screen_x = worldToScreenX(cam_x);
    int cam_screen_y = worldToScreenY(cam_y);
    int end_x = worldToScreenX(point_x);
    int end_y = worldToScreenY(point_y);
    
    // Draw ray line from camera to detected point (yellow, semi-transparent)
    SDL_SetRenderDrawColor(renderer_, 255, 200, 0, 150);
    SDL_RenderDrawLine(renderer_, cam_screen_x, cam_screen_y, end_x, end_y);
    
    // Draw current detection point (bright yellow)
    SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);
    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            if (dx*dx + dy*dy <= 9) {
                SDL_RenderDrawPoint(renderer_, end_x + dx, end_y + dy);
            }
        }
    }
    
    // Draw FOV cone edges (56 degree FOV = 28 degrees each side)
    float half_fov = 28.0f * PI_F / 180.0f;
    float left_angle = yaw_rad - half_fov;
    float right_angle = yaw_rad + half_fov;
    
    // Extend to max distance or current distance, whichever is larger for visibility
    float cone_dist = std::max(distance_mm_ * 1.2f, 100.0f);
    
    int left_x = cam_screen_x + static_cast<int>(cone_dist * pixels_per_mm_ * std::sin(left_angle));
    int left_y = cam_screen_y - static_cast<int>(cone_dist * pixels_per_mm_ * std::cos(left_angle));
    int right_x = cam_screen_x + static_cast<int>(cone_dist * pixels_per_mm_ * std::sin(right_angle));
    int right_y = cam_screen_y - static_cast<int>(cone_dist * pixels_per_mm_ * std::cos(right_angle));
    
    SDL_SetRenderDrawColor(renderer_, 255, 150, 0, 80);
    SDL_RenderDrawLine(renderer_, cam_screen_x, cam_screen_y, left_x, left_y);
    SDL_RenderDrawLine(renderer_, cam_screen_x, cam_screen_y, right_x, right_y);
}

void MapView::drawSensor() {
    // The camera rotates around a pivot point 35mm behind it
    // Draw the pivot point at sensor_x_, sensor_y_ and the camera on a rotating arm
    static const float PIVOT_RADIUS_MM = 35.0f;
    
    // Negate yaw for correct visual direction
    float yaw_rad = -yaw_deg_ * PI_F / 180.0f;
    
    // Camera position relative to pivot
    float cam_x = PIVOT_RADIUS_MM * std::sin(yaw_rad);
    float cam_y = PIVOT_RADIUS_MM * std::cos(yaw_rad);
    
    int cam_screen_x = worldToScreenX(cam_x);
    int cam_screen_y = worldToScreenY(cam_y);
    
    // Draw pivot point (small gray circle)
    SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            if (dx*dx + dy*dy <= 9) {
                SDL_RenderDrawPoint(renderer_, sensor_x_ + dx, sensor_y_ + dy);
            }
        }
    }
    
    // Draw arm from pivot to camera
    SDL_SetRenderDrawColor(renderer_, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer_, sensor_x_, sensor_y_, cam_screen_x, cam_screen_y);
    
    // Draw camera as a triangle pointing in viewing direction
    // The tip should point along the distance ray direction
    int tri_size = 12;
    
    // Direction vector (same as ray direction in screen space)
    // In world coords: direction is (sin(yaw_rad), cos(yaw_rad))
    // In screen coords: X stays same, Y is flipped
    float dir_x = std::sin(yaw_rad);
    float dir_y = -std::cos(yaw_rad);  // Negative because screen Y is inverted
    
    // Perpendicular vector for triangle base
    float perp_x = -dir_y;
    float perp_y = dir_x;
    
    // Triangle vertices in screen coordinates
    int tip_x = cam_screen_x + static_cast<int>(dir_x * tri_size * 1.2f);
    int tip_y = cam_screen_y + static_cast<int>(dir_y * tri_size * 1.2f);
    
    int back_left_x = cam_screen_x - static_cast<int>(dir_x * tri_size * 0.5f) - static_cast<int>(perp_x * tri_size * 0.6f);
    int back_left_y = cam_screen_y - static_cast<int>(dir_y * tri_size * 0.5f) - static_cast<int>(perp_y * tri_size * 0.6f);
    
    int back_right_x = cam_screen_x - static_cast<int>(dir_x * tri_size * 0.5f) + static_cast<int>(perp_x * tri_size * 0.6f);
    int back_right_y = cam_screen_y - static_cast<int>(dir_y * tri_size * 0.5f) + static_cast<int>(perp_y * tri_size * 0.6f);
    
    int screen_pts[3][2] = {
        {tip_x, tip_y},
        {back_left_x, back_left_y},
        {back_right_x, back_right_y}
    };
    
    // Draw filled triangle
    SDL_SetRenderDrawColor(renderer_, 0, 255, 100, 255);
    
    // Simple triangle fill
    int min_y = std::min({screen_pts[0][1], screen_pts[1][1], screen_pts[2][1]});
    int max_y = std::max({screen_pts[0][1], screen_pts[1][1], screen_pts[2][1]});
    
    for (int y = min_y; y <= max_y; y++) {
        int min_x = width_, max_x = 0;
        
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3;
            int y1 = screen_pts[i][1], y2 = screen_pts[j][1];
            int x1 = screen_pts[i][0], x2 = screen_pts[j][0];
            
            if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
                if (y1 != y2) {
                    int x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                    min_x = std::min(min_x, x);
                    max_x = std::max(max_x, x);
                }
            }
        }
        
        if (min_x <= max_x) {
            SDL_RenderDrawLine(renderer_, min_x, y, max_x, y);
        }
    }
    
    // Draw triangle outline
    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer_, screen_pts[0][0], screen_pts[0][1], screen_pts[1][0], screen_pts[1][1]);
    SDL_RenderDrawLine(renderer_, screen_pts[1][0], screen_pts[1][1], screen_pts[2][0], screen_pts[2][1]);
    SDL_RenderDrawLine(renderer_, screen_pts[2][0], screen_pts[2][1], screen_pts[0][0], screen_pts[0][1]);
}

void MapView::drawLabels() {
    if (!font_) return;
    
    char buf[64];
    SDL_Color white = {255, 255, 255, 255};
    SDL_Color cyan = {0, 255, 255, 255};
    SDL_Color yellow = {255, 255, 0, 255};
    SDL_Color gray = {120, 120, 120, 255};
    
    // Title
    SDL_Surface* surface = TTF_RenderText_Blended(font_, "SURFACE MAP", white);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        if (texture) {
            SDL_Rect dst = {width_/2 - surface->w/2, 8, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, NULL, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
    
    // Current measurements
    int label_x = 10;
    int label_y = 35;
    
    if (valid_) {
        snprintf(buf, sizeof(buf), "Dist: %.1f mm", distance_mm_);
    } else {
        snprintf(buf, sizeof(buf), "Dist: ---");
    }
    surface = TTF_RenderText_Blended(font_, buf, cyan);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        if (texture) {
            SDL_Rect dst = {label_x, label_y, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, NULL, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
    
    if (valid_) {
        snprintf(buf, sizeof(buf), "Yaw: %+.1f deg", yaw_deg_);
    } else {
        snprintf(buf, sizeof(buf), "Yaw: ---");
    }
    surface = TTF_RenderText_Blended(font_, buf, yellow);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        if (texture) {
            SDL_Rect dst = {label_x, label_y + 20, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, NULL, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
    
    // Point count
    snprintf(buf, sizeof(buf), "Points: %zu", mapped_points_.size());
    surface = TTF_RenderText_Blended(font_small_, buf, gray);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        if (texture) {
            SDL_Rect dst = {label_x, label_y + 42, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, NULL, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
    
    // Distance markers on the arcs
    if (font_small_) {
        for (float d = 100; d <= max_distance_mm_; d += 100) {
            snprintf(buf, sizeof(buf), "%.0f", d);
            surface = TTF_RenderText_Blended(font_small_, buf, gray);
            if (surface) {
                SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
                if (texture) {
                    int y = sensor_y_ - static_cast<int>(d * pixels_per_mm_);
                    SDL_Rect dst = {sensor_x_ + 5, y - 6, surface->w, surface->h};
                    SDL_RenderCopy(renderer_, texture, NULL, &dst);
                    SDL_DestroyTexture(texture);
                }
                SDL_FreeSurface(surface);
            }
        }
    }
    
    // Instructions
    surface = TTF_RenderText_Blended(font_small_, "Rotate camera to map glass surface", gray);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        if (texture) {
            SDL_Rect dst = {width_/2 - surface->w/2, height_ - 20, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, NULL, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

} // namespace thermal