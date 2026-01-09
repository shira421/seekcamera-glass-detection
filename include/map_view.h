/**
 * @file map_view.h
 * @brief 2D top-down map visualization showing sensor and mapped glass surface
 * 
 * The sensor is fixed at the bottom center. As the camera rotates and measures
 * distance, points are plotted to build up a map of the glass surface shape.
 * Distance is measured perpendicular to the camera (along its viewing direction).
 */

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include "thermal_distance_sensor.h"
#include <vector>
#include <deque>

#if defined(__linux__) || defined(__APPLE__)
#   include <SDL2/SDL.h>
#   include <SDL2/SDL_ttf.h>
#elif defined(_WIN32)
#   include <SDL.h>
#   include <SDL_ttf.h>
#endif

namespace thermal {

/**
 * @brief A single mapped point on the glass surface
 */
struct MappedPoint {
    float x_mm;         // X position relative to sensor (+ = right)
    float y_mm;         // Y position relative to sensor (+ = forward/up in view)
    float distance_mm;  // Original distance measurement
    float yaw_deg;      // Yaw angle when measured
    Uint32 timestamp;   // When this point was recorded
};

/**
 * @brief A detected glass edge point
 */
struct EdgePoint {
    float x_mm;         // X position relative to sensor
    float y_mm;         // Y position relative to sensor  
    float yaw_deg;      // Yaw when detected
    bool is_left_edge;  // True if left edge, false if right edge
    Uint32 timestamp;   // When this point was recorded
};

/**
 * @brief 2D Map visualization window with glass surface mapping
 */
class MapView {
public:
    MapView();
    ~MapView();
    
    /**
     * @brief Initialize the map view window
     * @param width Window width in pixels
     * @param height Window height in pixels
     * @return true if successful
     */
    bool init(int width = 500, int height = 500);
    
    /**
     * @brief Cleanup resources
     */
    void cleanup();
    
    /**
     * @brief Update the display with new sensor data
     * @param result Detection result containing distance and yaw
     */
    void update(const GridResult& result);
    
    /**
     * @brief Render the map view
     */
    void render();
    
    /**
     * @brief Clear all mapped points
     */
    void clearMap();
    
    /**
     * @brief Check if window is still open
     */
    bool isOpen() const { return window_ != nullptr; }
    
    /**
     * @brief Handle window events
     * @return false if window should close
     */
    bool handleEvents();
    
    /**
     * @brief Get the SDL window (for event handling)
     */
    SDL_Window* getWindow() const { return window_; }
    
    /**
     * @brief Get window ID for event routing
     */
    Uint32 getWindowID() const;

private:
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    TTF_Font* font_;
    TTF_Font* font_small_;
    
    int width_;
    int height_;
    
    // Current sensor state
    float distance_mm_;
    float yaw_deg_;
    float pitch_deg_;
    bool valid_;
    
    // Sensor position in view (fixed at bottom center)
    int sensor_x_;
    int sensor_y_;
    
    // View settings
    float max_distance_mm_;  // Maximum distance to show
    float pixels_per_mm_;    // Scale factor
    
    // Mapped glass surface points
    std::deque<MappedPoint> mapped_points_;
    static const size_t MAX_POINTS = 500;      // Maximum points to keep
    static const Uint32 POINT_LIFETIME_MS = 10000;  // Points fade after 10 seconds
    
    // Detected glass edge points
    std::deque<EdgePoint> edge_points_;
    static const size_t MAX_EDGE_POINTS = 200;
    float last_edge_yaw_;
    
    // Minimum angle change to record a new point (prevents duplicate points)
    float last_recorded_yaw_;
    static constexpr float MIN_YAW_CHANGE = 0.5f;  // degrees
    
    // Helper methods
    void drawSensor();
    void drawCurrentRay();
    void drawMappedPoints();
    void drawEdgePoints();
    void drawGrid();
    void drawLabels();
    
    // Convert world coordinates to screen coordinates
    int worldToScreenX(float x_mm);
    int worldToScreenY(float y_mm);
};

} // namespace thermal

#endif // MAP_VIEW_H