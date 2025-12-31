/**
 * @file pcb_shape_template.h
 * @brief PCB shape template for contour matching
 */

#pragma once

#include <vector>
#include <cmath>

namespace thermal {

struct ShapePoint {
    float x, y;
};

// Aspect ratio of main body (height / width) - typically around 0.3-0.5
static const float PCB_MAIN_BODY_ASPECT = 0.35f;

// Anchor spike dimensions relative to main body width
static const float ANCHOR_WIDTH_RATIO = 0.08f;   // Each anchor is ~8% of body width
static const float ANCHOR_HEIGHT_RATIO = 0.15f;  // Anchors extend ~15% of body width below
static const float ANCHOR_OFFSET_RATIO = 0.35f;  // Anchors are ~35% from center to edge

// PCB perimeter template - normalized coordinates
// Origin (0,0) = cold circle center
// Width normalized to 1.0
// Points in counter-clockwise order
static const std::vector<ShapePoint> PCB_TEMPLATE = {
    // Right side of main body (top to bottom)
    { 0.50f, -0.175f},  // Top right corner
    { 0.50f,  0.175f},  // Bottom right corner (before anchor)
    
    // Right anchor spike
    { 0.43f,  0.175f},  // Inner edge of right anchor start
    { 0.43f,  0.325f},  // Right anchor bottom inner
    { 0.35f,  0.325f},  // Right anchor bottom outer  
    { 0.35f,  0.175f},  // Right anchor top outer
    
    // Bottom of main body (right to left)
    { 0.35f,  0.175f},  // After right anchor
    {-0.35f,  0.175f},  // Before left anchor
    
    // Left anchor spike
    {-0.35f,  0.175f},  // Left anchor top outer
    {-0.35f,  0.325f},  // Left anchor bottom outer
    {-0.43f,  0.325f},  // Left anchor bottom inner
    {-0.43f,  0.175f},  // Left anchor top inner
    
    // Left side of main body (bottom to top)
    {-0.50f,  0.175f},  // Bottom left corner
    {-0.50f, -0.175f},  // Top left corner
    
    // Top of main body (left to right, back to start)
    {-0.50f, -0.175f},  // Top left
    { 0.50f, -0.175f},  // Top right (close loop)
};

// Cold circle parameters (relative to width)
static const float COLD_CIRCLE_RADIUS_RATIO = 0.04f;  // ~4% of width

/**
 * Generate a denser perimeter for matching
 * Interpolates between template points
 */
inline std::vector<ShapePoint> getDensePerimeter(int points_per_segment = 5) {
    std::vector<ShapePoint> dense;
    
    for (size_t i = 0; i < PCB_TEMPLATE.size(); i++) {
        const auto& p1 = PCB_TEMPLATE[i];
        const auto& p2 = PCB_TEMPLATE[(i + 1) % PCB_TEMPLATE.size()];
        
        for (int j = 0; j < points_per_segment; j++) {
            float t = static_cast<float>(j) / points_per_segment;
            dense.push_back({
                p1.x + t * (p2.x - p1.x),
                p1.y + t * (p2.y - p1.y)
            });
        }
    }
    
    return dense;
}

/**
 * Scale and translate template to match detected blob
 */
inline std::vector<ShapePoint> transformTemplate(
    float center_x, float center_y, 
    float width, float height_scale = 1.0f) {
    
    std::vector<ShapePoint> transformed;
    
    for (const auto& p : PCB_TEMPLATE) {
        transformed.push_back({
            center_x + p.x * width,
            center_y + p.y * width * height_scale
        });
    }
    
    return transformed;
}

} // namespace thermal