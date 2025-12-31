/**
 * @file rendering.cpp
 * @brief Implementation of rendering functions for grid visualization
 */

#include "rendering.h"
#include <cstdio>
#include <cmath>

namespace thermal {

void renderText(SDL_Renderer* renderer, TTF_Font* font, const char* text,
                int x, int y, SDL_Color color) {
    if (!font || !text || !renderer) return;
    
    SDL_Surface* surface = TTF_RenderText_Blended(font, text, color);
    if (!surface) return;
    
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture) {
        SDL_Rect dst;
        dst.x = x;
        dst.y = y;
        dst.w = surface->w;
        dst.h = surface->h;
        SDL_RenderCopy(renderer, texture, NULL, &dst);
        SDL_DestroyTexture(texture);
    }
    SDL_FreeSurface(surface);
}

void drawCrosshair(SDL_Renderer* renderer, int width, int height) {
    int cx = width / 2;
    int cy = height / 2;
    int size = 15;
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 200);
    
    // Horizontal line
    SDL_RenderDrawLine(renderer, cx - size, cy, cx + size, cy);
    SDL_RenderDrawLine(renderer, cx - size, cy + 1, cx + size, cy + 1);
    
    // Vertical line
    SDL_RenderDrawLine(renderer, cx, cy - size, cx, cy + size);
    SDL_RenderDrawLine(renderer, cx + 1, cy - size, cx + 1, cy + size);
}

void drawGridDots(SDL_Renderer* renderer, const GridResult& result, int scale) {
    if (result.dots.empty()) return;
    
    for (size_t i = 0; i < result.dots.size(); i++) {
        const GridDot& dot = result.dots[i];
        
        // Skip anchors and undetected dots
        if (dot.anchor_id >= 0) continue;
        if (!dot.detected) continue;
        
        int x = static_cast<int>(dot.pixel_x * scale);
        int y = static_cast<int>(dot.pixel_y * scale);
        
        // Detected: solid green box
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_Rect box;
        box.x = x - 2;
        box.y = y - 2;
        box.w = 5;
        box.h = 5;
        SDL_RenderFillRect(renderer, &box);
    }
}

// Helper to draw a circle
static void drawCircle(SDL_Renderer* renderer, int cx, int cy, int radius) {
    const int segments = 32;
    for (int i = 0; i < segments; i++) {
        float angle1 = 2.0f * 3.14159f * i / segments;
        float angle2 = 2.0f * 3.14159f * (i + 1) / segments;
        int x1 = cx + static_cast<int>(radius * std::cos(angle1));
        int y1 = cy + static_cast<int>(radius * std::sin(angle1));
        int x2 = cx + static_cast<int>(radius * std::cos(angle2));
        int y2 = cy + static_cast<int>(radius * std::sin(angle2));
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }
}

void drawGridBounds(SDL_Renderer* renderer, const GridResult& result, int scale) {
    if (!result.valid) return;
    
    // Fade alpha based on confidence (255 at full, ~50 at low confidence)
    Uint8 alpha = static_cast<Uint8>(50 + 205 * result.confidence);
    
    // Bounding box from actual blob bounds
    int bx1 = static_cast<int>(result.main_grid_x_min * scale);
    int by1 = static_cast<int>(result.main_grid_y_min * scale);
    int bx2 = static_cast<int>(result.main_grid_x_max * scale);
    int by2 = static_cast<int>(result.main_grid_y_max * scale);
    
    // Sanity check
    if (bx2 <= bx1 || by2 <= by1) return;
    if ((bx2 - bx1) < 10 || (by2 - by1) < 5) return;
    
    // Yellow box around detected thermal blob (fades with confidence)
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, alpha);
    SDL_Rect main_box;
    main_box.x = bx1;
    main_box.y = by1;
    main_box.w = bx2 - bx1;
    main_box.h = by2 - by1;
    SDL_RenderDrawRect(renderer, &main_box);
    
    // Grid center
    int cx = static_cast<int>(result.grid_center_x * scale);
    int cy = static_cast<int>(result.grid_center_y * scale);
    
    // Draw detected circle (cyan when found, dim when not)
    if (result.circle_found && result.circle_radius > 2.0f) {
        SDL_SetRenderDrawColor(renderer, 0, 255, 255, alpha);  // Cyan with fade
    } else {
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, static_cast<Uint8>(alpha * 0.6f));
    }
    int circle_r = static_cast<int>(result.circle_radius * scale);
    if (circle_r > 2) {
        drawCircle(renderer, cx, cy, circle_r);
    }
    
    // Small red crosshair at center (fades with confidence)
    SDL_SetRenderDrawColor(renderer, 255, 50, 50, alpha);
    SDL_RenderDrawLine(renderer, cx - 4, cy, cx + 4, cy);
    SDL_RenderDrawLine(renderer, cx, cy - 4, cx, cy + 4);
}

void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_large,
                 const GridResult& result, bool isolation_mode,
                 int window_width, int window_height) {
    
    int sidebar_x = window_width - SIDEBAR_WIDTH;
    
    // Background
    SDL_SetRenderDrawColor(renderer, 25, 25, 25, 255);
    SDL_Rect bg;
    bg.x = sidebar_x;
    bg.y = 0;
    bg.w = SIDEBAR_WIDTH;
    bg.h = window_height;
    SDL_RenderFillRect(renderer, &bg);
    
    // Colors
    SDL_Color white = {255, 255, 255, 255};
    SDL_Color green = {0, 255, 100, 255};
    SDL_Color yellow = {255, 200, 0, 255};
    SDL_Color red = {255, 100, 100, 255};
    SDL_Color gray = {120, 120, 120, 255};
    SDL_Color cyan = {0, 255, 255, 255};
    
    int y = 15;
    char buf[64];
    
    if (result.valid) {
        // Distance (large, green - fades with confidence)
        SDL_Color dist_color = {
            static_cast<Uint8>(green.r * result.confidence),
            static_cast<Uint8>(green.g * result.confidence + 100 * (1 - result.confidence)),
            static_cast<Uint8>(green.b * result.confidence),
            255
        };
        snprintf(buf, sizeof(buf), "%.2f cm", result.distance_cm);
        renderText(renderer, font_large, buf, sidebar_x + 15, y, dist_color);
        y += 50;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Detection stats
        renderText(renderer, font, "DETECTION", sidebar_x + 15, y, white);
        y += 22;
        
        snprintf(buf, sizeof(buf), "Main: %d / 35", result.main_dots_detected);
        SDL_Color main_color = (result.main_dots_detected >= 25) ? green : 
                               (result.main_dots_detected >= 15) ? yellow : 
                               (result.main_dots_detected >= 8) ? cyan : red;
        renderText(renderer, font, buf, sidebar_x + 20, y, main_color);
        y += 18;
        
        snprintf(buf, sizeof(buf), "Rows: %d", result.num_main_rows);
        SDL_Color rows_color = (result.num_main_rows >= 3) ? green : 
                               (result.num_main_rows >= 2) ? yellow : cyan;
        renderText(renderer, font, buf, sidebar_x + 20, y, rows_color);
        y += 18;
        
        snprintf(buf, sizeof(buf), "Scale: %.1f px/cm", result.pixels_per_cm);
        renderText(renderer, font, buf, sidebar_x + 20, y, gray);
        y += 18;
        
        snprintf(buf, sizeof(buf), "Max temp: %.1f C", result.max_temp);
        renderText(renderer, font, buf, sidebar_x + 20, y, cyan);
        y += 25;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Camera orientation
        renderText(renderer, font, "ORIENTATION", sidebar_x + 15, y, white);
        y += 25;
        
        snprintf(buf, sizeof(buf), "Yaw:   %+7.2f deg", result.yaw_deg);
        SDL_Color yaw_color = (std::fabs(result.yaw_deg) < 3.0f) ? green : yellow;
        renderText(renderer, font, buf, sidebar_x + 20, y, yaw_color);
        y += 20;
        
        snprintf(buf, sizeof(buf), "Pitch: %+7.2f deg", result.pitch_deg);
        SDL_Color pitch_color = (std::fabs(result.pitch_deg) < 3.0f) ? green : yellow;
        renderText(renderer, font, buf, sidebar_x + 20, y, pitch_color);
        y += 30;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Tilt indicator
        renderText(renderer, font, "TILT INDICATOR", sidebar_x + 15, y, white);
        y += 25;
        
        int box_size = 100;
        int box_x = sidebar_x + (SIDEBAR_WIDTH - box_size) / 2;
        int box_y = y;
        
        // Background
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_Rect tilt_bg;
        tilt_bg.x = box_x;
        tilt_bg.y = box_y;
        tilt_bg.w = box_size;
        tilt_bg.h = box_size;
        SDL_RenderFillRect(renderer, &tilt_bg);
        
        // Border
        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawRect(renderer, &tilt_bg);
        
        // Center crosshair
        int bcx = box_x + box_size / 2;
        int bcy = box_y + box_size / 2;
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, box_x, bcy, box_x + box_size, bcy);
        SDL_RenderDrawLine(renderer, bcx, box_y, bcx, box_y + box_size);
        
        // Dot position
        float max_yaw = 28.0f;
        float max_pitch = 22.0f;
        int dot_x = bcx + static_cast<int>((result.yaw_deg / max_yaw) * (box_size / 2));
        int dot_y = bcy - static_cast<int>((result.pitch_deg / max_pitch) * (box_size / 2));
        
        dot_x = std::max(box_x + 2, std::min(box_x + box_size - 2, dot_x));
        dot_y = std::max(box_y + 2, std::min(box_y + box_size - 2, dot_y));
        
        SDL_Color dot_color = (std::fabs(result.yaw_deg) < 3.0f && 
                               std::fabs(result.pitch_deg) < 3.0f) ? green : yellow;
        SDL_SetRenderDrawColor(renderer, dot_color.r, dot_color.g, dot_color.b, 255);
        
        SDL_Rect dot;
        dot.x = dot_x - 2;
        dot.y = dot_y - 2;
        dot.w = 4;
        dot.h = 4;
        SDL_RenderFillRect(renderer, &dot);
        
        // Labels
        renderText(renderer, font, "L", box_x - 12, bcy - 7, gray);
        renderText(renderer, font, "R", box_x + box_size + 4, bcy - 7, gray);
        renderText(renderer, font, "U", bcx - 4, box_y - 16, gray);
        renderText(renderer, font, "D", bcx - 4, box_y + box_size + 2, gray);
        
    } else {
        // Not detected
        renderText(renderer, font_large, "---", sidebar_x + 15, y, red);
        y += 50;
        
        renderText(renderer, font, "Grid not detected", sidebar_x + 15, y, gray);
        y += 20;
        
        snprintf(buf, sizeof(buf), "Matched: %d dots", result.main_dots_detected);
        renderText(renderer, font, buf, sidebar_x + 15, y, gray);
        y += 20;
        
        snprintf(buf, sizeof(buf), "Rows: %d", result.num_main_rows);
        renderText(renderer, font, buf, sidebar_x + 15, y, gray);
        y += 20;
        
        renderText(renderer, font, "Need 4+ dots OR 2+ rows", sidebar_x + 15, y, yellow);
    }
    
    // Footer
    y = window_height - 55;
    
    const char* mode_str = isolation_mode ? "ISOLATION: ON" : "ISOLATION: OFF";
    SDL_Color mode_color = isolation_mode ? green : yellow;
    renderText(renderer, font, mode_str, sidebar_x + 15, y, mode_color);
    y += 18;
    
    renderText(renderer, font, "Press I to toggle", sidebar_x + 15, y, gray);
    y += 18;
    renderText(renderer, font, "Press Q to quit", sidebar_x + 15, y, gray);
}

} // namespace thermal