/**
 * @file rendering.cpp
 * @brief Implementation of UI rendering functions
 */

#include "rendering.h"
#include <cstdio>
#include <cmath>

namespace thermal {

//=============================================================================
// TEXT RENDERING
//=============================================================================

void renderText(SDL_Renderer* renderer, TTF_Font* font, const char* text,
                int x, int y, SDL_Color color) {
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

//=============================================================================
// OVERLAY DRAWING
//=============================================================================

void drawCrosshair(SDL_Renderer* renderer, int width, int height) {
    int cx = width / 2;
    int cy = height / 2;
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    
    // Horizontal lines (2 pixels thick)
    SDL_RenderDrawLine(renderer, cx - 6, cy, cx + 6, cy);
    SDL_RenderDrawLine(renderer, cx - 6, cy + 1, cx + 6, cy + 1);
    
    // Vertical lines (2 pixels thick)
    SDL_RenderDrawLine(renderer, cx, cy - 6, cx, cy + 6);
    SDL_RenderDrawLine(renderer, cx + 1, cy - 6, cx + 1, cy + 6);
}

void drawSpotMarker(SDL_Renderer* renderer, TTF_Font* font,
                    int x, int y, float temp, SDL_Color color,
                    int scale, const char* label) {
    int dx = x * scale;
    int dy = y * scale;
    
    // Draw box around spot
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
    SDL_Rect box;
    box.x = dx - 3;
    box.y = dy - 3;
    box.w = 6;
    box.h = 6;
    SDL_RenderDrawRect(renderer, &box);
    
    // Draw label with temperature
    if (font && label) {
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
                      int spot1_x, int spot1_y, int spot2_x, int spot2_y,
                      int scale) {
    (void)font;  // Unused parameter
    
    // Camera points at: X from ABOVE spot (spot1), Y from LEFT spot (spot2)
    int camera_point_x = spot1_x * scale;
    int camera_point_y = spot2_y * scale;
    
    // Draw small red dot
    SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    SDL_Rect center_dot;
    center_dot.x = camera_point_x - 2;
    center_dot.y = camera_point_y - 2;
    center_dot.w = 4;
    center_dot.h = 4;
    SDL_RenderFillRect(renderer, &center_dot);
}

//=============================================================================
// SIDEBAR UI
//=============================================================================

void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_large,
                 bool detected, float distance_cm, 
                 float camera_yaw_deg, float camera_pitch_deg,
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
    
    // Define colors
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
        // Distance display (large, green)
        char dist_text[32];
        snprintf(dist_text, sizeof(dist_text), "%.2f cm", distance_cm);
        renderText(renderer, font_large, dist_text, sidebar_x + 15, y, green);
        y += 45;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Spot temperatures section
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
        
        // Camera orientation section
        renderText(renderer, font, "CAMERA ORIENTATION", sidebar_x + 15, y, white);
        y += 25;
        
        // Yaw display
        char yaw_text[48];
        snprintf(yaw_text, sizeof(yaw_text), "Yaw:   %+7.2f deg", camera_yaw_deg);
        SDL_Color yaw_color = (fabs(camera_yaw_deg) < 2.0f) ? green : yellow;
        renderText(renderer, font, yaw_text, sidebar_x + 20, y, yaw_color);
        y += 22;
        
        // Pitch display
        char pitch_text[48];
        snprintf(pitch_text, sizeof(pitch_text), "Pitch: %+7.2f deg", camera_pitch_deg);
        SDL_Color pitch_color = (fabs(camera_pitch_deg) < 2.0f) ? green : yellow;
        renderText(renderer, font, pitch_text, sidebar_x + 20, y, pitch_color);
        y += 30;
        
        // Separator
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y, sidebar_x + SIDEBAR_WIDTH - 10, y);
        y += 15;
        
        // Tilt indicator section
        renderText(renderer, font, "CAMERA TILT", sidebar_x + 15, y, white);
        y += 25;
        
        // Draw tilt indicator box
        int box_size = 100;
        int box_x = sidebar_x + (SIDEBAR_WIDTH - box_size) / 2;
        int box_y = y;
        
        // Box background
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_Rect tilt_box;
        tilt_box.x = box_x;
        tilt_box.y = box_y;
        tilt_box.w = box_size;
        tilt_box.h = box_size;
        SDL_RenderFillRect(renderer, &tilt_box);
        
        // Box border
        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawRect(renderer, &tilt_box);
        
        // Center crosshairs
        int bcx = box_x + box_size / 2;
        int bcy = box_y + box_size / 2;
        SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
        SDL_RenderDrawLine(renderer, box_x, bcy, box_x + box_size, bcy);
        SDL_RenderDrawLine(renderer, bcx, box_y, bcx, box_y + box_size);
        
        // Calculate dot position
        // Full FOV: HFOV = 56° (±28), VFOV ≈ 44° (±22)
        float max_yaw = 28.0f;
        float max_pitch = 22.0f;
        int dot_x = bcx + static_cast<int>(camera_yaw_deg / max_yaw * (box_size / 2));
        int dot_y = bcy - static_cast<int>(camera_pitch_deg / max_pitch * (box_size / 2));
        
        // Clamp to box bounds
        dot_x = std::max(box_x + 2, std::min(box_x + box_size - 2, dot_x));
        dot_y = std::max(box_y + 2, std::min(box_y + box_size - 2, dot_y));
        
        // Draw indicator dot
        SDL_Color dot_color = (fabs(camera_yaw_deg) < 2.0f && fabs(camera_pitch_deg) < 2.0f) 
                              ? green : yellow;
        SDL_SetRenderDrawColor(renderer, dot_color.r, dot_color.g, dot_color.b, 255);
        SDL_Rect dot;
        dot.x = dot_x - 1;
        dot.y = dot_y - 1;
        dot.w = 3;
        dot.h = 3;
        SDL_RenderFillRect(renderer, &dot);
        
        // Direction labels
        renderText(renderer, font, "L", box_x - 12, bcy - 7, gray);
        renderText(renderer, font, "R", box_x + box_size + 4, bcy - 7, gray);
        renderText(renderer, font, "U", bcx - 4, box_y - 16, gray);
        renderText(renderer, font, "D", bcx - 4, box_y + box_size + 2, gray);
        
    } else {
        // No detection state
        renderText(renderer, font, "Searching...", sidebar_x + 15, y, red);
        y += 25;
        renderText(renderer, font, "No heat spots detected", sidebar_x + 15, y, dark_gray);
    }
    
    // Footer section
    y = window_height - 55;
    
    // Isolation mode indicator
    const char* mode_text = isolation_mode ? "ISOLATION: ON (top 15%)" : "ISOLATION: OFF (full range)";
    SDL_Color mode_color = isolation_mode ? green : yellow;
    renderText(renderer, font, mode_text, sidebar_x + 15, y, mode_color);
    y += 18;
    
    // Control hints
    renderText(renderer, font, "Press I to toggle isolation", sidebar_x + 15, y, gray);
    y += 18;
    renderText(renderer, font, "Press Q to quit", sidebar_x + 15, y, gray);
}

} // namespace thermal