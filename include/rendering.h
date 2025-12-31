/**
 * @file rendering.h
 * @brief Rendering functions for thermal distance sensor display
 */

#ifndef THERMAL_RENDERING_H
#define THERMAL_RENDERING_H

#include "thermal_distance_sensor.h"

#if defined(__linux__) || defined(__APPLE__)
#   include <SDL2/SDL.h>
#   include <SDL2/SDL_ttf.h>
#elif defined(_WIN32)
#   include <SDL.h>
#   include <SDL_ttf.h>
#endif

namespace thermal {

/**
 * @brief Render text at position
 */
void renderText(SDL_Renderer* renderer, TTF_Font* font, const char* text,
                int x, int y, SDL_Color color);

/**
 * @brief Draw center crosshair on thermal display
 */
void drawCrosshair(SDL_Renderer* renderer, int width, int height);

/**
 * @brief Draw all 45 grid dots (green for main, expected positions for undetected)
 */
void drawGridDots(SDL_Renderer* renderer, const GridResult& result, int scale);

/**
 * @brief Draw bounding box of main grid
 */
void drawGridBounds(SDL_Renderer* renderer, const GridResult& result, int scale);

/**
 * @brief Draw the sidebar with measurements and info
 */
void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_large,
                 const GridResult& result, bool isolation_mode,
                 int window_width, int window_height);

} // namespace thermal

#endif // THERMAL_RENDERING_H