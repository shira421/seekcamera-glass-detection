/**
 * @file rendering.h
 * @brief UI rendering functions for thermal distance sensor display
 * 
 * Provides functions for drawing the thermal image visualization,
 * spot markers, crosshairs, and sidebar UI components.
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
 * @brief Render text to the screen
 * 
 * @param renderer SDL renderer
 * @param font TTF font to use
 * @param text Text string to render
 * @param x X position
 * @param y Y position
 * @param color Text color
 */
void renderText(SDL_Renderer* renderer, TTF_Font* font, const char* text,
                int x, int y, SDL_Color color);

/**
 * @brief Draw center crosshair on thermal image
 * 
 * Draws a white 2-pixel thick crosshair at the center of the display area.
 * 
 * @param renderer SDL renderer
 * @param width Display width (excluding sidebar)
 * @param height Display height
 */
void drawCrosshair(SDL_Renderer* renderer, int width, int height);

/**
 * @brief Draw marker for detected heat spot
 * 
 * Draws a colored box around the spot position with a label showing
 * the spot name and temperature.
 * 
 * @param renderer SDL renderer
 * @param font Font for label text
 * @param x Spot X position in thermal coordinates
 * @param y Spot Y position in thermal coordinates
 * @param temp Spot temperature (°C)
 * @param color Marker color
 * @param scale Display scale factor
 * @param label Label text ("ABOVE" or "LEFT")
 */
void drawSpotMarker(SDL_Renderer* renderer, TTF_Font* font,
                    int x, int y, float temp, SDL_Color color,
                    int scale, const char* label);

/**
 * @brief Draw camera pointing indicator
 * 
 * Draws a small red dot at the intersection point showing where
 * the camera is actually pointing (X from ABOVE spot, Y from LEFT spot).
 * 
 * @param renderer SDL renderer
 * @param font Font (unused, for consistency)
 * @param spot1_x ABOVE spot X position
 * @param spot1_y ABOVE spot Y position
 * @param spot2_x LEFT spot X position
 * @param spot2_y LEFT spot Y position
 * @param scale Display scale factor
 */
void drawCameraMarker(SDL_Renderer* renderer, TTF_Font* font,
                      int spot1_x, int spot1_y, int spot2_x, int spot2_y,
                      int scale);

/**
 * @brief Draw the sidebar UI panel
 * 
 * Displays:
 * - Distance measurement (large text)
 * - Spot temperatures
 * - Camera orientation (yaw/pitch)
 * - Visual tilt indicator
 * - Mode status and controls
 * 
 * @param renderer SDL renderer
 * @param font Regular font
 * @param font_large Large font for distance display
 * @param detected Whether spots were detected
 * @param distance_cm Calculated distance
 * @param camera_yaw_deg Camera yaw angle
 * @param camera_pitch_deg Camera pitch angle
 * @param spot1_temp ABOVE spot temperature
 * @param spot2_temp LEFT spot temperature
 * @param isolation_mode Current isolation mode state
 * @param window_width Total window width
 * @param window_height Window height
 */
void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_large,
                 bool detected, float distance_cm, 
                 float camera_yaw_deg, float camera_pitch_deg,
                 float spot1_temp, float spot2_temp, bool isolation_mode,
                 int window_width, int window_height);

} // namespace thermal

#endif // THERMAL_RENDERING_H