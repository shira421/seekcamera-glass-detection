/*
 * Thermal Object Tracker - Custom Color Mapping
 * Maps raw temperature data to custom colors with steep gradient
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

 // COLOR MODES - must be defined BEFORE structs
enum ColorMode {
    ABSOLUTE,    // Fixed temp range (20-70°C) - for distance sensing
    RELATIVE     // Frame min/max - for debugging/visibility
};

// ABSOLUTE mode temperature range
const float ABS_TEMP_MIN = 22.0f;
const float ABS_TEMP_MAX = 30.0f;

// DETECTION PARAMETERS
const int MIN_OBJECT_SIZE = 15;
const int SIDEBAR_WIDTH = 300;

// Thermal Object structure
struct ThermalObject {
    int id;
    int x, y, width, height;
    float min_temp;
    float max_temp;
    float avg_temp;
    SDL_Color box_color;
};

// Renderer structure
struct seekrenderer_t {
    seekcamera_t* camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
    TTF_Font* font_small;
    std::mutex the_mutex;
    std::atomic<bool> is_active;
    std::atomic<bool> is_dirty;
    seekcamera_frame_t* frame;

    std::vector<ThermalObject> detected_objects;
    int next_object_id;
    float center_pixel_temp;
    float user_temp_threshold;
    ColorMode color_mode;  // Toggle between ABSOLUTE and RELATIVE

    int mouse_x, mouse_y;
    bool mouse_down;

    seekrenderer_t() : camera(nullptr), window(nullptr), renderer(nullptr),
        texture(nullptr), font(nullptr), font_small(nullptr),
        frame(nullptr), next_object_id(1),
        center_pixel_temp(0.0f), user_temp_threshold(35.0f),
        color_mode(ABSOLUTE),  // Default to ABSOLUTE for production
        mouse_x(0), mouse_y(0), mouse_down(false) {
        is_active.store(false);
        is_dirty.store(false);
    }
};

static std::atomic<bool> g_exit_requested;
static std::map<std::string, seekrenderer_t*> g_renderers;
static std::mutex g_mutex;
static std::condition_variable g_condition_variable;
static std::atomic<bool> g_is_dirty;

// ABSOLUTE COLOR MAPPING - Fixed temp range (20-70°C)
// Always maps same temperature to same color
SDL_Color mapTemperatureAbsolute(float temp) {
    SDL_Color color;

    // Clamp to absolute range
    float normalized = (temp - ABS_TEMP_MIN) / (ABS_TEMP_MAX - ABS_TEMP_MIN);
    normalized = std::max(0.0f, std::min(1.0f, normalized));

    // Apply steep power curve for sensitivity
    normalized = std::pow(normalized, 0.3f);  // Steep but not crazy

    // Map to vibrant gradient: Blue → Cyan → Green → Yellow → Red
    if (normalized < 0.25f) {
        // Blue to Cyan
        float t = normalized * 4.0f;
        color.r = 0;
        color.g = (Uint8)(t * 255);
        color.b = 255;
    }
    else if (normalized < 0.5f) {
        // Cyan to Green
        float t = (normalized - 0.25f) * 4.0f;
        color.r = 0;
        color.g = 255;
        color.b = (Uint8)((1.0f - t) * 255);
    }
    else if (normalized < 0.75f) {
        // Green to Yellow
        float t = (normalized - 0.5f) * 4.0f;
        color.r = (Uint8)(t * 255);
        color.g = 255;
        color.b = 0;
    }
    else {
        // Yellow to Red
        float t = (normalized - 0.75f) * 4.0f;
        color.r = 255;
        color.g = (Uint8)((1.0f - t) * 255);
        color.b = 0;
    }

    color.a = 255;
    return color;
}

// RELATIVE COLOR MAPPING - Frame min/max (ultra steep for debugging)
// Maps relative to current frame for maximum visibility
SDL_Color mapTemperatureRelative(float temp, float min_temp, float max_temp) {
    SDL_Color color;

    float range = max_temp - min_temp;
    if (range < 0.01f) {
        // Uniform temperature - show as dark
        color.r = 20; color.g = 20; color.b = 20; color.a = 255;
        return color;
    }

    // Normalize temperature relative to THIS frame
    float normalized = (temp - min_temp) / range;

    // Apply ULTRA STEEP power curve
    normalized = std::pow(normalized, 0.15f);  // Very steep for debugging!

    // Map to vibrant gradient: Blue → Cyan → Green → Yellow → Red
    if (normalized < 0.25f) {
        // Blue to Cyan
        float t = normalized * 4.0f;
        color.r = 0;
        color.g = (Uint8)(t * 255);
        color.b = 255;
    }
    else if (normalized < 0.5f) {
        // Cyan to Green
        float t = (normalized - 0.25f) * 4.0f;
        color.r = 0;
        color.g = 255;
        color.b = (Uint8)((1.0f - t) * 255);
    }
    else if (normalized < 0.75f) {
        // Green to Yellow
        float t = (normalized - 0.5f) * 4.0f;
        color.r = (Uint8)(t * 255);
        color.g = 255;
        color.b = 0;
    }
    else {
        // Yellow to Red
        float t = (normalized - 0.75f) * 4.0f;
        color.r = 255;
        color.g = (Uint8)((1.0f - t) * 255);
        color.b = 0;
    }

    color.a = 255;
    return color;
}

// Get color based on temperature (for bounding boxes)
SDL_Color getTemperatureColor(float temp, float min_threshold, float max_range) {
    SDL_Color color;

    if (temp < min_threshold) {
        temp = min_threshold;
    }

    float normalized = (temp - min_threshold) / max_range;
    normalized = std::max(0.0f, std::min(1.0f, normalized));

    // Bright Yellow -> Orange -> Dark Red
    if (normalized < 0.5f) {
        float t = normalized * 2.0f;
        color.r = 255;
        color.g = (Uint8)(255 - (127 * t));
        color.b = 0;
    }
    else {
        float t = (normalized - 0.5f) * 2.0f;
        color.r = (Uint8)(255 - (75 * t));
        color.g = (Uint8)(128 * (1.0f - t));
        color.b = 0;
    }

    color.a = 255;
    return color;
}

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

                    const int dx[] = { -1, 1, 0, 0 };
                    const int dy[] = { 0, 0, -1, 1 };

                    for (int i = 0; i < 4; i++) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];

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

// Detect hot objects
std::vector<ThermalObject> detectHotObjects(float* temps, int width, int height, int& next_id, float threshold) {
    std::vector<ThermalObject> objects;

    std::vector<std::vector<bool> > hot_pixels(height, std::vector<bool>(width, false));

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (temps[y * width + x] >= threshold) {
                hot_pixels[y][x] = true;
            }
        }
    }

    std::vector<std::vector<int> > labels = findConnectedComponents(hot_pixels, width, height);

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

    for (std::map<int, ThermalObject>::iterator it = object_map.begin();
        it != object_map.end(); ++it) {
        ThermalObject& obj = it->second;

        if (obj.width < MIN_OBJECT_SIZE || obj.height < MIN_OBJECT_SIZE) {
            continue;
        }

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

        obj.box_color = getTemperatureColor(obj.avg_temp, threshold, 80.0f);

        objects.push_back(obj);
    }

    return objects;
}

// Draw bounding box
void drawThermalObject(SDL_Renderer* renderer, TTF_Font* font, const ThermalObject& obj, int scale_factor) {
    SDL_Rect box;
    box.x = obj.x * scale_factor;
    box.y = obj.y * scale_factor;
    box.w = obj.width * scale_factor;
    box.h = obj.height * scale_factor;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(renderer, obj.box_color.r, obj.box_color.g, obj.box_color.b, 255);

    int border_thickness = 2;

    for (int i = 0; i < border_thickness; i++) {
        SDL_Rect border;
        border.x = box.x - i;
        border.y = box.y - i;
        border.w = box.w + 2 * i;
        border.h = box.h + 2 * i;
        SDL_RenderDrawRect(renderer, &border);
    }

    // Draw text label
    if (font) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << obj.avg_temp << " C";
        std::string label = ss.str();

        SDL_Color textColor = { 255, 255, 255, 255 };

        SDL_Surface* textSurface = TTF_RenderText_Solid(font, label.c_str(), textColor);
        if (textSurface) {
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            if (textTexture) {
                SDL_Rect textRect;
                textRect.x = box.x;
                textRect.y = box.y - 22;
                textRect.w = textSurface->w;
                textRect.h = textSurface->h;

                SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 200);
                SDL_Rect bgRect;
                bgRect.x = textRect.x - 2; bgRect.y = textRect.y - 2;
                bgRect.w = textRect.w + 4; bgRect.h = textRect.h + 4;
                SDL_RenderFillRect(renderer, &bgRect);

                SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
                SDL_DestroyTexture(textTexture);
            }
            SDL_FreeSurface(textSurface);
        }
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
}

// Draw center crosshair
void drawCenterCrosshair(SDL_Renderer* renderer, TTF_Font* font, int width, int height, float temp, float threshold) {
    int center_x = width / 2;
    int center_y = height / 2;
    int crosshair_size = 6;

    if (temp >= threshold) {
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    }
    else {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    SDL_RenderDrawLine(renderer, center_x - crosshair_size, center_y, center_x + crosshair_size, center_y);
    SDL_RenderDrawLine(renderer, center_x, center_y - crosshair_size, center_x, center_y + crosshair_size);

    SDL_Rect dot;
    dot.x = center_x - 1;
    dot.y = center_y - 1;
    dot.w = 2;
    dot.h = 2;
    SDL_RenderFillRect(renderer, &dot);
}

// Helper to render text
void renderTextLine(SDL_Renderer* renderer, TTF_Font* font, const char* text, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Blended(font, text, color);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect rect = { x, y, surface->w, surface->h };
            SDL_RenderCopy(renderer, texture, NULL, &rect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

// Draw sidebar
void drawSidebar(SDL_Renderer* renderer, TTF_Font* font, TTF_Font* font_small,
    const std::vector<ThermalObject>& objects, float center_temp,
    int center_x_thermal, int center_y_thermal, float& user_threshold,
    ColorMode color_mode, int window_width, int window_height,
    int mouse_x, int mouse_y, bool mouse_down) {

    int sidebar_x = window_width - SIDEBAR_WIDTH;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_Rect sidebar = { sidebar_x, 0, SIDEBAR_WIDTH, window_height };
    SDL_RenderFillRect(renderer, &sidebar);

    SDL_Color white = { 255, 255, 255, 255 };
    SDL_Color black = { 0, 0, 0, 255 };
    int y_pos = 15;

    // Temperature Threshold Control
    renderTextLine(renderer, font, "Temperature Threshold", sidebar_x + 10, y_pos, white);
    y_pos += 25;

    int slider_x = sidebar_x + 10;
    int slider_y = y_pos;
    int slider_width = 280;
    int slider_height = 20;

    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_Rect slider_bg = { slider_x, slider_y, slider_width, slider_height };
    SDL_RenderFillRect(renderer, &slider_bg);

    float normalized = (user_threshold - 20.0f) / 80.0f;
    int fill_width = (int)(slider_width * normalized);
    SDL_SetRenderDrawColor(renderer, 0, 200, 0, 255);
    SDL_Rect slider_fill = { slider_x, slider_y, fill_width, slider_height };
    SDL_RenderFillRect(renderer, &slider_fill);

    int handle_x = slider_x + fill_width - 5;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_Rect slider_handle = { handle_x, slider_y - 2, 10, slider_height + 4 };
    SDL_RenderFillRect(renderer, &slider_handle);

    if (mouse_down && mouse_y >= slider_y && mouse_y <= slider_y + slider_height &&
        mouse_x >= slider_x && mouse_x <= slider_x + slider_width) {
        float new_normalized = (float)(mouse_x - slider_x) / slider_width;
        new_normalized = std::max(0.0f, std::min(1.0f, new_normalized));
        user_threshold = 20.0f + (new_normalized * 80.0f);
    }

    y_pos += 25;
    char threshold_text[50];
    snprintf(threshold_text, sizeof(threshold_text), "%.1f C (20-100 C range)", user_threshold);
    renderTextLine(renderer, font_small, threshold_text, sidebar_x + 10, y_pos, white);
    y_pos += 25;

    // Center Pixel Info
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 12;

    renderTextLine(renderer, font, "Center Pixel", sidebar_x + 10, y_pos, white);
    y_pos += 25;

    SDL_Color temp_color = getTemperatureColor(center_temp, user_threshold, 80.0f);
    SDL_SetRenderDrawColor(renderer, temp_color.r, temp_color.g, temp_color.b, 255);
    SDL_Rect temp_bar = { sidebar_x + 10, y_pos, 280, 30 };
    SDL_RenderFillRect(renderer, &temp_bar);

    char temp_text[50];
    snprintf(temp_text, sizeof(temp_text), "%.1f C", center_temp);
    renderTextLine(renderer, font, temp_text, sidebar_x + 20, y_pos + 5, black);
    y_pos += 38;

    char pos_text[100];
    snprintf(pos_text, sizeof(pos_text), "Pos: (%d, %d)", center_x_thermal, center_y_thermal);
    renderTextLine(renderer, font_small, pos_text, sidebar_x + 10, y_pos, white);
    y_pos += 18;

    // Find targeted object
    const ThermalObject* targeted_object = NULL;
    for (size_t i = 0; i < objects.size(); i++) {
        const ThermalObject& obj = objects[i];
        if (center_x_thermal >= obj.x && center_x_thermal < obj.x + obj.width &&
            center_y_thermal >= obj.y && center_y_thermal < obj.y + obj.height) {
            targeted_object = &obj;
            break;
        }
    }

    const char* status_text;
    if (targeted_object != NULL) {
        status_text = "Status: Inside Object";
    }
    else if (center_temp >= user_threshold) {
        status_text = "Status: Above Threshold";
    }
    else {
        status_text = "Status: Below Threshold";
    }
    renderTextLine(renderer, font_small, status_text, sidebar_x + 10, y_pos, white);
    y_pos += 25;

    // Targeted Object
    if (targeted_object != NULL) {
        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
        y_pos += 12;

        renderTextLine(renderer, font, "Targeted Object", sidebar_x + 10, y_pos, white);
        y_pos += 25;

        SDL_SetRenderDrawColor(renderer, targeted_object->box_color.r,
            targeted_object->box_color.g,
            targeted_object->box_color.b, 255);
        SDL_Rect colorRect = { sidebar_x + 10, y_pos, 8, 60 };
        SDL_RenderFillRect(renderer, &colorRect);

        int text_x = sidebar_x + 22;
        char line[100];

        snprintf(line, sizeof(line), "Avg: %.1f C", targeted_object->avg_temp);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 18;

        snprintf(line, sizeof(line), "Min: %.1f C Max: %.1f C", targeted_object->min_temp, targeted_object->max_temp);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 18;

        snprintf(line, sizeof(line), "Size: %dx%d px (%d px)",
            targeted_object->width, targeted_object->height,
            targeted_object->width * targeted_object->height);
        renderTextLine(renderer, font_small, line, text_x, y_pos, white);
        y_pos += 25;
    }

    // Detection Summary
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 12;

    renderTextLine(renderer, font, "Detection Summary", sidebar_x + 10, y_pos, white);
    y_pos += 25;

    char summary[100];
    snprintf(summary, sizeof(summary), "Detected Objects: %zu", objects.size());
    renderTextLine(renderer, font_small, summary, sidebar_x + 10, y_pos, white);
    y_pos += 18;

    snprintf(summary, sizeof(summary), "Active Threshold: %.1f C", user_threshold);
    renderTextLine(renderer, font_small, summary, sidebar_x + 10, y_pos, white);
    y_pos += 25;

    // Color Mode Display
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawLine(renderer, sidebar_x + 10, y_pos, sidebar_x + SIDEBAR_WIDTH - 10, y_pos);
    y_pos += 12;

    renderTextLine(renderer, font, "Color Mode", sidebar_x + 10, y_pos, white);
    y_pos += 25;

    const char* mode_text = (color_mode == ABSOLUTE) ? "ABSOLUTE (22-30 C)" : "RELATIVE (Frame Min/Max)";
    SDL_Color mode_color = (color_mode == ABSOLUTE) ?
        SDL_Color{ 0, 255, 100, 255 } :  // Green for ABSOLUTE
        SDL_Color{ 255, 200, 0, 255 };    // Yellow for RELATIVE
    renderTextLine(renderer, font_small, mode_text, sidebar_x + 10, y_pos, mode_color);
    y_pos += 18;

    const char* usage_text = (color_mode == ABSOLUTE) ?
        "For: Distance sensing" : "For: Max visibility";
    renderTextLine(renderer, font_small, usage_text, sidebar_x + 10, y_pos, white);
    y_pos += 18;

    renderTextLine(renderer, font_small, "Press 'M' to toggle", sidebar_x + 10, y_pos, SDL_Color{ 150, 150, 150, 255 });
}

void handle_camera_frame_available(seekcamera_t* camera, seekcamera_frame_t* camera_frame, void* user_data) {
    (void)camera;
    seekrenderer_t* renderer = (seekrenderer_t*)user_data;

    std::lock_guard<std::mutex> guard(renderer->the_mutex);

    seekcamera_frame_lock(camera_frame);
    renderer->is_dirty.store(true);
    renderer->frame = camera_frame;

    seekframe_t* thermal_frame = nullptr;
    if (seekcamera_frame_get_frame_by_format(camera_frame,
        SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT,
        &thermal_frame) == SEEKCAMERA_SUCCESS) {

        float* temps = (float*)seekframe_get_data(thermal_frame);
        int width = seekframe_get_width(thermal_frame);
        int height = seekframe_get_height(thermal_frame);

        if (temps && width > 0 && height > 0) {
            renderer->detected_objects = detectHotObjects(temps, width, height,
                renderer->next_object_id,
                renderer->user_temp_threshold);

            int center_x = width / 2;
            int center_y = height / 2;
            renderer->center_pixel_temp = temps[center_y * width + center_x];
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

    seekrenderer_t* renderer = g_renderers[chipID] == nullptr ? new seekrenderer_t() : g_renderers[chipID];
    renderer->is_active.store(true);
    renderer->camera = camera;

    seekcamera_error_t status = seekcamera_register_frame_available_callback(camera, handle_camera_frame_available, (void*)renderer);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to register frame callback: " << seekcamera_error_get_str(status) << std::endl;
        renderer->is_active.store(false);
        return;
    }

    // Use WHITE_HOT as base - we'll override with custom colors anyway
    status = seekcamera_set_color_palette(camera, SEEKCAMERA_COLOR_PALETTE_WHITE_HOT);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to set color palette: " << seekcamera_error_get_str(status) << std::endl;
    }

    uint32_t frame_formats = SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
    status = seekcamera_capture_session_start(camera, frame_formats);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to start capture session: " << seekcamera_error_get_str(status) << std::endl;
        renderer->is_active.store(false);
        return;
    }

    g_renderers[chipID] = renderer;
    std::cout << "Camera connected: " << chipID << std::endl;
}

void handle_camera_disconnect(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID((char*)&cid);
    seekrenderer_t* renderer = g_renderers[chipID];

    seekcamera_capture_session_stop(camera);

    if (renderer != nullptr) {
        renderer->is_active.store(false);
        g_is_dirty.store(true);
    }
}

void handle_camera_error(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    (void)user_data;
    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::cerr << "Camera error (CID: " << cid << "): " << seekcamera_error_get_str(event_status) << std::endl;
}

void handle_camera_ready_to_pair(seekcamera_t* camera, seekcamera_error_t event_status, void* user_data) {
    const seekcamera_error_t status = seekcamera_store_calibration_data(camera, nullptr, nullptr, nullptr);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to pair device: " << seekcamera_error_get_str(status) << std::endl;
    }
    handle_camera_connect(camera, event_status, user_data);
}

void camera_event_callback(seekcamera_t* camera, seekcamera_manager_event_t event, seekcamera_error_t event_status, void* user_data) {
    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::cout << seekcamera_manager_get_event_str(event) << " (CID: " << cid << ")" << std::endl;

    switch (event) {
    case SEEKCAMERA_MANAGER_EVENT_CONNECT:
        handle_camera_connect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
        handle_camera_disconnect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_ERROR:
        handle_camera_error(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
        handle_camera_ready_to_pair(camera, event_status, user_data);
        break;
    default:
        break;
    }
}

void seekrenderer_close_window(seekrenderer_t* renderer) {
    if (renderer == nullptr) return;

    if (renderer->is_active.load())
        seekcamera_capture_session_stop(renderer->camera);

    renderer->is_active.store(false);
    renderer->is_dirty.store(false);
    renderer->frame = nullptr;

    if (renderer->font != nullptr) {
        TTF_CloseFont(renderer->font);
        renderer->font = nullptr;
    }

    if (renderer->font_small != nullptr) {
        TTF_CloseFont(renderer->font_small);
        renderer->font_small = nullptr;
    }

    if (renderer->texture != nullptr) {
        SDL_DestroyTexture(renderer->texture);
        renderer->texture = nullptr;
    }

    if (renderer->renderer != nullptr) {
        SDL_DestroyRenderer(renderer->renderer);
        renderer->renderer = nullptr;
    }

    if (renderer->window != nullptr) {
        SDL_DestroyWindow(renderer->window);
        renderer->window = nullptr;
    }

    renderer->camera = nullptr;
}

static void signal_callback(int signum) {
    (void)signum;
    std::cout << "\nCaught termination signal" << std::endl;
    g_exit_requested.store(true);
}

int main() {
    g_exit_requested.store(false);
    g_is_dirty.store(false);

    signal(SIGINT, signal_callback);
    signal(SIGTERM, signal_callback);

    std::cout << "Thermal Object Tracker - Hybrid Color Mapping" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "- HYBRID system: Toggle between ABSOLUTE and RELATIVE" << std::endl;
    std::cout << "- ABSOLUTE mode: 22-30 C fixed (optimized for low-temp sources)" << std::endl;
    std::cout << "- RELATIVE mode: Frame min/max (for debugging)" << std::endl;
    std::cout << "- Custom color mapping bypasses hardware palette!" << std::endl;
    std::cout << "- Blue (cool) -> Cyan -> Green -> Yellow -> Red (hot)" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "- Press 'M' to toggle between ABSOLUTE/RELATIVE modes" << std::endl;
    std::cout << "- Press 'Q' to quit" << std::endl;
    std::cout << "- Drag slider (20-100 C) to set detection threshold" << std::endl;
    std::cout << "\nStarting in ABSOLUTE mode (22-30 C optimized for your heat source)\n" << std::endl;

    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    seekcamera_manager_t* manager = nullptr;
    seekcamera_error_t status = seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager: " << seekcamera_error_get_str(status) << std::endl;
        return 1;
    }

    status = seekcamera_manager_register_event_callback(manager, camera_event_callback, nullptr);
    if (status != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to register event callback: " << seekcamera_error_get_str(status) << std::endl;
        return 1;
    }

    while (!g_exit_requested.load()) {
        std::unique_lock<std::mutex> event_lock(g_mutex);

        if (g_condition_variable.wait_for(event_lock, std::chrono::milliseconds(150),
            []() { return g_is_dirty.load(); })) {
            g_is_dirty.store(false);

            for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                it != g_renderers.end(); ++it) {
                seekrenderer_t* renderer = it->second;
                if (renderer == nullptr) continue;

                if (!renderer->is_active.load()) {
                    if (renderer->window != nullptr)
                        seekrenderer_close_window(renderer);
                    continue;
                }

                if (renderer->is_active.load() && renderer->window == nullptr) {
                    seekcamera_chipid_t cid;
                    memset(&cid, 0, sizeof(cid));
                    seekcamera_get_chipid(renderer->camera, &cid);
                    std::stringstream window_title;
                    window_title << "Thermal Object Tracker - Custom Colors (CID: " << cid << ")";

                    renderer->window = SDL_CreateWindow(window_title.str().c_str(), 100, 100, 0, 0, SDL_WINDOW_HIDDEN);
                    renderer->renderer = SDL_CreateRenderer(renderer->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

                    renderer->font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
                    if (!renderer->font) {
                        renderer->font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
                    }

                    renderer->font_small = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12);
                    if (!renderer->font_small) {
                        renderer->font_small = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 12);
                    }
                }

                {
                    std::lock_guard<std::mutex> guard(renderer->the_mutex);

                    if (renderer->is_dirty.load() && renderer->frame != nullptr && renderer->is_active.load()) {

                        // Get RAW THERMOGRAPHY DATA
                        seekframe_t* thermal_frame = nullptr;
                        status = seekcamera_frame_get_frame_by_format(renderer->frame,
                            SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT,
                            &thermal_frame);
                        if (status != SEEKCAMERA_SUCCESS) {
                            seekcamera_frame_unlock(renderer->frame);
                            continue;
                        }

                        const int frame_width = (int)seekframe_get_width(thermal_frame);
                        const int frame_height = (int)seekframe_get_height(thermal_frame);
                        float* temps = (float*)seekframe_get_data(thermal_frame);

                        if (renderer->texture == nullptr) {
                            const int scale_factor = 2;
                            int window_width = frame_width * scale_factor + SIDEBAR_WIDTH;
                            int window_height = frame_height * scale_factor;

                            SDL_RenderSetLogicalSize(renderer->renderer, window_width, window_height);
                            renderer->texture = SDL_CreateTexture(renderer->renderer, SDL_PIXELFORMAT_ARGB8888,
                                SDL_TEXTUREACCESS_STREAMING, frame_width, frame_height);
                            SDL_SetWindowSize(renderer->window, window_width, window_height);
                            SDL_ShowWindow(renderer->window);
                        }

                        // CUSTOM COLOR MAPPING - bypass hardware palette!
                        // Find min/max temps in frame
                        float min_temp = 1000.0f;
                        float max_temp = -1000.0f;
                        for (int i = 0; i < frame_width * frame_height; i++) {
                            min_temp = std::min(min_temp, temps[i]);
                            max_temp = std::max(max_temp, temps[i]);
                        }

                        // Create custom colored image based on mode
                        Uint32* pixels;
                        int pitch;
                        SDL_LockTexture(renderer->texture, nullptr, (void**)&pixels, &pitch);

                        for (int y = 0; y < frame_height; y++) {
                            for (int x = 0; x < frame_width; x++) {
                                float temp = temps[y * frame_width + x];

                                SDL_Color color;
                                if (renderer->color_mode == ABSOLUTE) {
                                    // ABSOLUTE: Fixed 20-70°C range
                                    color = mapTemperatureAbsolute(temp);
                                }
                                else {
                                    // RELATIVE: Frame min/max for maximum visibility
                                    color = mapTemperatureRelative(temp, min_temp, max_temp);
                                }

                                // Convert to ARGB8888
                                Uint32 pixel = (255 << 24) | (color.r << 16) | (color.g << 8) | color.b;
                                pixels[y * (pitch / 4) + x] = pixel;
                            }
                        }

                        SDL_UnlockTexture(renderer->texture);

                        int window_width, window_height;
                        SDL_GetWindowSize(renderer->window, &window_width, &window_height);

                        SDL_Rect thermal_rect;
                        thermal_rect.x = 0;
                        thermal_rect.y = 0;
                        thermal_rect.w = window_width - SIDEBAR_WIDTH;
                        thermal_rect.h = window_height;
                        SDL_RenderCopy(renderer->renderer, renderer->texture, nullptr, &thermal_rect);

                        const int scale_factor = 2;

                        int center_x_thermal = frame_width / 2;
                        int center_y_thermal = frame_height / 2;

                        for (size_t i = 0; i < renderer->detected_objects.size(); i++) {
                            drawThermalObject(renderer->renderer, renderer->font,
                                renderer->detected_objects[i], scale_factor);
                        }

                        drawCenterCrosshair(renderer->renderer, renderer->font,
                            window_width - SIDEBAR_WIDTH, window_height,
                            renderer->center_pixel_temp, renderer->user_temp_threshold);

                        drawSidebar(renderer->renderer, renderer->font, renderer->font_small,
                            renderer->detected_objects, renderer->center_pixel_temp,
                            center_x_thermal, center_y_thermal, renderer->user_temp_threshold,
                            renderer->color_mode, window_width, window_height,
                            renderer->mouse_x, renderer->mouse_y, renderer->mouse_down);

                        SDL_RenderPresent(renderer->renderer);

                        seekcamera_frame_unlock(renderer->frame);
                        renderer->is_dirty.store(false);
                        renderer->frame = nullptr;
                    }
                }
            }
        }
        else {
            event_lock.unlock();
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_m) {
                // Toggle color mode with 'M' key
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->color_mode = (it->second->color_mode == ABSOLUTE) ?
                            RELATIVE : ABSOLUTE;
                        std::cout << "Color mode: " <<
                            ((it->second->color_mode == ABSOLUTE) ? "ABSOLUTE" : "RELATIVE")
                            << std::endl;
                    }
                }
            }
            else if (event.type == SDL_MOUSEMOTION) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_x = event.motion.x;
                        it->second->mouse_y = event.motion.y;
                    }
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_down = true;
                    }
                }
            }
            else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                    it != g_renderers.end(); ++it) {
                    if (it->second != nullptr) {
                        it->second->mouse_down = false;
                    }
                }
            }
        }
    }

    std::cout << "Shutting down..." << std::endl;

    seekcamera_manager_destroy(&manager);

    for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
        it != g_renderers.end(); ++it) {
        if (it->second != nullptr) {
            seekrenderer_close_window(it->second);
            delete it->second;
        }
    }
    g_renderers.clear();

    TTF_Quit();
    SDL_Quit();

    std::cout << "Done" << std::endl;
    return 0;
}