/**
 * @file main.cpp
 * @brief Main application for thermal distance sensor
 * 
 * This is the entry point for the thermal distance measurement application.
 * It handles:
 * - SDL/TTF initialization
 * - Camera manager setup and event handling
 * - Main render loop
 * - Keyboard input processing
 */

#include "thermal_distance_sensor.h"
#include "rendering.h"

#include <csignal>
#include <cstring>
#include <cstdio>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <chrono>
#include <mutex>
#include <condition_variable>

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

using namespace thermal;

/**
 * @brief Per-camera renderer state
 */
struct seekrenderer_t {
    seekcamera_t* camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
    TTF_Font* font_large;
    std::mutex the_mutex;
    std::atomic<bool> is_active;
    std::atomic<bool> is_dirty;
    seekcamera_frame_t* frame;

    std::vector<ThermalObject> detected_objects;
    int next_object_id;
    float center_pixel_temp;
    bool isolation_mode;

    seekrenderer_t() 
        : camera(nullptr)
        , window(nullptr)
        , renderer(nullptr)
        , texture(nullptr)
        , font(nullptr)
        , font_large(nullptr)
        , frame(nullptr)
        , next_object_id(1)
        , center_pixel_temp(0.0f)
        , isolation_mode(true)
    {
        is_active.store(false);
        is_dirty.store(false);
    }
};

static std::atomic<bool> g_exit_requested;
static std::map<std::string, seekrenderer_t*> g_renderers;
static std::mutex g_mutex;
static std::condition_variable g_condition_variable;
static std::atomic<bool> g_is_dirty;

/**
 * @brief Called when a new frame is available from the camera
 */
void handle_camera_frame_available(seekcamera_t* camera, 
                                   seekcamera_frame_t* camera_frame, 
                                   void* user_data) {
    (void)camera;
    seekrenderer_t* renderer = static_cast<seekrenderer_t*>(user_data);

    std::lock_guard<std::mutex> guard(renderer->the_mutex);

    seekcamera_frame_lock(camera_frame);
    renderer->is_dirty.store(true);
    renderer->frame = camera_frame;

    // Get thermal data for object detection
    seekframe_t* thermal_frame = NULL;
    if (seekcamera_frame_get_frame_by_format(camera_frame,
        SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame) == SEEKCAMERA_SUCCESS) {

        float* temps = static_cast<float*>(seekframe_get_data(thermal_frame));
        int width = seekframe_get_width(thermal_frame);
        int height = seekframe_get_height(thermal_frame);

        if (temps && width > 0 && height > 0) {
            renderer->detected_objects = detectThermalObjects(
                temps, width, height, renderer->next_object_id);
            renderer->center_pixel_temp = temps[(height / 2) * width + (width / 2)];
        }
    }

    g_is_dirty.store(true);
    g_condition_variable.notify_one();
}

/**
 * @brief Called when a camera connects
 */
void handle_camera_connect(seekcamera_t* camera, 
                          seekcamera_error_t event_status, 
                          void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID(reinterpret_cast<char*>(&cid));

    seekrenderer_t* renderer = g_renderers[chipID] ? g_renderers[chipID] : new seekrenderer_t();
    renderer->is_active.store(true);
    renderer->camera = camera;

    seekcamera_register_frame_available_callback(camera, handle_camera_frame_available, renderer);
    seekcamera_set_color_palette(camera, SEEKCAMERA_COLOR_PALETTE_WHITE_HOT);
    
    uint32_t formats = SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | 
                       SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
    seekcamera_capture_session_start(camera, formats);

    g_renderers[chipID] = renderer;
    std::cout << "Camera connected" << std::endl;
}

/**
 * @brief Called when a camera disconnects
 */
void handle_camera_disconnect(seekcamera_t* camera, 
                             seekcamera_error_t event_status, 
                             void* user_data) {
    (void)event_status;
    (void)user_data;

    seekcamera_chipid_t cid;
    memset(&cid, 0, sizeof(cid));
    seekcamera_get_chipid(camera, &cid);
    std::string chipID(reinterpret_cast<char*>(&cid));
    
    seekcamera_capture_session_stop(camera);
    
    if (g_renderers[chipID]) {
        g_renderers[chipID]->is_active.store(false);
        g_is_dirty.store(true);
    }
    std::cout << "Camera disconnected" << std::endl;
}

/**
 * @brief Called when a camera is ready to pair
 */
void handle_camera_ready_to_pair(seekcamera_t* camera, 
                                 seekcamera_error_t event_status, 
                                 void* user_data) {
    seekcamera_store_calibration_data(camera, NULL, NULL, NULL);
    handle_camera_connect(camera, event_status, user_data);
}

/**
 * @brief Main camera event dispatcher
 */
void camera_event_callback(seekcamera_t* camera, 
                          seekcamera_manager_event_t event, 
                          seekcamera_error_t event_status, 
                          void* user_data) {
    switch (event) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT:
            handle_camera_connect(camera, event_status, user_data);
            break;
        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
            handle_camera_disconnect(camera, event_status, user_data);
            break;
        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            handle_camera_ready_to_pair(camera, event_status, user_data);
            break;
        default:
            break;
    }
}

/**
 * @brief Clean up renderer resources
 */
void cleanup_renderer(seekrenderer_t* renderer) {
    if (!renderer) return;
    
    if (renderer->is_active.load()) {
        seekcamera_capture_session_stop(renderer->camera);
    }
    renderer->is_active.store(false);
    
    if (renderer->font) {
        TTF_CloseFont(renderer->font);
        renderer->font = NULL;
    }
    if (renderer->font_large) {
        TTF_CloseFont(renderer->font_large);
        renderer->font_large = NULL;
    }
    if (renderer->texture) {
        SDL_DestroyTexture(renderer->texture);
        renderer->texture = NULL;
    }
    if (renderer->renderer) {
        SDL_DestroyRenderer(renderer->renderer);
        renderer->renderer = NULL;
    }
    if (renderer->window) {
        SDL_DestroyWindow(renderer->window);
        renderer->window = NULL;
    }
}

/**
 * @brief Signal handler for graceful shutdown
 */
static void signal_callback(int signum) {
    (void)signum;
    g_exit_requested.store(true);
}

int main() {
    g_exit_requested.store(false);
    g_is_dirty.store(false);

    // Register signal handlers
    signal(SIGINT, signal_callback);
    signal(SIGTERM, signal_callback);

    std::cout << "Thermal Distance Sensor - Dual Emitter Triangulation" << std::endl;
    std::cout << "Press I to toggle isolation mode" << std::endl;
    std::cout << "Press Q to quit" << std::endl << std::endl;

    // Initialize SDL and TTF
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    // Create camera manager
    seekcamera_manager_t* manager = NULL;
    if (seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager" << std::endl;
        return 1;
    }
    seekcamera_manager_register_event_callback(manager, camera_event_callback, NULL);

    // Create distance sensor instance
    TriangulationDistanceSensor distance_sensor;
    
    // Terminal output rate limiting
    std::chrono::steady_clock::time_point last_print = std::chrono::steady_clock::now();
    const std::chrono::milliseconds print_interval(200);
    
    while (!g_exit_requested.load()) {
        std::unique_lock<std::mutex> lock(g_mutex);

        // Wait for new frame with timeout
        if (g_condition_variable.wait_for(lock, std::chrono::milliseconds(150),
            []() { return g_is_dirty.load(); })) {
            g_is_dirty.store(false);

            // Process each connected camera
            for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                 it != g_renderers.end(); ++it) {
                seekrenderer_t* renderer = it->second;
                if (!renderer || !renderer->is_active.load()) continue;

                // Create window if needed
                if (!renderer->window) {
                    renderer->window = SDL_CreateWindow("Thermal Distance Sensor", 
                        100, 100, 0, 0, SDL_WINDOW_HIDDEN);
                    renderer->renderer = SDL_CreateRenderer(renderer->window, -1, 
                        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
                    
                    // Load fonts (try Linux paths first, then Windows)
                    renderer->font = TTF_OpenFont(
                        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
                    if (!renderer->font) {
                        renderer->font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
                    }
                    
                    renderer->font_large = TTF_OpenFont(
                        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28);
                    if (!renderer->font_large) {
                        renderer->font_large = TTF_OpenFont("C:\\Windows\\Fonts\\arialbd.ttf", 28);
                    }
                }

                std::lock_guard<std::mutex> guard(renderer->the_mutex);

                if (!renderer->is_dirty.load() || !renderer->frame) continue;

                // Get thermal frame
                seekframe_t* thermal_frame = NULL;
                if (seekcamera_frame_get_frame_by_format(renderer->frame,
                    SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame) != SEEKCAMERA_SUCCESS) {
                    seekcamera_frame_unlock(renderer->frame);
                    continue;
                }

                int fw = seekframe_get_width(thermal_frame);
                int fh = seekframe_get_height(thermal_frame);
                float* temps = static_cast<float*>(seekframe_get_data(thermal_frame));

                // Create texture if needed
                if (!renderer->texture) {
                    const int scale = 2;
                    int ww = fw * scale + SIDEBAR_WIDTH;
                    int wh = fh * scale;
                    
                    SDL_RenderSetLogicalSize(renderer->renderer, ww, wh);
                    renderer->texture = SDL_CreateTexture(renderer->renderer, 
                        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, fw, fh);
                    SDL_SetWindowSize(renderer->window, ww, wh);
                    SDL_ShowWindow(renderer->window);
                }

                // Find temperature range
                float min_t = 1000.0f, max_t = -1000.0f;
                for (int i = 0; i < fw * fh; i++) {
                    min_t = std::min(min_t, temps[i]);
                    max_t = std::max(max_t, temps[i]);
                }

                // Render thermal image to texture
                Uint32* pixels;
                int pitch;
                SDL_LockTexture(renderer->texture, NULL, 
                               reinterpret_cast<void**>(&pixels), &pitch);

                for (int y = 0; y < fh; y++) {
                    for (int x = 0; x < fw; x++) {
                        SDL_Color c = mapTemperature(temps[y * fw + x], min_t, max_t, 
                                                     renderer->isolation_mode);
                        pixels[y * (pitch / 4) + x] = (255 << 24) | (c.r << 16) | 
                                                      (c.g << 8) | c.b;
                    }
                }
                SDL_UnlockTexture(renderer->texture);

                // Get window size
                int ww, wh;
                SDL_GetWindowSize(renderer->window, &ww, &wh);

                // Draw thermal image
                SDL_Rect thermal_rect;
                thermal_rect.x = 0;
                thermal_rect.y = 0;
                thermal_rect.w = ww - SIDEBAR_WIDTH;
                thermal_rect.h = wh;
                SDL_RenderCopy(renderer->renderer, renderer->texture, NULL, &thermal_rect);

                const int scale = 2;
                
                // Draw center crosshair
                drawCrosshair(renderer->renderer, ww - SIDEBAR_WIDTH, wh);

                // Calculate distance and angles
                DistanceResult result = distance_sensor.calculate(temps, fw, fh);
                
                if (result.detected) {
                    // Draw spot markers
                    SDL_Color spot1_color;
                    spot1_color.r = 0; spot1_color.g = 255; spot1_color.b = 0; spot1_color.a = 255;
                    drawSpotMarker(renderer->renderer, renderer->font, 
                        result.spot1_x, result.spot1_y, result.spot1_temp,
                        spot1_color, scale, "ABOVE");
                    
                    SDL_Color spot2_color;
                    spot2_color.r = 0; spot2_color.g = 255; spot2_color.b = 255; spot2_color.a = 255;
                    drawSpotMarker(renderer->renderer, renderer->font,
                        result.spot2_x, result.spot2_y, result.spot2_temp,
                        spot2_color, scale, "LEFT");
                    
                    // Draw camera pointing indicator
                    drawCameraMarker(renderer->renderer, renderer->font,
                        result.spot1_x, result.spot1_y,
                        result.spot2_x, result.spot2_y, scale);
                    
                    // Terminal output (rate limited)
                    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                    if (now - last_print >= print_interval) {
                        std::cout << "\rDist: " << std::fixed << std::setprecision(2) 
                                  << result.distance_cm 
                                  << " cm | Yaw: " << std::setw(7) << result.camera_yaw_deg 
                                  << " deg | Pitch: " << std::setw(7) << result.camera_pitch_deg 
                                  << " deg   " << std::flush;
                        last_print = now;
                    }
                }

                // Draw sidebar UI
                drawSidebar(renderer->renderer, renderer->font, renderer->font_large,
                    result.detected, result.distance_cm, 
                    result.camera_yaw_deg, result.camera_pitch_deg,
                    result.spot1_temp, result.spot2_temp, renderer->isolation_mode,
                    ww, wh);

                // Present to screen
                SDL_RenderPresent(renderer->renderer);

                // Unlock frame
                seekcamera_frame_unlock(renderer->frame);
                renderer->is_dirty.store(false);
                renderer->frame = NULL;
            }
        }

        // Handle keyboard events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_WINDOWEVENT && 
                event.window.event == SDL_WINDOWEVENT_CLOSE) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q) {
                g_exit_requested.store(true);
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_i) {
                // Toggle isolation mode
                for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                     it != g_renderers.end(); ++it) {
                    if (it->second != NULL) {
                        it->second->isolation_mode = !it->second->isolation_mode;
                        std::cout << "\nIsolation mode: " 
                                  << (it->second->isolation_mode ? "ON" : "OFF") << std::endl;
                    }
                }
            }
        }
    }
    
    std::cout << std::endl << "Shutting down..." << std::endl;

    seekcamera_manager_destroy(&manager);

    for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
         it != g_renderers.end(); ++it) {
        cleanup_renderer(it->second);
        delete it->second;
    }
    g_renderers.clear();

    TTF_Quit();
    SDL_Quit();

    return 0;
}