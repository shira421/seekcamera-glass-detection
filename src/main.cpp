/**
 * @file main.cpp
 * @brief Main application for PCB grid thermal distance sensor
 */

#include "thermal_distance_sensor.h"
#include "rendering.h"

#include <csignal>
#include <cstdio>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>

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

struct seekrenderer_t {
    seekcamera_t* camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
    TTF_Font* font_large;
    std::mutex mtx;
    std::atomic<bool> is_active;
    std::atomic<bool> is_dirty;
    seekcamera_frame_t* frame;
    bool isolation_mode;

    seekrenderer_t()
        : camera(NULL)
        , window(NULL)
        , renderer(NULL)
        , texture(NULL)
        , font(NULL)
        , font_large(NULL)
        , frame(NULL)
        , isolation_mode(false)  // Start with isolation OFF
    {
        is_active.store(false);
        is_dirty.store(false);
    }
};

static std::atomic<bool> g_exit(false);
static std::map<std::string, seekrenderer_t*> g_renderers;
static std::mutex g_mutex;
static std::condition_variable g_cond;
static std::atomic<bool> g_dirty(false);

void signal_handler(int) {
    g_exit.store(true);
}

void on_frame(seekcamera_t*, seekcamera_frame_t* frame, void* user) {
    seekrenderer_t* r = static_cast<seekrenderer_t*>(user);
    if (!r) return;
    
    std::lock_guard<std::mutex> lock(r->mtx);
    
    seekcamera_frame_lock(frame);
    r->frame = frame;
    r->is_dirty.store(true);
    
    g_dirty.store(true);
    g_cond.notify_one();
}

void on_event(seekcamera_t* camera, seekcamera_manager_event_t event, 
              seekcamera_error_t, void*) {
    
    seekcamera_chipid_t cid;
    seekcamera_get_chipid(camera, &cid);
    std::string id(cid);
    
    switch (event) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT: {
            std::cout << "Camera connected: " << id << std::endl;
            
            seekrenderer_t* r = new seekrenderer_t();
            r->camera = camera;
            r->is_active.store(true);
            
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_renderers[id] = r;
            }
            
            seekcamera_register_frame_available_callback(camera, on_frame, r);
            seekcamera_capture_session_start(camera, 
                SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | 
                SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT);
            break;
        }
        
        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT: {
            std::cout << "Camera disconnected: " << id << std::endl;
            
            std::lock_guard<std::mutex> lock(g_mutex);
            if (g_renderers.count(id)) {
                seekrenderer_t* r = g_renderers[id];
                r->is_active.store(false);
                seekcamera_capture_session_stop(camera);
            }
            break;
        }
        
        case SEEKCAMERA_MANAGER_EVENT_ERROR:
            std::cerr << "Camera error: " << id << std::endl;
            break;
            
        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            std::cout << "Pairing camera: " << id << std::endl;
            seekcamera_store_calibration_data(camera, NULL, NULL, NULL);
            break;
    }
}

void cleanup_renderer(seekrenderer_t* r) {
    if (!r) return;
    if (r->texture) SDL_DestroyTexture(r->texture);
    if (r->renderer) SDL_DestroyRenderer(r->renderer);
    if (r->window) SDL_DestroyWindow(r->window);
    if (r->font) TTF_CloseFont(r->font);
    if (r->font_large) TTF_CloseFont(r->font_large);
    delete r;
}

int main(int, char**) {
    std::cout << "PCB Grid Thermal Distance Sensor" << std::endl;
    std::cout << "Press I to toggle isolation mode" << std::endl;
    std::cout << "Press Q to quit" << std::endl;
    std::cout << std::endl;
    
    signal(SIGINT, signal_handler);
    
    // Initialize SDL
    SDL_SetMainReady();
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return 1;
    }
    
    if (TTF_Init() < 0) {
        std::cerr << "TTF_Init failed: " << TTF_GetError() << std::endl;
        return 1;
    }
    
    // Create camera manager
    seekcamera_manager_t* manager = NULL;
    if (seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager" << std::endl;
        return 1;
    }
    seekcamera_manager_register_event_callback(manager, on_event, NULL);
    
    // Create sensor instance
    PCBGridSensor sensor;
    
    // Rate limiting for terminal output
    auto last_print = std::chrono::steady_clock::now();
    const auto print_interval = std::chrono::milliseconds(200);
    
    while (!g_exit.load()) {
        std::unique_lock<std::mutex> lock(g_mutex);
        
        // Wait for new frame
        if (g_cond.wait_for(lock, std::chrono::milliseconds(150),
            []() { return g_dirty.load(); })) {
            g_dirty.store(false);
            
            // Process each camera
            for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                 it != g_renderers.end(); ++it) {
                
                seekrenderer_t* r = it->second;
                if (!r || !r->is_active.load()) continue;
                
                // Create window if needed
                if (!r->window) {
                    r->window = SDL_CreateWindow("Thermal Distance Sensor",
                        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                        SENSOR_WIDTH * 2 + SIDEBAR_WIDTH, SENSOR_HEIGHT * 2,
                        SDL_WINDOW_SHOWN);
                    r->renderer = SDL_CreateRenderer(r->window, -1,
                        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
                    
                    // Load fonts
                    r->font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
                    if (!r->font) {
                        r->font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
                    }
                    r->font_large = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28);
                    if (!r->font_large) {
                        r->font_large = TTF_OpenFont("C:\\Windows\\Fonts\\arialbd.ttf", 28);
                    }
                }
                
                std::lock_guard<std::mutex> guard(r->mtx);
                
                if (!r->is_dirty.load() || !r->frame) continue;
                
                // Get thermal frame
                seekframe_t* thermal_frame = NULL;
                seekcamera_frame_get_frame_by_format(r->frame,
                    SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame);
                
                if (!thermal_frame) {
                    seekcamera_frame_unlock(r->frame);
                    r->is_dirty.store(false);
                    r->frame = NULL;
                    continue;
                }
                
                // Get dimensions
                int width = static_cast<int>(seekframe_get_width(thermal_frame));
                int height = static_cast<int>(seekframe_get_height(thermal_frame));
                
                // Get temperature data
                float* temps = static_cast<float*>(seekframe_get_data(thermal_frame));
                
                float min_t = temps[0];
                float max_t = temps[0];
                for (int i = 0; i < width * height; i++) {
                    min_t = std::min(min_t, temps[i]);
                    max_t = std::max(max_t, temps[i]);
                }
                
                static std::vector<float> sharpened_buffer;
                if (sharpened_buffer.size() != static_cast<size_t>(width * height)) {
                    sharpened_buffer.resize(width * height);
                }
                
                // Sharpening strength: higher = sharper edges but more noise
                const float SHARPEN_STRENGTH = 0.4f;
                sharpenFrame(temps, sharpened_buffer.data(), width, height, SHARPEN_STRENGTH);
                
                if (!r->texture) {
                    r->texture = SDL_CreateTexture(r->renderer,
                        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                        width, height);
                }
                
                // Map temperatures to colors with sharpening
                Uint32* pixels;
                int pitch;
                SDL_LockTexture(r->texture, NULL, reinterpret_cast<void**>(&pixels), &pitch);
                
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        int idx = y * width + x;
                        SDL_Color c = mapTemperatureSharp(temps[idx], sharpened_buffer[idx],
                                                          min_t, max_t, r->isolation_mode);
                        pixels[y * (pitch / 4) + x] = (255 << 24) | (c.r << 16) | 
                                                       (c.g << 8) | c.b;
                    }
                }
                SDL_UnlockTexture(r->texture);
                
                int ww, wh;
                SDL_GetWindowSize(r->window, &ww, &wh);
                
                SDL_Rect thermal_rect;
                thermal_rect.x = 0;
                thermal_rect.y = 0;
                thermal_rect.w = ww - SIDEBAR_WIDTH;
                thermal_rect.h = wh;
                SDL_RenderCopy(r->renderer, r->texture, NULL, &thermal_rect);
                
                const int scale = 2;  // Display is 2x thermal resolution
                
                GridResult result = sensor.detect(temps, width, height);
                
                drawCrosshair(r->renderer, ww - SIDEBAR_WIDTH, wh);
                drawGridDots(r->renderer, result, scale);
                drawGridBounds(r->renderer, result, scale);
                
                if (result.valid) {
                    auto now = std::chrono::steady_clock::now();
                    if (now - last_print >= print_interval) {
                        std::cout << "\rDist: " << std::fixed << std::setprecision(2)
                                  << result.distance_cm << " cm"
                                  << " | Yaw: " << std::setw(6) << result.yaw_deg << " deg"
                                  << " | Pitch: " << std::setw(6) << result.pitch_deg << " deg"
                                  << " | " << result.main_dots_detected << " dots"
                                  << " | " << result.num_main_rows << " rows"
                                  << "   " << std::flush;
                        last_print = now;
                    }
                }
                drawSidebar(r->renderer, r->font, r->font_large,
                            result, r->isolation_mode, ww, wh);
                
                // Present
                SDL_RenderPresent(r->renderer);
                
                // Unlock frame
                seekcamera_frame_unlock(r->frame);
                r->is_dirty.store(false);
                r->frame = NULL;
            }
        }
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    g_exit.store(true);
                    break;
                    
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_q) {
                        g_exit.store(true);
                    } else if (event.key.keysym.sym == SDLK_i) {
                        // Toggle isolation mode for all renderers
                        for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
                             it != g_renderers.end(); ++it) {
                            if (it->second) {
                                it->second->isolation_mode = !it->second->isolation_mode;
                                std::cout << "\nIsolation mode: " 
                                          << (it->second->isolation_mode ? "ON" : "OFF") 
                                          << std::endl;
                            }
                        }
                    }
                    break;
            }
        }
    }
    std::cout << std::endl << "Shutting down..." << std::endl;
    
    seekcamera_manager_destroy(&manager);
    
    for (std::map<std::string, seekrenderer_t*>::iterator it = g_renderers.begin();
         it != g_renderers.end(); ++it) {
        cleanup_renderer(it->second);
    }
    g_renderers.clear();
    
    TTF_Quit();
    SDL_Quit();
    
    return 0;
}