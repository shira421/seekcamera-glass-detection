/**
 * @file capture_shape.cpp
 * @brief Tool to capture the PCB heat blob shape for template matching
 */

#include <SDL2/SDL.h>
#include <seekcamera/seekcamera.h>
#include <seekcamera/seekcamera_manager.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <atomic>

// Frame data
static std::mutex frame_mutex;
static float* temp_data = nullptr;
static int frame_width = 0;
static int frame_height = 0;
static std::atomic<bool> new_frame{false};

struct Point {
    float x, y;
};

struct CapturedShape {
    std::vector<Point> perimeter;  // Normalized perimeter points
    Point cold_center;              // (0, 0) after normalization
    float aspect_ratio;             // height / width
    bool valid;
};

// Find contour of hot region using marching squares simplified
std::vector<Point> findContour(float* temps, int width, int height, float threshold) {
    std::vector<Point> contour;
    
    // Create binary mask
    std::vector<bool> mask(width * height, false);
    for (int i = 0; i < width * height; i++) {
        mask[i] = temps[i] >= threshold;
    }
    
    // Find edge pixels (hot pixels adjacent to cold pixels)
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            if (!mask[y * width + x]) continue;
            
            // Check if this is an edge pixel
            bool is_edge = false;
            for (int dy = -1; dy <= 1 && !is_edge; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (!mask[(y + dy) * width + (x + dx)]) {
                        is_edge = true;
                        break;
                    }
                }
            }
            
            if (is_edge) {
                contour.push_back({static_cast<float>(x), static_cast<float>(y)});
            }
        }
    }
    
    return contour;
}

// Find cold circle center
Point findColdCenter(float* temps, int width, int height, 
                     const std::vector<Point>& contour, float threshold) {
    if (contour.empty()) return {0, 0};
    
    // Find bounding box of contour
    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    for (const auto& p : contour) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    // Search center region for coldest pixel
    float center_x = (min_x + max_x) / 2;
    float center_y = (min_y + max_y) / 2;
    float search_r = std::min(max_x - min_x, max_y - min_y) / 4;
    
    float coldest = 1e9f;
    int cold_x = static_cast<int>(center_x);
    int cold_y = static_cast<int>(center_y);
    
    for (int y = static_cast<int>(center_y - search_r); 
         y <= static_cast<int>(center_y + search_r); y++) {
        for (int x = static_cast<int>(center_x - search_r); 
             x <= static_cast<int>(center_x + search_r); x++) {
            if (x < 0 || x >= width || y < 0 || y >= height) continue;
            float t = temps[y * width + x];
            if (t < coldest) {
                coldest = t;
                cold_x = x;
                cold_y = y;
            }
        }
    }
    
    // Refine with centroid of cold pixels
    float cold_thresh = coldest + 2.0f;
    float sum_x = 0, sum_y = 0, sum_w = 0;
    
    for (int dy = -15; dy <= 15; dy++) {
        for (int dx = -15; dx <= 15; dx++) {
            int px = cold_x + dx, py = cold_y + dy;
            if (px < 0 || px >= width || py < 0 || py >= height) continue;
            float t = temps[py * width + px];
            if (t <= cold_thresh) {
                float w = cold_thresh - t + 0.1f;
                sum_x += px * w;
                sum_y += py * w;
                sum_w += w;
            }
        }
    }
    
    if (sum_w > 0) {
        return {sum_x / sum_w, sum_y / sum_w};
    }
    return {static_cast<float>(cold_x), static_cast<float>(cold_y)};
}

// Normalize shape: center at cold circle, scale width to 1.0
CapturedShape normalizeShape(const std::vector<Point>& contour, Point cold_center) {
    CapturedShape shape;
    shape.valid = false;
    
    if (contour.size() < 20) return shape;
    
    // Find bounding box
    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    for (const auto& p : contour) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    float width = max_x - min_x;
    float height = max_y - min_y;
    
    if (width < 10) return shape;
    
    shape.aspect_ratio = height / width;
    shape.cold_center = {0, 0};  // Normalized to origin
    
    // Normalize all points
    for (const auto& p : contour) {
        Point normalized;
        normalized.x = (p.x - cold_center.x) / width;
        normalized.y = (p.y - cold_center.y) / width;  // Use width for both to preserve aspect
        shape.perimeter.push_back(normalized);
    }
    
    // Sort by angle for ordered perimeter
    std::sort(shape.perimeter.begin(), shape.perimeter.end(), 
        [](const Point& a, const Point& b) {
            return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);
        });
    
    // Subsample to reduce points (keep ~100 points)
    if (shape.perimeter.size() > 100) {
        std::vector<Point> subsampled;
        int step = shape.perimeter.size() / 100;
        for (size_t i = 0; i < shape.perimeter.size(); i += step) {
            subsampled.push_back(shape.perimeter[i]);
        }
        shape.perimeter = subsampled;
    }
    
    shape.valid = true;
    return shape;
}

// Save shape to header file
void saveShapeHeader(const CapturedShape& shape, const std::string& filename) {
    std::ofstream file(filename);
    
    file << "/**\n";
    file << " * @file pcb_shape_template.h\n";
    file << " * @brief PCB shape template for matching\n";
    file << " * Auto-generated by capture_shape tool\n";
    file << " */\n\n";
    file << "#pragma once\n\n";
    file << "#include <vector>\n\n";
    file << "namespace thermal {\n\n";
    file << "struct ShapePoint {\n";
    file << "    float x, y;\n";
    file << "};\n\n";
    file << "// PCB shape template - normalized with cold center at (0,0), width = 1.0\n";
    file << "static const float PCB_ASPECT_RATIO = " << shape.aspect_ratio << "f;\n\n";
    file << "static const std::vector<ShapePoint> PCB_PERIMETER = {\n";
    
    for (size_t i = 0; i < shape.perimeter.size(); i++) {
        file << "    {" << shape.perimeter[i].x << "f, " << shape.perimeter[i].y << "f}";
        if (i < shape.perimeter.size() - 1) file << ",";
        file << "\n";
    }
    
    file << "};\n\n";
    file << "} // namespace thermal\n";
    
    file.close();
    std::cout << "Saved shape template to " << filename << std::endl;
    std::cout << "Points: " << shape.perimeter.size() << std::endl;
    std::cout << "Aspect ratio: " << shape.aspect_ratio << std::endl;
}

// Camera callback
void frameCallback(seekcamera_t* camera, seekcamera_frame_t* frame, void* user_data) {
    seekcamera_frame_lock(frame, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, 
                          (void**)&temp_data, nullptr, nullptr, nullptr);
    if (!temp_data) return;
    
    size_t w, h;
    seekcamera_frame_get_width(frame, &w);
    seekcamera_frame_get_height(frame, &h);
    
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frame_width = static_cast<int>(w);
        frame_height = static_cast<int>(h);
        new_frame = true;
    }
}

void connectCallback(seekcamera_t* camera, seekcamera_error_t status, void* user_data) {
    if (status == SEEKCAMERA_SUCCESS) {
        seekcamera_register_frame_available_callback(camera, frameCallback, nullptr);
        seekcamera_capture_session_start(camera, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT);
        std::cout << "Camera connected!" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "PCB Shape Capture Tool\n";
    std::cout << "======================\n";
    std::cout << "Position the PCB in view, then:\n";
    std::cout << "  Press 'C' to capture shape\n";
    std::cout << "  Press 'Q' to quit\n\n";
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL init failed\n";
        return 1;
    }
    
    SDL_Window* window = SDL_CreateWindow("Shape Capture",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        640, 480, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    
    // Initialize camera
    seekcamera_manager_t* manager = nullptr;
    seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB);
    seekcamera_manager_register_event_callback(manager, connectCallback, nullptr);
    
    bool running = true;
    SDL_Event event;
    
    std::vector<Point> current_contour;
    Point current_cold_center = {0, 0};
    
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_q) {
                    running = false;
                } else if (event.key.keysym.sym == SDLK_c) {
                    // Capture current shape
                    if (!current_contour.empty()) {
                        CapturedShape shape = normalizeShape(current_contour, current_cold_center);
                        if (shape.valid) {
                            saveShapeHeader(shape, "pcb_shape_template.h");
                            std::cout << "\nShape captured successfully!\n";
                        } else {
                            std::cout << "\nFailed to capture shape - not enough contour points\n";
                        }
                    }
                }
            }
        }
        
        // Process frame
        if (new_frame) {
            std::lock_guard<std::mutex> lock(frame_mutex);
            new_frame = false;
            
            if (temp_data && frame_width > 0 && frame_height > 0) {
                // Find temperature range
                float min_t = temp_data[0], max_t = temp_data[0];
                float avg_t = 0;
                for (int i = 0; i < frame_width * frame_height; i++) {
                    min_t = std::min(min_t, temp_data[i]);
                    max_t = std::max(max_t, temp_data[i]);
                    avg_t += temp_data[i];
                }
                avg_t /= (frame_width * frame_height);
                
                float threshold = avg_t + (max_t - avg_t) * 0.15f;
                
                // Find contour
                current_contour = findContour(temp_data, frame_width, frame_height, threshold);
                current_cold_center = findColdCenter(temp_data, frame_width, frame_height, 
                                                      current_contour, threshold);
                
                // Render
                SDL_SetRenderDrawColor(renderer, 0, 0, 30, 255);
                SDL_RenderClear(renderer);
                
                // Draw thermal image (simple grayscale)
                float scale_x = 640.0f / frame_width;
                float scale_y = 480.0f / frame_height;
                
                for (int y = 0; y < frame_height; y++) {
                    for (int x = 0; x < frame_width; x++) {
                        float t = temp_data[y * frame_width + x];
                        float norm = (t - min_t) / (max_t - min_t + 0.1f);
                        Uint8 c = static_cast<Uint8>(norm * 255);
                        
                        SDL_SetRenderDrawColor(renderer, c, c/2, c/3, 255);
                        SDL_Rect rect = {
                            static_cast<int>(x * scale_x),
                            static_cast<int>(y * scale_y),
                            static_cast<int>(scale_x) + 1,
                            static_cast<int>(scale_y) + 1
                        };
                        SDL_RenderFillRect(renderer, &rect);
                    }
                }
                
                // Draw contour (green)
                SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
                for (const auto& p : current_contour) {
                    int px = static_cast<int>(p.x * scale_x);
                    int py = static_cast<int>(p.y * scale_y);
                    SDL_RenderDrawPoint(renderer, px, py);
                }
                
                // Draw cold center (red cross)
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                int cx = static_cast<int>(current_cold_center.x * scale_x);
                int cy = static_cast<int>(current_cold_center.y * scale_y);
                SDL_RenderDrawLine(renderer, cx - 10, cy, cx + 10, cy);
                SDL_RenderDrawLine(renderer, cx, cy - 10, cx, cy + 10);
                
                // Info text
                std::cout << "\rContour points: " << current_contour.size() 
                          << " Cold center: (" << current_cold_center.x 
                          << ", " << current_cold_center.y << ")     " << std::flush;
                
                SDL_RenderPresent(renderer);
            }
        }
        
        SDL_Delay(30);
    }
    
    seekcamera_manager_destroy(&manager);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return 0;
}