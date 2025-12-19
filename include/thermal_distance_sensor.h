/**
 * @file thermal_distance_sensor.h
 * @brief Thermal Distance Sensor - Dual Emitter Triangulation System
 * 
 * This system uses a thermal camera and two heat emitters to measure distance
 * to a reflective surface (glass) using triangulation. The emitters are positioned
 * at known offsets from the camera, and their reflections appear as hot spots
 * in the thermal image. The pixel separation between spots is inversely
 * proportional to distance.
 * 
 * @author EE2026
 * @version 1.0.0
 * @date 2024
 * 
 * Hardware Requirements:
 * - Seek Thermal SD314SPX Drone Core (320x240, 56° HFOV)
 * - Two heat emitters positioned at known offsets from camera
 * - Reflective surface (glass) for measurement
 * 
 * Dependencies:
 * - Seek Thermal SDK
 * - SDL2 and SDL2_ttf
 * - C++11 or later
 */

#ifndef THERMAL_DISTANCE_SENSOR_H
#define THERMAL_DISTANCE_SENSOR_H

#include <vector>
#include <cmath>
#include <algorithm>

#if defined(__linux__) || defined(__APPLE__)
#   include <SDL2/SDL.h>
#elif defined(_WIN32)
#   define SDL_MAIN_HANDLED
#   include <SDL.h>
#endif

namespace thermal {

//=============================================================================
// CONFIGURATION CONSTANTS
//=============================================================================

/** @brief Top percentage of temperatures shown in isolation mode */
constexpr float ISOLATION_THRESHOLD = 0.15f;

/** @brief Width of the UI sidebar in pixels */
constexpr int SIDEBAR_WIDTH = 280;

//=============================================================================
// DATA STRUCTURES
//=============================================================================

/**
 * @brief Represents a detected thermal region in the image
 */
struct ThermalObject {
    int id;                     ///< Unique identifier
    int x, y;                   ///< Top-left corner of bounding box
    int width, height;          ///< Bounding box dimensions
    float min_temp;             ///< Minimum temperature in region (°C)
    float max_temp;             ///< Maximum temperature in region (°C)
    float avg_temp;             ///< Average temperature in region (°C)
    SDL_Color box_color;        ///< Display color based on confidence
};

/**
 * @brief Result of distance calculation
 */
struct DistanceResult {
    bool detected;              ///< Whether two valid spots were found
    float distance_cm;          ///< Calculated distance to surface (cm)
    float camera_yaw_deg;       ///< Camera horizontal angle (+ = right)
    float camera_pitch_deg;     ///< Camera vertical angle (+ = up)
    
    int spot1_x, spot1_y;       ///< ABOVE emitter spot position (pixels)
    int spot2_x, spot2_y;       ///< LEFT emitter spot position (pixels)
    float spot1_temp;           ///< ABOVE spot temperature (°C)
    float spot2_temp;           ///< LEFT spot temperature (°C)
};

/**
 * @brief Score components for heat signature analysis
 */
struct SignatureScore {
    float gradient_quality;     ///< Radial temperature falloff quality (0-1)
    float compactness;          ///< Size appropriateness score (0-1)
    float peak_centrality;      ///< How centered is the peak (0-1)
    float aspect_ratio_score;   ///< Shape regularity score (0-1)
    float circularity;          ///< Roundness score (0-1)
    float overall_score;        ///< Weighted combined score (0-1)
};

//=============================================================================
// TRIANGULATION DISTANCE SENSOR CLASS
//=============================================================================

/**
 * @brief Main class for thermal triangulation distance measurement
 * 
 * This class implements a dual-emitter triangulation system for measuring
 * distance to reflective surfaces using thermal imaging. It handles:
 * - Hot spot detection with sub-pixel accuracy
 * - Spot identification (which emitter each spot corresponds to)
 * - Distance calculation from pixel separation
 * - Angle calculation for camera orientation
 * - Temporal filtering with bias correction for stable output
 * 
 * Physical Setup:
 * ```
 *     [ABOVE EMITTER] ← 2.5cm above camera
 *            |
 *            | 2.5cm
 *            |
 *     [CAMERA]-------- 3.3cm --------[LEFT EMITTER]
 * ```
 * 
 * The emitters create two hot spots visible in the thermal image via
 * reflection off glass. The pixel separation between spots is inversely
 * proportional to distance: closer = more separation, farther = less.
 * 
 * Usage:
 * @code
 * TriangulationDistanceSensor sensor;
 * 
 * // In frame callback:
 * float* temps = get_temperature_data();
 * DistanceResult result = sensor.calculate(temps, 320, 240);
 * 
 * if (result.detected) {
 *     printf("Distance: %.2f cm\n", result.distance_cm);
 *     printf("Yaw: %.2f°, Pitch: %.2f°\n", 
 *            result.camera_yaw_deg, result.camera_pitch_deg);
 * }
 * @endcode
 */
class TriangulationDistanceSensor {
public:
    /**
     * @brief Construct a new sensor with default physical parameters
     * 
     * Initializes filter state and calculates derived values (VFOV, focal length)
     * from the physical constants.
     */
    TriangulationDistanceSensor();
    
    /**
     * @brief Calculate distance and orientation from thermal frame
     * 
     * This is the main entry point for distance measurement. It:
     * 1. Finds the two hottest local maxima in the image
     * 2. Calculates sub-pixel centroids using Gaussian-weighted averaging
     * 3. Identifies which spot corresponds to which emitter
     * 4. Calculates distance from pixel separation
     * 5. Calculates camera yaw/pitch from spot positions
     * 6. Applies temporal filtering with bias correction
     * 
     * @param temps Temperature array in row-major order (°C)
     * @param width Image width in pixels (typically 320)
     * @param height Image height in pixels (typically 240)
     * @return DistanceResult containing distance, angles, and spot info
     */
    DistanceResult calculate(float* temps, int width, int height);

private:
    //-------------------------------------------------------------------------
    // Physical Setup Constants
    //-------------------------------------------------------------------------
    
    /** @brief Vertical offset of ABOVE emitter from camera (cm) */
    static constexpr float BASELINE_ABOVE_CM = 2.5f;
    
    /** @brief Horizontal offset of LEFT emitter from camera (cm) */
    static constexpr float BASELINE_LEFT_CM = 3.3f;
    
    /** @brief Camera horizontal field of view (degrees) */
    static constexpr float HFOV_DEGREES = 56.0f;
    
    /** @brief Thermal image width (pixels) */
    static constexpr float SENSOR_WIDTH = 320.0f;
    
    /** @brief Thermal image height (pixels) */
    static constexpr float SENSOR_HEIGHT = 240.0f;
    
    /** @brief Calibration constant: pixels × cm at reference distance */
    static constexpr float K_CALIBRATION = 1335.0f;
    
    //-------------------------------------------------------------------------
    // Derived Values (calculated in constructor)
    //-------------------------------------------------------------------------
    
    float vfov_degrees_;        ///< Vertical FOV calculated from HFOV
    float focal_length_pixels_; ///< Focal length in pixel units
    
    //-------------------------------------------------------------------------
    // Filter State
    //-------------------------------------------------------------------------
    
    float filtered_dist_, filtered_yaw_, filtered_pitch_;
    float filtered_spot1_x_, filtered_spot1_y_;
    float filtered_spot2_x_, filtered_spot2_y_;
    bool filter_initialized_;
    
    float prev_dist_, prev_yaw_, prev_pitch_;
    float prev_spot1_x_, prev_spot1_y_;
    float prev_spot2_x_, prev_spot2_y_;
    
    //-------------------------------------------------------------------------
    // Median Filter Buffers (for bias correction)
    //-------------------------------------------------------------------------
    
    std::vector<float> dist_buffer_;
    std::vector<float> yaw_buffer_;
    std::vector<float> pitch_buffer_;
    static constexpr size_t MEDIAN_WINDOW = 21;
    
    //-------------------------------------------------------------------------
    // Private Helper Structures
    //-------------------------------------------------------------------------
    
    /** @brief Internal representation of a detected hot spot */
    struct HotSpot {
        float x, y;     ///< Sub-pixel position
        float temp;     ///< Peak temperature
        bool valid;     ///< Whether detection was successful
    };
    
    //-------------------------------------------------------------------------
    // Private Methods
    //-------------------------------------------------------------------------
    
    /**
     * @brief Calculate median of buffer with new value added
     * @param buffer Rolling buffer of values
     * @param new_val New value to add
     * @return Median of buffer contents
     */
    float getMedian(std::vector<float>& buffer, float new_val);
    
    /**
     * @brief Find sub-pixel centroid of hot spot using Gaussian weighting
     * 
     * Uses a 7x7 window with Gaussian spatial weights and cubic temperature
     * weighting to calculate a precise centroid position.
     * 
     * @param temps Temperature array
     * @param width Image width
     * @param height Image height
     * @param center_x Seed X position (integer pixel)
     * @param center_y Seed Y position (integer pixel)
     * @param max_temp Peak temperature at seed
     * @return HotSpot with sub-pixel position
     */
    HotSpot findHotSpotCentroid(float* temps, int width, int height,
                                 int center_x, int center_y, float max_temp);
};

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/**
 * @brief Convert temperature to display color
 * 
 * Maps temperature values to a Blue→Cyan→Green→Yellow→Red gradient.
 * In isolation mode, only the top 15% of temperatures are colored;
 * everything else is black.
 * 
 * @param temp Temperature value (°C)
 * @param min_temp Minimum temperature in frame (°C)
 * @param max_temp Maximum temperature in frame (°C)
 * @param isolation_mode If true, only show top 15% of range
 * @return SDL_Color for display
 */
SDL_Color mapTemperature(float temp, float min_temp, float max_temp, 
                         bool isolation_mode);

/**
 * @brief Find connected components in binary image using flood fill
 * 
 * @param binary_map 2D boolean array of hot pixels
 * @param width Image width
 * @param height Image height
 * @return 2D array of component labels (0 = background)
 */
std::vector<std::vector<int>> findConnectedComponents(
    std::vector<std::vector<bool>>& binary_map, int width, int height);

/**
 * @brief Analyze thermal object to score signature quality
 * 
 * Evaluates gradient quality, peak centrality, aspect ratio,
 * circularity, and compactness to determine if object is a
 * valid heat signature vs noise.
 * 
 * @param temps Temperature array
 * @param width Image width
 * @param height Image height
 * @param obj Object to analyze
 * @return SignatureScore with component and overall scores
 */
SignatureScore analyzeSignaturePattern(float* temps, int width, int height,
                                       const ThermalObject& obj);

/**
 * @brief Detect thermal objects in frame
 * 
 * Full pipeline: threshold → connected components → scoring → top 2
 * 
 * @param temps Temperature array
 * @param width Image width
 * @param height Image height
 * @param next_id Reference to ID counter (incremented for each new object)
 * @return Vector of detected ThermalObjects (max 2)
 */
std::vector<ThermalObject> detectThermalObjects(float* temps, int width, 
                                                 int height, int& next_id);

} // namespace thermal

#endif // THERMAL_DISTANCE_SENSOR_H