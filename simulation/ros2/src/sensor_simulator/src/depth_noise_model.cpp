// Depth Camera Noise Model Implementation
// Simulates realistic noise characteristics for depth cameras

#include <random>
#include <vector>
#include <cmath>
#include <iostream>

class DepthCameraNoiseModel {
public:
    DepthCameraNoiseModel(double baseline_noise = 0.01,  // 1cm baseline noise at 1m
                          double sigma_random = 0.002,    // Random noise component
                          double sigma_bias = 0.001,      // Bias component
                          double focal_length = 525.0)    // Pixels (for Kinect-like camera)
        : baseline_noise_(baseline_noise), 
          sigma_random_(sigma_random), 
          sigma_bias_(sigma_bias),
          focal_length_(focal_length) {
        
        // Initialize random number generator
        random_device_ = std::random_device{}();
        rng_.seed(random_device_);
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);  // Standard normal
    }
    
    // Apply noise to a single depth measurement
    double addNoise(double true_depth, double pixel_distance_from_center = 0.0) {
        // Distance-dependent noise model
        // Noise increases with distance and with distance from optical center
        
        // Base noise that increases with depth squared (radial component)
        double radial_noise = baseline_noise_ * true_depth * true_depth;
        
        // Additional noise from distance from optical center
        double optical_center_noise = pixel_distance_from_center * 0.0001;  // Small effect
        
        // Total standard deviation
        double total_stddev = std::sqrt(radial_noise * radial_noise + 
                                     sigma_random_ * sigma_random_ + 
                                     optical_center_noise * optical_center_noise);
        
        // Generate noise using normal distribution
        double noise = normal_dist_(rng_) * total_stddev + sigma_bias_;
        
        // Apply noise to true depth
        double noisy_depth = true_depth + noise;
        
        // Ensure depth is positive and within reasonable bounds
        if (noisy_depth < 0.1) noisy_depth = 0.1;  // Minimum 10cm
        if (noisy_depth > 10.0) noisy_depth = 10.0;  // Maximum 10m for this sensor
        
        return noisy_depth;
    }
    
    // Apply noise to an entire depth image (represented as vector of depths)
    std::vector<std::vector<double>> addNoiseToImage(
        const std::vector<std::vector<double>>& true_depth_image,
        double fov_x = 1.0472,  // 60 degrees in radians
        double fov_y = 0.7854)  // 45 degrees in radians
    {
        int height = true_depth_image.size();
        int width = true_depth_image[0].size();
        
        std::vector<std::vector<double>> noisy_image(height, std::vector<double>(width));
        
        // Calculate center of image
        double center_x = width / 2.0;
        double center_y = height / 2.0;
        
        // Calculate pixel scale factors based on FOV
        double pixel_scale_x = fov_x / width;
        double pixel_scale_y = fov_y / height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double true_depth = true_depth_image[y][x];
                
                // Skip invalid depth values
                if (true_depth <= 0.0 || true_depth > 10.0) {
                    noisy_image[y][x] = true_depth;
                    continue;
                }
                
                // Calculate distance from optical center in radians
                double dx = (x - center_x) * pixel_scale_x;
                double dy = (y - center_y) * pixel_scale_y;
                double pixel_dist = std::sqrt(dx*dx + dy*dy);
                
                noisy_image[y][x] = addNoise(true_depth, pixel_dist);
            }
        }
        
        return noisy_image;
    }
    
    // Generate a point cloud with noise from depth image
    struct Point3D {
        double x, y, z;
    };
    
    std::vector<Point3D> depthImageToPointCloud(
        const std::vector<std::vector<double>>& depth_image,
        double fov_x = 1.0472,  // 60 degrees in radians
        double fov_y = 0.7854)  // 45 degrees in radians
    {
        int height = depth_image.size();
        int width = depth_image[0].size();
        
        std::vector<Point3D> point_cloud;
        point_cloud.reserve(height * width);
        
        // Calculate angular resolution
        double x_res = fov_x / width;
        double y_res = fov_y / height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double depth = depth_image[y][x];
                
                // Skip invalid depth values
                if (depth <= 0.0 || depth > 10.0) continue;
                
                // Calculate angles
                double angle_x = (x - width/2.0) * x_res;
                double angle_y = (y - height/2.0) * y_res;
                
                // Convert to 3D point
                Point3D point;
                point.z = depth * std::cos(angle_y) * std::cos(angle_x);
                point.x = depth * std::cos(angle_y) * std::sin(angle_x);
                point.y = depth * std::sin(angle_y);
                
                point_cloud.push_back(point);
            }
        }
        
        return point_cloud;
    }
    
    // Get noise statistics
    struct NoiseStats {
        double baseline_noise;
        double sigma_random;
        double sigma_bias;
        double min_range;
        double max_range;
        double focal_length;
    };
    
    NoiseStats getNoiseStats() const {
        return {baseline_noise_, sigma_random_, sigma_bias_, 0.1, 10.0, focal_length_};
    }
    
    // Update parameters during runtime
    void updateParameters(double new_baseline_noise, double new_random_sigma) {
        baseline_noise_ = new_baseline_noise;
        sigma_random_ = new_random_sigma;
    }

private:
    // Noise parameters
    double baseline_noise_;
    double sigma_random_;
    double sigma_bias_;
    double focal_length_;
    
    // Random number generation
    std::random_device::result_type random_device_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
};

// Example usage function
void exampleUsage() {
    std::cout << "Depth Camera Noise Model Example" << std::endl;
    
    // Create a noise model with typical parameters for a depth camera
    DepthCameraNoiseModel depth_model(0.01, 0.002, 0.001, 525.0);
    
    // Simulate a depth measurement
    double true_depth = 2.5;  // meters
    double noisy_depth = depth_model.addNoise(true_depth, 0.1);  // 0.1 radians from center
    
    std::cout << "True depth: " << true_depth << "m" << std::endl;
    std::cout << "Noisy depth: " << noisy_depth << "m" << std::endl;
    std::cout << "Error: " << (noisy_depth - true_depth) << "m" << std::endl;
    
    // Simulate a small depth image
    std::vector<std::vector<double>> true_image = {
        {1.0, 1.1, 1.0},
        {1.1, 1.2, 1.1},
        {1.0, 1.1, 1.0}
    };
    
    auto noisy_image = depth_model.addNoiseToImage(true_image);
    
    std::cout << "\n3x3 depth image with noise:" << std::endl;
    for (size_t y = 0; y < noisy_image.size(); ++y) {
        for (size_t x = 0; x < noisy_image[y].size(); ++x) {
            std::cout << noisy_image[y][x] << "\t";
        }
        std::cout << std::endl;
    }
    
    // Show noise statistics
    auto stats = depth_model.getNoiseStats();
    std::cout << "\nNoise Statistics:" << std::endl;
    std::cout << "Baseline noise: " << stats.baseline_noise << "m" << std::endl;
    std::cout << "Random sigma: " << stats.sigma_random << "m" << std::endl;
    std::cout << "Bias: " << stats.sigma_bias << "m" << std::endl;
    std::cout << "Range: " << stats.min_range << "m to " << stats.max_range << "m" << std::endl;
}

// Main function for testing purposes
int main() {
    exampleUsage();
    return 0;
}