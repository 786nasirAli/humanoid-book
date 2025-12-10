// LiDAR Noise Model Implementation
// Simulates realistic noise characteristics for LiDAR sensors

#include <random>
#include <vector>
#include <cmath>
#include <iostream>

class LidarNoiseModel {
public:
    LidarNoiseModel(double mean_noise = 0.0, double stddev_noise = 0.01, 
                    double bias = 0.0, double drift_rate = 0.0) 
        : mean_noise_(mean_noise), stddev_noise_(stddev_noise), 
          bias_(bias), drift_rate_(drift_rate) {
        
        // Initialize random number generator
        random_device_ = std::random_device{}();
        rng_.seed(random_device_);
        normal_dist_ = std::normal_distribution<double>(mean_noise_, stddev_noise_);
    }
    
    // Apply noise to a single distance measurement
    double addNoise(double true_distance) {
        // Generate Gaussian noise
        double noise = normal_dist_(rng_);
        
        // Add bias that may drift over time
        bias_drift_ += drift_rate_ * generateRandom(-1.0, 1.0);
        double total_bias = bias_ + bias_drift_;
        
        // Apply noise and bias to the true distance
        double noisy_distance = true_distance + noise + total_bias;
        
        // Ensure distance is positive
        if (noisy_distance < 0) {
            noisy_distance = 0;
        }
        
        return noisy_distance;
    }
    
    // Apply noise to a complete scan (vector of distances)
    std::vector<double> addNoiseToScan(const std::vector<double>& true_scan) {
        std::vector<double> noisy_scan;
        noisy_scan.reserve(true_scan.size());
        
        for (double distance : true_scan) {
            noisy_scan.push_back(addNoise(distance));
        }
        
        return noisy_scan;
    }
    
    // Calculate signal-to-noise ratio for a measurement
    double calculateSNR(double true_distance) {
        if (stddev_noise_ == 0) return std::numeric_limits<double>::infinity();
        
        double signal_power = true_distance * true_distance;
        double noise_power = stddev_noise_ * stddev_noise_;
        
        // SNR in linear scale
        double snr_linear = signal_power / noise_power;
        
        // Convert to dB
        return 10 * std::log10(snr_linear);
    }
    
    // Get noise statistics
    struct NoiseStats {
        double mean;
        double stddev;
        double min_range;
        double max_range;
        double bias;
    };
    
    NoiseStats getNoiseStats() const {
        return {mean_noise_, stddev_noise_, 0.1, 30.0, bias_};
    }
    
    // Update parameters during runtime
    void updateParameters(double new_stddev) {
        stddev_noise_ = new_stddev;
        normal_dist_ = std::normal_distribution<double>(mean_noise_, stddev_noise_);
    }

private:
    // Generate random number in range [min, max]
    double generateRandom(double min_val, double max_val) {
        std::uniform_real_distribution<double> uniform_dist(min_val, max_val);
        return uniform_dist(rng_);
    }
    
    // Noise parameters
    double mean_noise_;
    double stddev_noise_;
    double bias_;
    double drift_rate_;
    double bias_drift_ = 0.0;
    
    // Random number generation
    std::random_device::result_type random_device_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
};

// Example usage function
void exampleUsage() {
    std::cout << "LiDAR Noise Model Example" << std::endl;
    
    // Create a noise model with typical parameters for a 2D LiDAR
    LidarNoiseModel lidar_model(0.0, 0.01, 0.0, 0.0001);  // 1cm stddev, slight drift
    
    // Simulate a measurement
    double true_distance = 5.0;  // meters
    double noisy_distance = lidar_model.addNoise(true_distance);
    
    std::cout << "True distance: " << true_distance << "m" << std::endl;
    std::cout << "Noisy distance: " << noisy_distance << "m" << std::endl;
    std::cout << "Error: " << (noisy_distance - true_distance) << "m" << std::endl;
    
    // Simulate a complete scan
    std::vector<double> true_scan = {1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 3.0, 2.0, 1.0};
    std::vector<double> noisy_scan = lidar_model.addNoiseToScan(true_scan);
    
    std::cout << "\nFull scan simulation:" << std::endl;
    for (size_t i = 0; i < true_scan.size(); ++i) {
        std::cout << "Angle " << i*20 << "°: " << true_scan[i] 
                  << "m -> " << noisy_scan[i] << "m" << std::endl;
    }
    
    // Show noise statistics
    auto stats = lidar_model.getNoiseStats();
    std::cout << "\nNoise Statistics:" << std::endl;
    std::cout << "Mean: " << stats.mean << std::endl;
    std::cout << "Std Dev: " << stats.stddev << std::endl;
    std::cout << "Bias: " << stats.bias << std::endl;
}

// Main function for testing purposes
int main() {
    exampleUsage();
    return 0;
}