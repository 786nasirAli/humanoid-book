// IMU Noise Model Implementation
// Simulates realistic noise characteristics for IMU sensors

#include <random>
#include <vector>
#include <cmath>
#include <iostream>

struct Vector3 {
    double x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    
    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
};

class IMUNoiseModel {
public:
    IMUNoiseModel(double gyro_noise_density = 1.6e-4,      // rad/s/sqrt(Hz) - typical for MPU6050
                  double gyro_random_walk = 2.5e-5,        // rad/s^2/sqrt(Hz)
                  double accel_noise_density = 2.5e-3,     // m/s^2/sqrt(Hz)
                  double accel_random_walk = 3.6e-3,       // m/s^3/sqrt(Hz)
                  double gyro_bias_instability = 3.6e-5,   // rad/s
                  double accel_bias_instability = 3.6e-2   // m/s^2
                  ) 
        : gyro_noise_density_(gyro_noise_density),
          gyro_random_walk_(gyro_random_walk),
          accel_noise_density_(accel_noise_density),
          accel_random_walk_(accel_random_walk),
          gyro_bias_instability_(gyro_bias_instability),
          accel_bias_instability_(accel_bias_instability),
          dt_(0.01) {  // 100Hz default update rate
        
        // Initialize random number generators
        random_device_ = std::random_device{}();
        rng_.seed(random_device_);
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
        
        // Initialize bias random walks
        gyro_bias_rw_x_ = 0.0;
        gyro_bias_rw_y_ = 0.0;
        gyro_bias_rw_z_ = 0.0;
        accel_bias_rw_x_ = 0.0;
        accel_bias_rw_y_ = 0.0;
        accel_bias_rw_z_ = 0.0;
    }
    
    struct IMUReading {
        Vector3 gyro;      // Angular velocity (rad/s)
        Vector3 accel;     // Linear acceleration (m/s^2)
        Vector3 mag;       // Magnetic field (will be simulated separately)
        double temperature; // Temperature (not implemented in this simple model)
        
        IMUReading() : temperature(25.0) {}  // Default room temperature
    };
    
    // Apply noise to IMU reading
    IMUReading addNoise(const IMUReading& true_reading) {
        IMUReading noisy_reading;
        
        // Apply noise to gyroscope
        noisy_reading.gyro.x = true_reading.gyro.x + generateGyroNoise(0) + getGyroBiasX();
        noisy_reading.gyro.y = true_reading.gyro.y + generateGyroNoise(1) + getGyroBiasY();
        noisy_reading.gyro.z = true_reading.gyro.z + generateGyroNoise(2) + getGyroBiasZ();
        
        // Apply noise to accelerometer
        noisy_reading.accel.x = true_reading.accel.x + generateAccelNoise(0) + getAccelBiasX();
        noisy_reading.accel.y = true_reading.accel.y + generateAccelNoise(1) + getAccelBiasY();
        noisy_reading.accel.z = true_reading.accel.z + generateAccelNoise(2) + getAccelBiasZ();
        
        // Magnetic field (simplified - typically has different noise characteristics)
        noisy_reading.mag = true_reading.mag;
        
        // Temperature (assumed accurate for this simulation)
        noisy_reading.temperature = true_reading.temperature;
        
        return noisy_reading;
    }
    
    // Apply noise to a sequence of IMU readings
    std::vector<IMUReading> addNoiseToSequence(const std::vector<IMUReading>& true_sequence) {
        std::vector<IMUReading> noisy_sequence;
        noisy_sequence.reserve(true_sequence.size());
        
        for (const auto& reading : true_sequence) {
            noisy_sequence.push_back(addNoise(reading));
        }
        
        return noisy_sequence;
    }
    
    // Update the internal state (should be called at each time step)
    void update(double delta_t = 0.01) {  // Default 100Hz
        dt_ = delta_t;
        
        // Update random walk biases (these change slowly over time)
        updateGyroBias();
        updateAccelBias();
    }
    
    // Get noise statistics
    struct NoiseStats {
        double gyro_noise_density;
        double gyro_random_walk;
        double accel_noise_density;
        double accel_random_walk;
        double gyro_bias_instability;
        double accel_bias_instability;
        double update_rate;
    };
    
    NoiseStats getNoiseStats() const {
        return {gyro_noise_density_, gyro_random_walk_, accel_noise_density_, 
                accel_random_walk_, gyro_bias_instability_, accel_bias_instability_, 1.0/dt_};
    }
    
    // Reset bias random walks to initial state
    void resetBias() {
        gyro_bias_rw_x_ = 0.0;
        gyro_bias_rw_y_ = 0.0;
        gyro_bias_rw_z_ = 0.0;
        accel_bias_rw_x_ = 0.0;
        accel_bias_rw_y_ = 0.0;
        accel_bias_rw_z_ = 0.0;
    }
    
    // Update parameters during runtime
    void updateParameters(double new_gyro_nd, double new_accel_nd) {
        gyro_noise_density_ = new_gyro_nd;
        accel_noise_density_ = new_accel_nd;
    }

private:
    // Generate noise for gyro (axis 0, 1, 2 for x, y, z)
    double generateGyroNoise(int axis) {
        // Gyro noise based on noise density and update rate
        double noise = normal_dist_(rng_) * gyro_noise_density_ / std::sqrt(dt_);
        return noise;
    }
    
    // Generate noise for accelerometer (axis 0, 1, 2 for x, y, z)
    double generateAccelNoise(int axis) {
        // Accel noise based on noise density and update rate
        double noise = normal_dist_(rng_) * accel_noise_density_ / std::sqrt(dt_);
        return noise;
    }
    
    // Update gyro bias random walk
    void updateGyroBias() {
        gyro_bias_rw_x_ += normal_dist_(rng_) * gyro_random_walk_ * std::sqrt(dt_);
        gyro_bias_rw_y_ += normal_dist_(rng_) * gyro_random_walk_ * std::sqrt(dt_);
        gyro_bias_rw_z_ += normal_dist_(rng_) * gyro_random_walk_ * std::sqrt(dt_);
    }
    
    // Update accel bias random walk
    void updateAccelBias() {
        accel_bias_rw_x_ += normal_dist_(rng_) * accel_random_walk_ * std::sqrt(dt_);
        accel_bias_rw_y_ += normal_dist_(rng_) * accel_random_walk_ * std::sqrt(dt_);
        accel_bias_rw_z_ += normal_dist_(rng_) * accel_random_walk_ * std::sqrt(dt_);
    }
    
    // Get current bias values
    double getGyroBiasX() { return gyro_bias_rw_x_; }
    double getGyroBiasY() { return gyro_bias_rw_y_; }
    double getGyroBiasZ() { return gyro_bias_rw_z_; }
    
    double getAccelBiasX() { return accel_bias_rw_x_; }
    double getAccelBiasY() { return accel_bias_rw_y_; }
    double getAccelBiasZ() { return accel_bias_rw_z_; }
    
    // Noise parameters
    double gyro_noise_density_;
    double gyro_random_walk_;
    double accel_noise_density_;
    double accel_random_walk_;
    double gyro_bias_instability_;
    double accel_bias_instability_;
    
    // Current bias values
    double gyro_bias_rw_x_, gyro_bias_rw_y_, gyro_bias_rw_z_;
    double accel_bias_rw_x_, accel_bias_rw_y_, accel_bias_rw_z_;
    
    // Time step
    double dt_;
    
    // Random number generation
    std::random_device::result_type random_device_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
};

// Example usage function
void exampleUsage() {
    std::cout << "IMU Noise Model Example" << std::endl;
    
    // Create an IMU noise model with typical parameters
    IMUNoiseModel imu_model(1.6e-4, 2.5e-5, 2.5e-3, 3.6e-3, 3.6e-5, 3.6e-2);
    
    // Simulate a few IMU readings
    IMUNoiseModel::IMUReading true_reading;
    true_reading.gyro = Vector3(0.01, -0.02, 0.005);  // Small angular velocities
    true_reading.accel = Vector3(0.1, 0.2, 9.8);      // Small linear accels + gravity
    
    auto noisy_reading = imu_model.addNoise(true_reading);
    
    std::cout << "True gyro: (" << true_reading.gyro.x << ", " << true_reading.gyro.y 
              << ", " << true_reading.gyro.z << ") rad/s" << std::endl;
    std::cout << "Noisy gyro: (" << noisy_reading.gyro.x << ", " << noisy_reading.gyro.y 
              << ", " << noisy_reading.gyro.z << ") rad/s" << std::endl;
    
    std::cout << "True accel: (" << true_reading.accel.x << ", " << true_reading.accel.y 
              << ", " << true_reading.accel.z << ") m/s^2" << std::endl;
    std::cout << "Noisy accel: (" << noisy_reading.accel.x << ", " << noisy_reading.accel.y 
              << ", " << noisy_reading.accel.z << ") m/s^2" << std::endl;
    
    // Simulate a sequence of readings
    std::vector<IMUNoiseModel::IMUReading> true_sequence;
    for (int i = 0; i < 10; ++i) {
        IMUNoiseModel::IMUReading reading;
        reading.gyro = Vector3(0.001 * i, 0.002 * i, 0.0005 * i);
        reading.accel = Vector3(0.01 * i, 0.02 * i, 9.8);
        true_sequence.push_back(reading);
        
        // Update model to simulate time passing
        imu_model.update(0.01);
    }
    
    auto noisy_sequence = imu_model.addNoiseToSequence(true_sequence);
    
    std::cout << "\nFirst and last readings in sequence:" << std::endl;
    std::cout << "Initial - True Gyro: (" << true_sequence[0].gyro.x << ", " 
              << true_sequence[0].gyro.y << ", " << true_sequence[0].gyro.z << ")" << std::endl;
    std::cout << "Initial - Noisy Gyro: (" << noisy_sequence[0].gyro.x << ", " 
              << noisy_sequence[0].gyro.y << ", " << noisy_sequence[0].gyro.z << ")" << std::endl;
    
    std::cout << "Final - True Gyro: (" << true_sequence[9].gyro.x << ", " 
              << true_sequence[9].gyro.y << ", " << true_sequence[9].gyro.z << ")" << std::endl;
    std::cout << "Final - Noisy Gyro: (" << noisy_sequence[9].gyro.x << ", " 
              << noisy_sequence[9].gyro.y << ", " << noisy_sequence[9].gyro.z << ")" << std::endl;
    
    // Show noise statistics
    auto stats = imu_model.getNoiseStats();
    std::cout << "\nNoise Statistics:" << std::endl;
    std::cout << "Gyro noise density: " << stats.gyro_noise_density << " rad/s/sqrt(Hz)" << std::endl;
    std::cout << "Accel noise density: " << stats.accel_noise_density << " m/s²/sqrt(Hz)" << std::endl;
    std::cout << "Update rate: " << stats.update_rate << " Hz" << std::endl;
}

// Main function for testing purposes
int main() {
    exampleUsage();
    return 0;
}