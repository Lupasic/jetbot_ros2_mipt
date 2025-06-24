#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <thread>
#include <unistd.h>
#include "four_ch_motor_drive_comms.hpp"

struct PIDParams {
    float kp, ki, kd;
    
    PIDParams(float p = 0.0f, float i = 0.0f, float d = 0.0f) 
        : kp(p), ki(i), kd(d) {}
    
    std::string toString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) 
           << "P:" << kp << " I:" << ki << " D:" << kd;
        return ss.str();
    }
};

struct DataPoint {
    double timestamp;
    int target_speed_left, target_speed_right;
    double actual_speed_left, actual_speed_right;
    double position_left, position_right;
    double error_left, error_right;
};

struct TestResult {
    PIDParams params;
    double settling_time_left, settling_time_right;
    double overshoot_left, overshoot_right;
    double steady_state_error_left, steady_state_error_right;
    double rise_time_left, rise_time_right;
    double synchronization_error;
    double overall_score;
    std::vector<DataPoint> data;
    
    void calculateMetrics() {
        if (data.empty()) return;
        
        // Calculate metrics from data
        calculateSettlingTime();
        calculateOvershoot();
        calculateSteadyStateError();
        calculateRiseTime();
        calculateSynchronizationError();
        calculateOverallScore();
    }
    
private:
    void calculateSettlingTime() {
        // Find settling time (time to stay within 5% of final value)
        if (data.size() < 10) return;
        
        double final_speed_left = data.back().actual_speed_left;
        double final_speed_right = data.back().actual_speed_right;
        double tolerance = 0.05; // 5%
        
        settling_time_left = settling_time_right = data.back().timestamp;
        
        for (int i = data.size() - 1; i >= 0; --i) {
            if (std::abs(data[i].actual_speed_left - final_speed_left) > tolerance * std::abs(final_speed_left)) {
                settling_time_left = data[i].timestamp;
                break;
            }
        }
        
        for (int i = data.size() - 1; i >= 0; --i) {
            if (std::abs(data[i].actual_speed_right - final_speed_right) > tolerance * std::abs(final_speed_right)) {
                settling_time_right = data[i].timestamp;
                break;
            }
        }
    }
    
    void calculateOvershoot() {
        if (data.empty()) return;
        
        double target_left = data.back().target_speed_left;
        double target_right = data.back().target_speed_right;
        
        double max_left = 0, max_right = 0;
        for (const auto& point : data) {
            max_left = std::max(max_left, std::abs(point.actual_speed_left));
            max_right = std::max(max_right, std::abs(point.actual_speed_right));
        }
        
        overshoot_left = (std::abs(target_left) > 0) ? 
            std::max(0.0, (max_left - std::abs(target_left)) / std::abs(target_left)) : 0.0;
        overshoot_right = (std::abs(target_right) > 0) ? 
            std::max(0.0, (max_right - std::abs(target_right)) / std::abs(target_right)) : 0.0;
    }
    
    void calculateSteadyStateError() {
        if (data.size() < 10) return;
        
        // Average last 10% of data points for steady state
        int start_idx = std::max(0, (int)data.size() - (int)data.size()/10);
        double sum_error_left = 0, sum_error_right = 0;
        int count = 0;
        
        for (int i = start_idx; i < data.size(); ++i) {
            sum_error_left += std::abs(data[i].error_left);
            sum_error_right += std::abs(data[i].error_right);
            count++;
        }
        
        steady_state_error_left = (count > 0) ? sum_error_left / count : 0.0;
        steady_state_error_right = (count > 0) ? sum_error_right / count : 0.0;
    }
    
    void calculateRiseTime() {
        if (data.empty()) return;
        
        double target_left = std::abs(data.back().target_speed_left);
        double target_right = std::abs(data.back().target_speed_right);
        
        rise_time_left = rise_time_right = 0.0;
        
        // Find 10% and 90% points
        for (const auto& point : data) {
            if (std::abs(point.actual_speed_left) >= 0.1 * target_left && rise_time_left == 0.0) {
                rise_time_left = point.timestamp;
            }
            if (std::abs(point.actual_speed_right) >= 0.1 * target_right && rise_time_right == 0.0) {
                rise_time_right = point.timestamp;
            }
        }
        
        for (const auto& point : data) {
            if (std::abs(point.actual_speed_left) >= 0.9 * target_left) {
                rise_time_left = point.timestamp - rise_time_left;
                break;
            }
        }
        
        for (const auto& point : data) {
            if (std::abs(point.actual_speed_right) >= 0.9 * target_right) {
                rise_time_right = point.timestamp - rise_time_right;
                break;
            }
        }
    }
    
    void calculateSynchronizationError() {
        if (data.empty()) return;
        
        double sum_sync_error = 0.0;
        for (const auto& point : data) {
            sum_sync_error += std::abs(point.actual_speed_left - point.actual_speed_right);
        }
        synchronization_error = sum_sync_error / data.size();
    }
    
    void calculateOverallScore() {
        // Weighted scoring system (lower is better)
        double score = 0.0;
        
        // Settling time (weight: 0.25)
        score += 0.25 * (settling_time_left + settling_time_right) / 2.0;
        
        // Overshoot (weight: 0.20)
        score += 0.20 * (overshoot_left + overshoot_right) * 100.0;
        
        // Steady state error (weight: 0.30)
        score += 0.30 * (steady_state_error_left + steady_state_error_right);
        
        // Rise time (weight: 0.15)
        score += 0.15 * (rise_time_left + rise_time_right) / 2.0;
        
        // Synchronization (weight: 0.10)
        score += 0.10 * synchronization_error;
        
        overall_score = score;
    }
};

class PIDTuner {
public:
    PIDTuner(FourChMotorDriveComms& comm) : motor_comm_(comm) {
        // Default test parameters
        test_speeds_ = {100, 200, 300, 500}; // Motor speed values
        test_duration_ = 8.0; // seconds
        sample_rate_hz_ = 50.0; // 50 Hz sampling
        safety_timeout_ = 15.0; // Safety timeout in seconds
    }
    
    void runFullCalibration() {
        std::cout << "\n=== INTERACTIVE PID CALIBRATION SYSTEM ===" << std::endl;
        
        if (!motor_comm_.connected()) {
            std::cerr << "Error: Motor communication not established!" << std::endl;
            return;
        }
        
        // Safety check
        std::cout << "\nSafety check: Testing basic communication..." << std::endl;
        if (!performSafetyCheck()) {
            std::cerr << "Safety check failed! Aborting calibration." << std::endl;
            return;
        }
        
        // Initialize data logging
        initializeLogging();
        
        // Step 1: Get initial parameters via user input
        std::cout << "\n--- Step 1: Get initial parameters (Ziegler-Nichols) ---" << std::endl;
        PIDParams current_params = zieglerNicholsTuning();
        
        TestResult best_result;
        best_result.overall_score = std::numeric_limits<double>::max();
        
        char choice = ' ';
        while (choice != 's') {
            std::cout << "\n--- Step 2: Interactive Tuning ---" << std::endl;
            std::cout << "Current PID: " << current_params.toString() << std::endl;
            if (best_result.overall_score != std::numeric_limits<double>::max()) {
                std::cout << "Best PID so far: " << best_result.params.toString() 
                          << " (Score: " << std::fixed << std::setprecision(3) << best_result.overall_score << ")" << std::endl;
            }

            std::cout << "\nChoose an action:" << std::endl;
            std::cout << " [t] Test current PID values" << std::endl;
            std::cout << " [m] Manually enter new PID values" << std::endl;
            std::cout << " [s] Save best values and exit" << std::endl;
            std::cout << "> ";
            std::cin >> choice;
            
            // Clear input buffer in case of extra characters
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            switch (choice) {
                case 't': {
                    TestResult result = testPIDParams(current_params);
                    all_results_.push_back(result);
                    std::cout << "\nTest complete. Score: " << result.overall_score << " (lower is better)" << std::endl;
                    if (result.overall_score < best_result.overall_score) {
                        best_result = result;
                        std::cout << "  -> This is the new best score!" << std::endl;
                    }
                    break;
                }
                case 'm': {
                    std::cout << "Enter new Kp: ";
                    std::cin >> current_params.kp;
                    std::cout << "Enter new Ki: ";
                    std::cin >> current_params.ki;
                    std::cout << "Enter new Kd: ";
                    std::cin >> current_params.kd;
                    // Clear input buffer
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    break;
                }
                case 's':
                    std::cout << "Finishing tuning..." << std::endl;
                    break;
                default:
                    std::cout << "Invalid choice. Please try again." << std::endl;
                    break;
            }
        }

        if (best_result.overall_score == std::numeric_limits<double>::max()) {
            std::cout << "\nNo tests were performed. Exiting without saving." << std::endl;
            return;
        }

        std::cout << "\n=== CALIBRATION COMPLETE ===" << std::endl;
        std::cout << "Applying final optimal PID parameters..." << std::endl;
        motor_comm_.set_pwm_pid_values(best_result.params.kp, best_result.params.kd, best_result.params.ki);

        generateReports(best_result);
        
        std::cout << "Optimal PID parameters: " << best_result.params.toString() << std::endl;
        std::cout << "Overall score: " << std::fixed << std::setprecision(3) 
                  << best_result.overall_score << std::endl;
    }
    
    TestResult testPIDParams(const PIDParams& params) {
        std::cout << "Testing PID params: " << params.toString() << std::endl;
        
        // Set PID parameters
        motor_comm_.set_pwm_pid_values(params.kp, params.kd, params.ki);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        TestResult result;
        result.params = params;
        
        static int test_run_counter = 0;
        // Test multiple speed levels
        for (int speed : test_speeds_) {
            auto test_data = performStepResponse(speed, test_run_counter, params);
            result.data.insert(result.data.end(), test_data.begin(), test_data.end());
            
            // Safety delay between tests
            motor_comm_.set_motor_speed_values(0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            test_run_counter++;
        }
        
        result.calculateMetrics();
        return result;
    }
    
    void generateReports() {
        // This will be called with final results
        if (all_results_.empty()) return;
        
        // Find best result
        auto best_result = std::min_element(all_results_.begin(), all_results_.end(),
            [](const TestResult& a, const TestResult& b) {
                return a.overall_score < b.overall_score;
            });
        
        generateReports(*best_result);
    }

private:
    FourChMotorDriveComms& motor_comm_;
    std::vector<int> test_speeds_;
    double test_duration_;
    double sample_rate_hz_;
    double safety_timeout_;
    std::vector<TestResult> all_results_;
    std::string log_filename_;
    
    bool performSafetyCheck() {
        try {
            // Test basic communication
            double left_enc, right_enc;
            motor_comm_.read_encoder_data(left_enc, right_enc, 1);
            
            // Test motor control with low speed
            motor_comm_.set_motor_speed_values(50, 50);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            motor_comm_.set_motor_speed_values(0, 0);
            
            std::cout << "Safety check passed." << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Safety check failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void initializeLogging() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "pid_tuning_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        log_filename_ = ss.str();
        
        std::ofstream file(log_filename_);
        file << "timestamp,test_id,pid_p,pid_i,pid_d,target_left,target_right,"
             << "actual_speed_left,actual_speed_right,pos_left,pos_right,"
             << "error_left,error_right\n";
        file.close();
        
        std::cout << "Data logging initialized: " << log_filename_ << std::endl;
    }
    
    PIDParams zieglerNicholsTuning() {
        std::cout << "The Ziegler-Nichols method can provide a good starting point for PID values." << std::endl;
        std::cout << "It requires finding the 'ultimate gain' (Ku) and 'ultimate period' (Tu)." << std::endl;
        std::cout << "To do this: set Ki and Kd to 0, and gradually increase Kp until the motor output oscillates with a constant amplitude." << std::endl;
        std::cout << "Ku is the value of Kp that causes this oscillation." << std::endl;
        std::cout << "Tu is the period of the oscillation (in seconds)." << std::endl;
        std::cout << "\nThis program cannot do this automatically. Please perform this test manually if you wish." << std::endl;
        std::cout << "Alternatively, you can just enter your best guess for starting parameters." << std::endl;
        
        float ku, tu;
        std::cout << "\nPlease enter the ultimate gain (Ku) you found (or a starting guess for Kp): ";
        std::cin >> ku;
        std::cout << "Please enter the ultimate period in seconds (Tu) you found (or a starting guess like 1.0): ";
        std::cin >> tu;
        
        if (ku <= 0 || tu <= 0) {
            std::cout << "Invalid Ku or Tu. Using conservative default values." << std::endl;
            return PIDParams(1.0f, 0.1f, 0.0f);
        }
        
        // Apply Ziegler-Nichols formulas for PID (classic)
        float zn_kp = 0.6f * ku;
        float zn_ki = 2.0f * zn_kp / tu;
        float zn_kd = zn_kp * tu / 8.0f;
        
        PIDParams zn_params(zn_kp, zn_ki, zn_kd);
        std::cout << "\nCalculated initial Ziegler-Nichols parameters: " << zn_params.toString() << std::endl;
        
        return zn_params;
    }

    // The automatic tuning methods (optimizeAroundBase, comprehensiveTest, checkForOscillation, etc.) 
    // have been removed as they are not suitable for a process with manual reboots. 
    // The new interactive workflow in runFullCalibration() replaces their functionality.
    
    std::vector<DataPoint> performStepResponse(int target_speed, int test_id, const PIDParams& params) {
        std::vector<DataPoint> data;
        
        // Start motors
        motor_comm_.set_motor_speed_values(target_speed, target_speed);
        
        auto start_time = std::chrono::steady_clock::now();
        double sample_interval = 1.0 / sample_rate_hz_;
        
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed >= test_duration_) break;
            
            DataPoint point;
            point.timestamp = elapsed;
            point.target_speed_left = target_speed;
            point.target_speed_right = target_speed;
            
            // Read actual values
            motor_comm_.read_encoder_data(point.actual_speed_left, point.actual_speed_right, 3);
            motor_comm_.read_encoder_data(point.position_left, point.position_right, 1);
            
            // Calculate errors
            point.error_left = point.target_speed_left - point.actual_speed_left;
            point.error_right = point.target_speed_right - point.actual_speed_right;
            
            data.push_back(point);
            
            // Log to file
            logDataPoint(point, test_id, params);
            
            std::this_thread::sleep_for(std::chrono::duration<double>(sample_interval));
        }
        
        return data;
    }
    
    void performSynchronizationTest(const PIDParams& params) {
        std::cout << "Running synchronization test..." << std::endl;
        
        motor_comm_.set_pwm_pid_values(params.kp, params.kd, params.ki);
        
        // Test with different speed combinations
        int speed_combinations[][2] = {
            {300, 300}, {500, 500}, {700, 700},
            {300, 500}, {500, 300}, {200, 800}
        };
        
        for (auto& speeds : speed_combinations) {
            std::cout << "Testing speeds: L=" << speeds[0] << " R=" << speeds[1] << std::endl;
            
            motor_comm_.set_motor_speed_values(speeds[0], speeds[1]);
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Collect sync data
            for (int i = 0; i < 50; ++i) {
                double left_speed, right_speed;
                motor_comm_.read_encoder_data(left_speed, right_speed, 3);
                // Log synchronization data...
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            
            motor_comm_.set_motor_speed_values(0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    void performDisturbanceRejectionTest(const PIDParams& params) {
        std::cout << "Running disturbance rejection test..." << std::endl;
        
        motor_comm_.set_pwm_pid_values(params.kp, params.kd, params.ki);
        
        // Set constant speed
        int base_speed = 400;
        motor_comm_.set_motor_speed_values(base_speed, base_speed);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Apply disturbances (speed changes)
        int disturbances[] = {600, 200, 800, 100, 500};
        
        for (int dist_speed : disturbances) {
            std::cout << "Applying disturbance: " << dist_speed << std::endl;
            
            motor_comm_.set_motor_speed_values(dist_speed, dist_speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            motor_comm_.set_motor_speed_values(base_speed, base_speed);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        motor_comm_.set_motor_speed_values(0, 0);
    }
    
    void logDataPoint(const DataPoint& point, int test_id, const PIDParams& params) {
        std::ofstream file(log_filename_, std::ios::app);
        file << std::fixed << std::setprecision(6);
        file << point.timestamp << "," << test_id << ","
             << params.kp << "," << params.ki << "," << params.kd << ","
             << point.target_speed_left << "," << point.target_speed_right << ","
             << point.actual_speed_left << "," << point.actual_speed_right << ","
             << point.position_left << "," << point.position_right << ","
             << point.error_left << "," << point.error_right << "\n";
        file.close();
    }
    
    void generateReports(const TestResult& result) {
        std::cout << "\n=== DETAILED TEST RESULTS ===" << std::endl;
        std::cout << "Optimal PID Parameters: " << result.params.toString() << std::endl;
        std::cout << "\nPerformance Metrics:" << std::endl;
        std::cout << "  Settling Time:    L=" << std::fixed << std::setprecision(3) 
                  << result.settling_time_left << "s, R=" << result.settling_time_right << "s" << std::endl;
        std::cout << "  Overshoot:        L=" << std::setprecision(1) 
                  << result.overshoot_left*100 << "%, R=" << result.overshoot_right*100 << "%" << std::endl;
        std::cout << "  Steady State Err: L=" << std::setprecision(3) 
                  << result.steady_state_error_left << ", R=" << result.steady_state_error_right << std::endl;
        std::cout << "  Rise Time:        L=" << result.rise_time_left 
                  << "s, R=" << result.rise_time_right << "s" << std::endl;
        std::cout << "  Sync Error:       " << result.synchronization_error << std::endl;
        std::cout << "  Overall Score:    " << result.overall_score << " (lower is better)" << std::endl;
        
        // Generate analysis report
        std::string report_filename = log_filename_.substr(0, log_filename_.find('.')) + "_report.txt";
        std::ofstream report(report_filename);
        
        report << "PID TUNING REPORT\n";
        report << "================\n\n";
        report << "Optimal Parameters: " << result.params.toString() << "\n\n";
        report << "Performance Analysis:\n";
        report << "- System demonstrates ";
        if (result.overshoot_left < 0.1 && result.overshoot_right < 0.1) {
            report << "excellent stability with minimal overshoot\n";
        } else if (result.overshoot_left < 0.2 && result.overshoot_right < 0.2) {
            report << "good stability with acceptable overshoot\n";
        } else {
            report << "marginal stability - consider reducing Kp\n";
        }
        
        report << "- Settling time is ";
        double avg_settling = (result.settling_time_left + result.settling_time_right) / 2.0;
        if (avg_settling < 2.0) {
            report << "fast (" << avg_settling << "s)\n";
        } else if (avg_settling < 4.0) {
            report << "moderate (" << avg_settling << "s)\n";
        } else {
            report << "slow (" << avg_settling << "s) - consider increasing Kp or Ki\n";
        }
        
        report << "\nRecommendations:\n";
        if (result.synchronization_error > 50.0) {
            report << "- High synchronization error detected. Check mechanical coupling.\n";
        }
        if (result.steady_state_error_left > 20.0 || result.steady_state_error_right > 20.0) {
            report << "- Consider increasing Ki for better steady-state performance.\n";
        }
        if (result.overshoot_left > 0.25 || result.overshoot_right > 0.25) {
            report << "- Reduce Kp or increase Kd to minimize overshoot.\n";
        }
        
        report << "\nData Files Generated:\n";
        report << "- Raw data: " << log_filename_ << "\n";
        report << "- Analysis: " << report_filename << "\n";
        
        report.close();
        
        std::cout << "\nReports generated:" << std::endl;
        std::cout << "  Data file: " << log_filename_ << std::endl;
        std::cout << "  Report:    " << report_filename << std::endl;
        
        // Generate Python plotting script
        // generatePlotScript();
        std::cout << "\nTo visualize the results, run:" << std::endl;
        std::cout << "  python3 plots.py " << log_filename_ << std::endl;
    }
};

int main() {
    try {
        // Initialize motor communication
        FourChMotorDriveComms motor_comm;
        motor_comm.connect("/dev/ttyMOTOR", 115200, 100);
        
        // Configure motor parameters (adjust as needed)
        motor_comm.set_motor_mphase(40);
        motor_comm.set_motor_mline(11);
        motor_comm.read_flash_settings();
        
        // Create and run PID tuner
        PIDTuner tuner(motor_comm);
        tuner.runFullCalibration();
        
        motor_comm.disconnect();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
