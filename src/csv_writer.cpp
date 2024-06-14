#include "csv_writer.h"
#include <iostream>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

std::ofstream csvFile;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    if (csvFile.is_open()) {
        csvFile.close();
    }
    exit(signum);
}

// void csv_write(double time, float elapsed_time, float theta_p, float theta_w, float theta_p_dot, float theta_w_dot) {
//     csvFile << std::fixed << std::setprecision(3) << time << ",";
//     csvFile << std::fixed << std::setprecision(5) << elapsed_time << "," << theta_p << "," << theta_w << "," << theta_p_dot << "," << theta_w_dot << std::endl;
// }
void csv_write(double time, float elapsed_time, float theta_p, float theta_p_dot, float theta_w, float theta_w_dot, float theta_p_kf, float theta_p_dot_kf, float theta_w_kf, float theta_w_dot_kf, float log_motor_value, int log_motor_direction, float log_pwm_duty) {
    csvFile << std::fixed << std::setprecision(3) << time << ",";
    csvFile << std::fixed << std::setprecision(5) << elapsed_time << ",";
    csvFile << std::fixed << std::setprecision(5) << theta_p << "," << theta_p_dot << "," << theta_w << "," << theta_w_dot << ",";
    csvFile << std::fixed << std::setprecision(5) << theta_p_kf << "," << theta_p_dot_kf << "," << theta_w_kf << "," << theta_w_dot_kf << ",";
    csvFile << std::fixed << std::setprecision(5) << log_motor_value << "," << log_motor_direction << "," << log_pwm_duty << std::endl;
}

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void openCSVFile(const std::string &filename) {
    std::cout << "CSV try to open1." << std::endl;
    csvFile.open(filename);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        exit(1);
    }
    std::cout << "CSV try to open2." << std::endl;
    std::signal(SIGINT, signalHandler);
    csvFile << "time,elapsed_time,theta_p,theta_p_dot,theta_w,theta_w_dot,theta_p_kf,theta_p_dot_kf,theta_w_kf,theta_w_dot_kf,log_motor_value,log_motor_direction,log_pwm_duty" << std::endl;
    std::cout << "CSV file opened successfully." << std::endl;
}

void closeCSVFile() {
    if (csvFile.is_open()) {
        csvFile.close();
    }
}

double getCurrentEpochTimeUTC() {
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    double time = epoch.count() / 1000.0;
    return time;
}

void createDirectoryIfNotExists(const std::string &directory) {
    if (!std::filesystem::exists(directory)) {
        std::filesystem::create_directories(directory);
        std::cout << "Directory created: " << directory << std::endl; // デバッグメッセージ
    } else {
        std::cout << "Directory already exists: " << directory << std::endl; // デバッグメッセージ
    }
}
