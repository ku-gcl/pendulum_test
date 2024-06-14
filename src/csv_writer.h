#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <string>
#include <fstream>

void signalHandler(int signum);
void csv_write(double time, float elapsed_time, float theta_p, float theta_p_dot, float theta_w, float theta_w_dot, float theta_p_kf, float theta_p_dot_kf, float theta_w_kf, float theta_w_dot_kf, float log_motor_value, int log_motor_direction, float log_pwm_duty);
std::string getCurrentDateTime();
void openCSVFile(const std::string &filename);
void closeCSVFile();
double getCurrentEpochTimeUTC();
void createDirectoryIfNotExists(const std::string &directory);

#endif // CSV_WRITER_H
