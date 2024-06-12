#include <iostream>
#include <fstream>
#include <csignal>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

/*CSVへのデータ出力をテスト*/

std::ofstream csvFile;
int main_rate = 100; // 100ms

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    if (csvFile.is_open()) {
        csvFile.close();
    }
    exit(signum);
}

void csv_write(double time, float elapsed_time, float theta_p, float theta_w, float theta_p_dot, float theta_w_dot) {
    csvFile << std::fixed << std::setprecision(3) << time << ",";
    csvFile << std::fixed << std::setprecision(5) << elapsed_time << "," << theta_p << "," << theta_w << "," << theta_p_dot << "," << theta_w_dot << std::endl;
}

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

int main() {
    // std::string filename = "csv_test_data/output_" + getCurrentDateTime() + ".csv";
    std::string filename = "test/csv_test_data/output_" + getCurrentDateTime() + ".csv";
    csvFile.open(filename); // ファイルを開く
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        return 1;
    }

    std::signal(SIGINT, signalHandler);

    csvFile << "time" << "," << "elapsed_time" << "," << "theta_p" << "," << "theta_w" << "," << "theta_p_dot" << "," << "theta_w_dot" << std::endl;

    float theta_p, theta_w, theta_p_dot, theta_w_dot;

    auto start = std::chrono::system_clock::now();

    while (true) {
        // 現在時刻の取得（秒単位）
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        double time = epoch.count() / 1000.0;

        // スタートからの経過時間の取得
        auto elapsed = now - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();

        // データの取得（仮の値を使用）
        theta_p = 2.0f;
        theta_w = 3.0f;
        theta_p_dot = 4.0f;
        theta_w_dot = 5.0f;

        // CSV書き込み
        csv_write(time, elapsed_time, theta_p, theta_w, theta_p_dot, theta_w_dot);

        // スリープ
        std::this_thread::sleep_for(std::chrono::milliseconds(main_rate));
    }

    csvFile.close();
    return 0;
}
