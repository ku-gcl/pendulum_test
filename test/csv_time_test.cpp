#include <iostream>
#include <chrono>
#include <iomanip> // std::fixed, std::setprecision

int main() {
    // 現在時刻の取得（秒単位）
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    float time = epoch.count() / 1000.0f;

    // 取得した現在時刻を表示（通常の浮動小数点数表記で）
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Current time in seconds since epoch: " << time << " seconds" << std::endl;

    return 0;
}
