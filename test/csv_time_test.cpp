#include <iostream>
#include <chrono>
#include <iomanip> // std::fixed, std::setprecision

/*CSVに記録するtimeデータの表示テスト*/

int main() {
    // 現在時刻の取得（秒単位）
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    double time = epoch.count() / 1000.0;

    // 日本時間に変換（エポックタイムに9時間を加算）
    // double jst_time = time + 9 * 3600;

    // 取得した現在時刻を表示（通常の浮動小数点数表記で）
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Current time in seconds since epoch: " << time << " seconds" << std::endl;

    return 0;
}
