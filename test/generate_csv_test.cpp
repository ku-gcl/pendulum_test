

std::ofstream csvFile;
int main_rate = 100; // 100ms

void csv_write(time, thetap, thetaw, thetap_dot, thetaw_dot){
    csvFile << time << "," << thetap << "," << thetaw << "," << thetap_dot << "," << thetaw_dot << std::endl;
}

csvFile.open("output_test.csv"); // ファイルを開く
std::signal(SIGINT, signalHandler);
csvFile << "time" << "," << "theta1" << "," << "theta2" << "," << "theta1_dot" << "," << "theta2_dot" << std::endl;

int main(){
    while(1) {
        // csv write
        time = 1; // 現在時刻を取得
        thetap = 2; 
        thetaw = 3;
        thetap_dot = 4;
        thetaw_dot = 5;
        csv_write(time, thetap, thetaw, thetap_dot, thetaw_dot);

        std::chrono::milliseconds(std::this_thread::sleep_for(main_rate));
    }
    csvFile.close();
    return 0;
}