#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

int main() {
    int sockfd;
    struct sockaddr_in servaddr;

    // ソケットの作成
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(12345); // 任意のポート番号
    servaddr.sin_addr.s_addr = inet_addr("192.168.11.2"); // MATLABが動作しているPCのIPアドレス

    // 送信する数字
    int number = 42;
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), "%d", number);

    // UDPパケットの送信
    sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

    // ソケットのクローズ
    close(sockfd);

    std::cout << "Number sent: " << number << std::endl;

    return 0;
}
