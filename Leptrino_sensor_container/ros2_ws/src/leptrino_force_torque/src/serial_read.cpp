#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

int main() {
    int fd;
    struct termios tty;

    // 打开串口
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Unable to open /dev/ttyACM0" << std::endl;
        return -1;
    }

    // 设置串口参数
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting tty attributes" << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // 设置为非阻塞模式
    fcntl(fd, F_SETFL, FNDELAY);

    // 设置 VTIME 和 VMIN
    tty.c_cc[VTIME] = 1; // 0.1 秒
    tty.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting tty attributes" << std::endl;
        close(fd);
        return -1;
    }

    // 连续读取串口数据
    int count = 0;
    char buffer[100];
    while (count < 5) {
        int n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            std::cout << "Data received (" << n << " bytes): ";
            for (int i = 0; i < n; i++) {
                printf("%02x ", buffer[i]);
            }
            std::cout << std::endl;
            count++;
        }
        usleep(100000); // 休眠 100ms
    }

    // 关闭串口
    close(fd);
    return 0;
}
