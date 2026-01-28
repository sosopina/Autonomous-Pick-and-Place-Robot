#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

int main_bas(void) {
    const char* port = "/dev/ttyUSB0";
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        std::cerr << "Erreur: impossible d'ouvrir le port série\n";
        return 1;
    }

    // Configuration du port
    termios options{};
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);     // Activer réception
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                  // 8 bits
    options.c_cflag &= ~PARENB;              // Pas de parité
    options.c_cflag &= ~CSTOPB;              // 1 stop bit
    options.c_cflag &= ~CRTSCTS;             // Pas de contrôle de flux

    tcsetattr(fd, TCSANOW, &options);

    // Envoi
    const char* msg = "VER \r";
    write(fd, msg, strlen(msg));
    usleep(50000);
    // Lecture
    char buffer[256];

    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
        buffer[n] = '\0';
        std::cout << "Reçu: " << buffer << "\n";
    }

    close(fd);
    return 0;
}
