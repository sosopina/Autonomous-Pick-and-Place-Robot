// adrport.c - Serial Port Handler
// Copyright MMI, MMII by Sisusypro Incorporated

// Permission is hereby granted to freely copy,
// modify, utilize and distribute this example in
// whatever manner you desire without restriction.

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "tty_communication.h"
// #include <bits/fcntl-linux.h>
static int global_fd = 0;
#define VERBOSE
/***************************************************************************
 * signal handler. sets wait_flag to FALSE, to indicate above loop that     *
 * characters have been received.                                           *
 ***************************************************************************/
//#define ASYNCHRONOUS
#ifdef ASYNCHRONOUS
int IO_wait_flag = 1; // true >waiting for data
int IO_count = 0;

void SIGIO_handler(int status) {
    printf("SIGIO_handler %d\n", IO_count);
    IO_count++;
    IO_wait_flag = 0;
}
#endif
// opens the serial port
// return code:
//   > 0 = fd for the port
//   -1 = open failed

int open_adr_port(char* serial_port_name) {
    int fd;
#ifdef ASYNCHRONOUS
    struct sigaction saio; //definition of signal action
#endif
    struct termios options;
    fd = open(serial_port_name, O_RDWR | O_NOCTTY );//|O_NDELAY 
    if (fd < 0) {
        printf("unable to open %s , err=%d\n",serial_port_name,errno);
        return fd;
    }
#ifdef ASYNCHRONOUS
    // install serial handler before making the device asynchronous
    saio.sa_handler = SIGIO_handler;
    sigemptyset(&saio.sa_mask); //saio.sa_mask = 0;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO, &saio, NULL);
    // allow the process to receive SIGIO
    fcntl(fd, F_SETOWN, getpid());
    // Make the file descriptor asynchronous (the manual page says only
    // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
    fcntl(fd, F_SETFL, FASYNC);
    // Make the file descriptor asynchronous (the manual page says only
    // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
    fcntl(fd, F_SETFL, FASYNC);
#endif

    /*
     * Get the current options for the port...
     */

    tcgetattr(fd, &options);
    //bzero(&options, sizeof (options));

    /*
     * Set the baud rates to 115200...
     */

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);

    /*
        IGNPAR  : ignore bytes with parity errors
        ICRNL   : map CR to NL (otherwise a CR input on the other computer
                  will not terminate input)
        otherwise make device raw (no other input processing)
     */
    options.c_iflag = IGNPAR;

    /*
     Raw output.
     */
    options.c_oflag = 1;

    /*
      ICANON  : enable canonical input
      disable all echo functionality, and don't send signals to calling program
     */
    options.c_lflag = ICANON;

    /*
      initialize all control characters
      default values can be found in /usr/include/termios.h, and are given
      in the comments, but we don't need them here
     */
    options.c_cc[VINTR] = 0; /* Ctrl-c */
    options.c_cc[VQUIT] = 0; /* Ctrl-\ */
    options.c_cc[VERASE] = 0; /* del */
    options.c_cc[VKILL] = 0; /* @ */
    options.c_cc[VEOF] = 4; /* Ctrl-d */
    options.c_cc[VTIME] = 0; /* inter-character timer unused */
    options.c_cc[VMIN] = 0; /* non blocking read until 1 character arrives */
    options.c_cc[VSWTC] = 0; /* '\0' */
    options.c_cc[VSTART] = 0; /* Ctrl-q */
    options.c_cc[VSTOP] = 0; /* Ctrl-s */
    options.c_cc[VSUSP] = 0; /* Ctrl-z */
    options.c_cc[VEOL] = 0; /* '\0' */
    options.c_cc[VREPRINT] = 0; /* Ctrl-r */
    options.c_cc[VDISCARD] = 0; /* Ctrl-u */
    options.c_cc[VWERASE] = 0; /* Ctrl-w */
    options.c_cc[VLNEXT] = 0; /* Ctrl-v */
    options.c_cc[VEOL2] = 0; /* '\0' */
    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, FNDELAY);

    // NOTE: you may want to save the port attributes
    //       here so that you can restore them later
    tcflush(fd, TCIFLUSH);
    return fd;
} // end OpenAdrPort

// writes zero terminated string to the serial port
// return code:
//   >= 0 = number of characters written
//   -1 = write failed

int write_adr_port(int fd, char* string_to_write) {
    int iOut;
    if (fd < 1) {
        return -1;
    } // end if
    iOut = write(fd, string_to_write, strlen(string_to_write));
    return iOut;
} // end WriteAdrPort

// read string from the serial port
// return code:
//   >= 0 = number of characters read
//   -1 = read failed

int read_adr_port(int fd, char* string_to_read, int nb_char) {
    int i_read, i;
    if (fd < 1) {
        return -1;
    } // end if
#ifdef ASYNCHRONOUS
    if (IO_wait_flag!=0){
        return 0;
    }
    IO_wait_flag=1;
#endif
    string_to_read[0] = 0;
    i_read = read(fd, string_to_read, nb_char);
    if (i_read < 0) {
        return - errno;
    }
    if (i_read==0) {
        return i_read;
    }
    if (i_read<nb_char) {
        nb_char=i_read;
    }
    string_to_read[i_read] = '\0';
    return i_read;
} // end ReadAdrPort

// closes the serial port

void close_adr_port(int fd) {
    // you may want to restore the saved port attributes
    if (fd > 0) {
        close(fd);
    } // end if
} // end CloseAdrPort

int main_tty_communication(int argc, char** argv) {
    char uart_port_name[]="/dev/ttyUSB0";
    int i_read, i;
    //system("stty -F /dev/ttyS0 115200");
    int fd = open_adr_port(uart_port_name);
    if (fd<0) {
        return EXIT_FAILURE;
    }
    char rep[1024];
    write_adr_port(fd, "VER \r");
    rep[0] = 27;
    rep[1] = 0;
    write_adr_port(fd, rep);
    sleep(1);
    write_adr_port(fd, "#00 p1500\r");
    sleep(1);
    write_adr_port(fd, "#00 p1000\r");
    sleep(5);
    int l = read_adr_port(fd, rep, 1);// COMMENT TO AVOID PROBLEM
    printf(" read_adr_port return %d\n",l);
    if (l > 0) {
        printf("rep[0..%d]=%s\n", l - 1, rep);
        for (i = 0; i < l; i++) {
            printf("rep[%d]=%d , ", i, (int)rep[i]);
        }
        printf("\n");
    }
    close_adr_port(fd);
    return 0;

}
