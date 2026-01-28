/* 
 * File:   tty_communication.h
 */
// from adrport.h
// Copyright MMI, MMII by Sisusypro Incorporated


#ifndef TTY_COMMUNICATION_H
#define	TTY_COMMUNICATION_H
int open_adr_port (char* serial_port_name);
int write_adr_port(int fd,char* string_to_write);
int read_adr_port(int fd,char* string_to_read, int iMax);
void close_adr_port(int fd);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* TTY_COMMUNICATION_H */

