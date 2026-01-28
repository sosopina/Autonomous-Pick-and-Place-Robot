/* 
 * File:   util.h
 * Author: ygorra
 *
 * Created on 15 novembre 2010, 14:32
 */

#ifndef UTIL_H
#define UTIL_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
float scale(float x, float x0, float x1, float y0, float y1, char switch_saturate);
#ifdef __cplusplus

extern "C" {
#endif
#ifndef PI
#define PI 3.14159265358979323846 /* pi */
#endif
#ifndef my_max
#define my_max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef my_min
#define my_min(a,b) ((a)<(b)?(a):(b))
#endif
    long long int get_cpu_time_in_microsec(void);
    float get_cpu_time_in_sec(void);
    // float scale(float x, float x0, float x1, float y0, float y1, char switch_saturate);
    char * get_stdout_from_command(char * cmd);
    // return the argument value or NULL ( use intenally malloc )
    // example :if the program is called with -vtoto ,
    // v=get_arg_if_exists("-v",argc,arv), will return toto ( don't forget to use free(toto) when unused
    char * get_arg_if_exists(char *name, int argc, char** argv);
    void *get_or_create_shm(int key, int size); // recup ou cree segment memoire partagee identifie par key
#ifdef __cplusplus
}
#endif

#endif /* UTIL_H */

