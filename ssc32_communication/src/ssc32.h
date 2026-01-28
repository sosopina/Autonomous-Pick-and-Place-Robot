/*
 * File:   simple_servo.h
 * Author: ygorra
 *
 * Created on 16 novembre 2010, 13:05
 */

#ifndef SSC32_H
#define SSC32_H
int main_ssc32_manual_test(int argc, char **argv);
#ifdef __cplusplus

extern "C"
{
#endif
#include <stdio.h>
#define NB_MAX_MOTOR 32
#define SSC_32_SHM_KEY 1272
    typedef struct
    {
        int fd; // file descriptor for serial port "/dev/ttyS0";
        int servo;
        const char *device_type;
        int serial_number;
        int device_version;
        int nb_servo;
        //------------------------------------------------
        // optional interface with external control
        //------------------------------------------------
        char do_activate_control;   // if 1 , activate control and put 0 int this field
        char do_deactivate_control; // if 1 , deactivate control and put 0 int this field
        char control_is_active;     //: if 0 don-apply control
        long int update_time_us;    // used for multi threading
        char is_used;               // used for multi threads
        //----------------------------
        // user side fields
        //----------------------------
        volatile float control_input_value[NB_MAX_MOTOR], initialControlInput[NB_MAX_MOTOR];
        int min_time_ms; // minimal time to join set_point for all axes in ms;
        //----------------------------
        // servo scale and limits
        //----------------------------
        int port[NB_MAX_MOTOR];      // servo number correponding to each servo
        char name[NB_MAX_MOTOR][50]; // string precising unity of Position, can be degre, m , mm,m/s etc
        char unit[NB_MAX_MOTOR][20]; // string precising unity of Position, can be degre, m , mm,m/s etc
        float currentPositionRefUnit[NB_MAX_MOTOR], currentPositionRefUs[NB_MAX_MOTOR];
        float position0Us[NB_MAX_MOTOR], position1Us[NB_MAX_MOTOR];
        float position0Unit[NB_MAX_MOTOR], position1Unit[NB_MAX_MOTOR];     // Attention : pos0 n'est pas forcement < pos1
        float positionMinUnit[NB_MAX_MOTOR], positionMaxUnit[NB_MAX_MOTOR]; // par contre posMin est forcement < posMAx
        // servo caracteristics
        float minAccel[NB_MAX_MOTOR], maxAccel[NB_MAX_MOTOR], minVel[NB_MAX_MOTOR], maxVelocityUnitPerSecond[NB_MAX_MOTOR];

    } struct_ssc32;
#define HS_322_MIN_US 600.0
#define HS_322_MAX_US 2400.0
#define HS_322_MIN_DEG 0.0
#define HS_322_MAX_DEG 180.0
#ifndef SSC32_C
    void free_ssc32(struct_ssc32 *s);
    char new_struct_ssc32(struct_ssc32 *, const char *port_name);
    void ssc32_apply_all_values(struct_ssc32 *s, int min_time_ms, char verbose);
    // int main_ssc32_manual_test(int argc, char **argv);
    void ssc32_set_value(struct_ssc32 *s, int num_servo, double value);
    void ssc32_set_armed(struct_ssc32 *s, int num_servo, char armed);
    void ssc32_set_us(struct_ssc32 *s, int num_servo, double us);
    int ssc32_query_pulse_width(struct_ssc32 *s, int num_servo);
    char refresh_info_ssc32(struct_ssc32 *s, int num_servo);
    // parse args in a string like #0 P25 #1 P18 T2000 and apply it to control ( values are in units, not in us)
    char ssc32_parse_args(struct_ssc32 *s, char command[]);
#else

#endif

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_SERVO_H */
