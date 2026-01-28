// - Servo simple -
// This simple example sets up a Servo objectm hooks the event handlers and opens it for device connections.  Once a Servo is attached
// with a motor in motor 0 it will simulate moving the motor from position 15 to 231, displaying the event details to the console.
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License.
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include <stdio.h>
#include <stdlib.h>
#define SSC32_C
#include "ssc32.h"
#include "tty_communication.h"
#include "util.h"

float scale_(float x, float x0, float x1, float y0, float y1, char switch_saturate) {
    float y;
    y = (x - x0) / (x1 - x0) *(y1 - y0) + y0;
    if (!switch_saturate) {
        return y;
    }
    if (y0 < y1) {
        if (y <= y0) return y0;
        if (y >= y1) return y1;
    } else {
        if (y <= y1) return y1;
        if (y >= y0) return y0;
    }
    return y;
}

void ssc32_print_info(struct_ssc32 *s) {
    float us;
    int i;
    // print information of the card configuration in matlab usable langage 
    printf(" nb_servo= %d ;\n", s->nb_servo);
    for (i = 0; i < s->nb_servo; i++) {
        printf("//----------------------------------------------------\n");
        printf("// axe[%d] = %s \n", i, s->name[i]);
        printf("//----------------------------------------------------\n");

        printf("  ssc_32_port = %d \n", s->port[i]);
        us = scale_(s->positionMinUnit[i], s->position0Unit[i], s->position1Unit[i], s->position0Us[i], s->position1Us[i], (char)0);
        printf("  min position = %.2f %s  <=> %d us\n", s->positionMinUnit[i], s->unit[i], (int) us);
        us = scale_(s->positionMaxUnit[i], s->position0Unit[i], s->position1Unit[i], s->position0Us[i], s->position1Us[i], (char)0);
        printf("  max position = %.2f %s  <=> %d us\n ", s->positionMaxUnit[i], s->unit[i], (int) us);
        printf("  current position ref = %.2f %s \n", s->control_input_value[i], s->unit[i]);

    }

}

void ssc32_set_armed(struct_ssc32 *s, int num_servo, char armed) {

}

void ssc32_set_us(struct_ssc32 *s, int num_servo, double us) {
    char command[64];
    int us_int = (int) us;
    s->currentPositionRefUs[num_servo] = us_int;
    sprintf(command, "#%2.2d P%d \r", s->port[num_servo], us_int);
    //  printf("command =%s\n", command);
    int i = write_adr_port(s->fd,(char*)command);
}

void ssc32_apply_all_values(struct_ssc32 *s, int min_time_ms,char verbose) {
    char ok_speed_i;
    int num_servo;
    float value_ref, value_us, value_speed_us_s;
    char command [1500], command_i[43];
    //    # <ch> P <pw> S <spd> ... # <ch> P <pw> S <spd> T <time> <cr>
    //exemple : "#5 P1600 S750 #6 P1800 T1000 <cr>"
    // les servos demarreront et s'arreteront en meme temps :
    // tps minimum de parcours = 1s
    // si vitesse max servo 5 => temps > 1000 ms , alors le temps de parcours augmentera
    command[0] = 0;
    for (num_servo = 0; num_servo < s->nb_servo; num_servo++) {
        //1- saturate control_input_value and put saturated value in currentPositionRefUnit  
        value_ref = s->control_input_value[num_servo];
        if (value_ref > s->positionMaxUnit[num_servo]) value_ref = s->positionMaxUnit[num_servo];
        else if (value_ref < s->positionMinUnit[num_servo]) value_ref = s->positionMinUnit[num_servo];
        s->currentPositionRefUnit[num_servo] = value_ref;
        //2- convert currentPositionRefUnit  in us, and update ieme command string, with port and us value
        value_us = scale_(value_ref, s->position0Unit[num_servo], s->position1Unit[num_servo], s->position0Us[num_servo], s->position1Us[num_servo], 1);
        s->currentPositionRefUs[num_servo] = (int) value_us;
        sprintf(command_i, "#%d P%d ", s->port[num_servo], (int) (value_us+0.5));
        //3- add optionnally max speed info to ieme command string [only if maxVelocityUnitPerSecond >0]
        ok_speed_i = s->maxVelocityUnitPerSecond[num_servo] > 0;
        if (ok_speed_i) {
            value_speed_us_s = scale_(s->maxVelocityUnitPerSecond[num_servo], s->position0Unit[num_servo], s->position1Unit[num_servo], s->position0Us[num_servo], s->position1Us[num_servo], (char)1);
            ok_speed_i = (value_speed_us_s > 1) &&(value_speed_us_s < 65535);
        }
        if (ok_speed_i) {
            sprintf(command_i, "%sS%d ", command_i, (int) (value_speed_us_s+0.5));
        }
        //4- catenate ieme command string to complete command string        
        sprintf(command, "%s%s", command, command_i);
    } // for(num_servo...
    // add time information if necessary
    if ((min_time_ms > 0)&&(min_time_ms < 65535)) {
        sprintf(command_i, "T%d ", min_time_ms);
        sprintf(command, "%s%s", command, command_i);
    }
    // finish command string 
    sprintf(command, "%s\r", command);
    if( verbose) {
      printf("ss32_command =%s\n", command);
    }
    //apply command string to port
    write_adr_port(s->fd, command);    
}

void ssc32_set_value(struct_ssc32 *s, int num_servo, double value_ref) {
    double value_us;
    if (value_ref > s->positionMaxUnit[num_servo]) value_ref = s->positionMaxUnit[num_servo];
    else if (value_ref < s->positionMinUnit[num_servo]) value_ref = s->positionMinUnit[num_servo];
    s->currentPositionRefUnit[num_servo] = value_ref;
    value_us = scale_(value_ref, s->position0Unit[num_servo], s->position1Unit[num_servo], s->position0Us[num_servo], s->position1Us[num_servo], 1);
    // saturate beetwen min and max reference position
    ssc32_set_us(s, num_servo, value_us);
}

char new_struct_ssc32(struct_ssc32 *s, char *port_name) {
    int i;
    double v;
    char tmp[100];
    s->is_used = 1;
    // initialize ttyS0 to dialog with ssc32 card
    //system("stty -F /dev/ttyS0 115200");
    s->fd = open_adr_port(port_name);
    if (s->fd < 0) return 0;
    // initialize positions according to HS_322 servo motor

    s->nb_servo = 32;
    // initialize max accelerations and speeds
    for (i = 0; i < s->nb_servo; i++) {
        s->port[i] = i;
        sprintf(tmp, "servo %d on port %d", i, s->port[i]);
        strcpy(s->name[i], tmp);
        strcpy(s->unit[i], "unknown unit");
        ssc32_set_armed(s, i, 0);
        s->position0Unit[i] = HS_322_MIN_DEG;
        s->position1Unit[i] = HS_322_MAX_DEG;
        s->position0Us[i] = HS_322_MIN_US;
        s->position1Us[i] = HS_322_MAX_US;
        s->positionMinUnit[i] = my_min(s->position0Unit[i], s->position1Unit[i]);
        s->positionMaxUnit[i] = my_max(s->position0Unit[i], s->position1Unit[i]);
        v = (s->positionMinUnit[i] + s->positionMaxUnit[i]) / 2;
        s->initialControlInput[i] = v;
        s->currentPositionRefUnit[i] = v;
        s->control_input_value[i] = v;
        s->minVel[i] = -10000;
        s->maxVelocityUnitPerSecond[i] = 100; // default 100us/s
        s->minAccel[i] = s->minVel[i] / 0.01;
        s->maxAccel[i] = s->maxVelocityUnitPerSecond[i] / 0.01;
    }
    return 1;
}

void free_ssc32(struct_ssc32 *s) {
    int i;
    if (s == NULL) {
        return;
    }

    close_adr_port(s->fd);
    s->fd = 0;
}

int get_pulse_with(int num_servo) {
    return 0;
}

void ssc32_send_command(struct_ssc32 *s, const char *command) {
    write_adr_port(s->fd, command);
}

int ssc32_receive_response(struct_ssc32 *s, char *rep, int nb_char_max) {
    return read_adr_port(s->fd, rep, nb_char_max + 1);
}

int ssc32_query_movement_status(struct_ssc32 *s, char *status) {
    int i;
    // char* msg = "Q\r";
    ssc32_send_command(s, "Q\r");
    usleep(5000);
    i = ssc32_receive_response(s, status, 2);
    return i;
}

void ssc32_flush_buffer(struct_ssc32 *s) {
    char rep[1024];
    while (ssc32_receive_response(s, rep, 1000) > 0) {

    }
}

int ssc32_query_pulse_width(struct_ssc32 *s, int num_servo) {
    int i;
    char command[100];
    char response[100];

    ssc32_flush_buffer(s);
    sprintf(command, "QP %d \r", num_servo);
    ssc32_send_command(s, command);
    do {
        i = ssc32_receive_response(s, response, 1);
    } while (i == 0);
    i = (int) response[0];
    return i * 10;
}

char ssc32_parse_args(struct_ssc32 *s, char command[]) {
    int i;
    long num_servo[NB_MAX_MOTOR], ns, nb_servo = 0,time_ms;
    float val_servo[NB_MAX_MOTOR], vs;

    char *next, *cur, ok, ok_servo, ok_time=0;
    cur = command;
    nb_servo = 0;
    while (cur[0] != 0) {
        ok_servo = (cur[0] == '#');
        ok_time = (cur[0] == 'T');
        ok = ok_servo || ok_time;
        if (!ok) return 0;
        // look at num servo
        ns = strtol(&(cur[1]), &next, 10);
        // at least one character has been traduced
        if (next == cur) return 0;
        if (ok_servo) {
            // first non traduced char must be a space 
            if (next[0] != ' ') return 0;
            if (ns < 0) return 0;
            if (ns > s->nb_servo) return 0;
            num_servo[nb_servo] = ns;
            // 2- traduce the value (floating point format)
            cur = next + 1;
            vs = strtof(cur, &next);
            // at least one character has been traduced
            if (next == cur) return 0;
            // first non traduced char must be a space, or end of string 
            if ((next[0] != ' ')&&(next[0] != 0)&&(next[0] != '"')&&(next[0] != 'T')) return 0;
            val_servo[nb_servo++] = vs;
            if (next[0] == ' ') cur = &(next[1]);
            else cur = next;
        } // if ok_servo
        if (ok_time) {
            // first non traduced char must be a 0 :time at end of string 
            if (next[0] != 0) return 0;
            if (ns < 0) return 0;
            time_ms=ns; 
            ok_time=ns<65535;
            cur = next;
        } // if ok_time       
    }
    // every thing ok if nb_servo>0
    if (nb_servo<=0) return 0;
//----------------------------------------------------
// transmit updated values to ssc32 structure
//----------------------------------------------------
    if (ok_time) { 
      s->min_time_ms=time_ms;        
    }else {
        s->min_time_ms=0;
    }
    // update values in structure (only the one specifed by user)
    for (i=0;i<nb_servo;i++) {
        ns=num_servo[i];vs=val_servo[i];
        s->control_input_value[ns]=vs;
    }
    // return 1
    return 1;
}
int main_ssc32_manual_test(int argc, char** argv) {
    char uart_port_name[] = "/dev/ttyUSB0";
    struct_ssc32 *s = (struct_ssc32 *)malloc(sizeof (struct_ssc32));
    char rep[1000], command[100], finish = 0, ok, need_update,apply_control;
    float step = 1, value_ref_us; // commn step for all axis
    char num_active_servo = 0;
    int i;
    long int count = -1;
    ok =new_struct_ssc32(s,uart_port_name);
    s->nb_servo=1;
    s->port[0]=16; 
    ssc32_print_info(s);
    
    if (!ok) {
        printf("unable to establish communication with ssc32 on port : %s \n", uart_port_name);
        return EXIT_FAILURE;
    }
    // show version of ssc32
    ssc32_send_command(s, "VER \r");
    ssc32_receive_response(s, rep, 1000);
    
    printf("SSC32 VER :%s\n", rep);
    printf("----------------------------------\nCOMMANDS\n#0 1200 #2 2500 T2000: new setpoint (values in servo units, not in us, T is optional )\nspace: apply setpoint,\nq or esc:quit\nempty line: show setpoint\n----------------------------------\n");
    finish = 0;
    while (!finish) {
        need_update=0;apply_control=0;
        if (count >=0) {
          fgets(command,100,stdin);
        } else {
            command[0]=0; // just to see servo state the very first time
            count =0;
        }
        // replace last \n by 0 
        i=strlen(command);
        if (i>0) {
          if (command[i-1]=='\n') command[i-1] =0;
        }  
        if (strlen(command)==1) {
        // single caracter command    
          finish = (command[0] == 27) || (command[0] == 'q');
          apply_control=command[0]==' ';
        }else if (strlen(command)==0){
           need_update =1; // just print servo states
        }else {
          ok=ssc32_parse_args(s,command);
          need_update = (ok==1);
        }
//        printf("ascii(command) =%d\n", (int) command);
        if (!finish) {
            if (need_update) {
                printf("set_point [ %d ]\n", (int)count++);
                if (s->min_time_ms>0) {
                    printf( "  min time = %d ms\n",s->min_time_ms);
                }else {
                    printf( "  min time = 0 ms\n");
                    
                }
                for (i = 0; i < s->nb_servo; i++) {
                    value_ref_us = scale_(s->control_input_value[i], s->position0Unit[i], s->position1Unit[i], s->position0Us[i], s->position1Us[i], 1);
                    printf(" sv[%2d], ssc32_port:%2d, value= %4d us <=> %4d %s [%s]\n", i, s->port[i], (int) value_ref_us, (int) s->control_input_value[i], s->unit[i], s->name[i]);
                }
                need_update = 0;
            }
            if (apply_control) {
                printf("applying control:");
                ssc32_apply_all_values(s,s->min_time_ms,1);s->min_time_ms=0;
            }
        }
    } // while (!finish)
    free_ssc32(s);
    return 0;


}

