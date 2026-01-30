/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   control.hpp
 * Author: sygorra
 *
 * Created on 1 février 2024, 08:19
 */

#ifndef CONTROL_HPP
#define CONTROL_HPP
#include <stdlib.h>
#include "tustin.hpp"
#define USE_OPTITRACK_CAMERAS
#ifdef USE_OPTITRACK_CAMERAS
#include "NatNetClient.hpp"
#endif
#define MEASURE_DEPTH 1
#define MEASURE_RGB 2

typedef struct
{
    float R;        // rayon des roues en m
    float Ly;       // largeur essieu en m;
    float Lx;       // distance de mesure camera RGB suivant axe Xe
    float encToRad; // angle roue (rad ) / (encoder value )

    float vx, wz, ref_vx, ref_wz;
    int32_t left_encoder, right_encoder;
    int left_pwm, right_pwm; // pwm of motors, between -128 and 127
    Filter lowpassExample;   // an example to understand how to use the tustin approx of continuous filters
                             // CAMERAS REALSENSE MEASURMENTS
    int typeMeasure;         // typeMeasure=MEASURE_RGB or typeMeasure= MEASURE_DEPTH specifies what type of measure has to be done by the camera
    // RGB Camera measurment in meter from first visible row
    unsigned long long frame_count;
    float measure;  // White line measure in meter in case of RGB, mean distance in case of Depth
    bool measureOk; // boolean : true if measure is ok, false otherwise
    // Depth Camera, only one row at the center of image
    float measureAngleDepth; // not yet implemented : mean Angle of retained depth pixels 
    float rowDepth[2000];     // depth row pixels [0 to 639 ] in meters
    float depthMin,depthMax; // seuils pour la mesure de distance
    // emergencyStop
    bool emergencyStop;
    // undocumented kobuki internal pid parameters
    unsigned int p_gain;
    unsigned int i_gain;
    unsigned int d_gain;
    // user measures , appended to data columns in the file data_kobuki.txt
    int nbUserMeasure; // number of user Measures , initialised in function initControl
    float userMeasure[100]; // array of user measures , updated in function oneStepControl
    #ifdef USE_OPTITRACK_CAMERAS
       bool okOptitrack; // true if optitrack localisation system is ok
       structOptitrack optitrackData; // contains rigid bodies and markers coordinates
    #endif
    Filter regAngle;
} ControlStruct;
void initControl(ControlStruct &control);
void oneStepControl(ControlStruct &control);
void endControl(ControlStruct &control);
void handleKeyboard(ControlStruct &control, char key);
#endif /* CONTROL_HPP */
