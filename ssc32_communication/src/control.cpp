/*

 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 * Pour compiler ecrire dans un terminal sous le même dossier:
  sh.compile/sh 
 Pour executer ecrire dans un terminal sous le même dossier
  ./kobuki_with_rs_program
 */

#include "tustin.hpp"
#include "control.hpp"
#include <cstdio>                 // to use printf as in c, because std:cout is redirected to file and is not usable
#define PI 3.14159265358979323846 /* pi */

void initControl(ControlStruct &control)
{    
    system("clear"); // effacement de la console
    // undocumented kobuki internal pid parameters, initialized to default values
    control.p_gain = 1 * 100 * 1000;
    control.i_gain = 0.1 * 1000;
    control.d_gain = 2 * 1000;
    // emergencyStop is false when starting control
    control.emergencyStop = false;

    // A EMPLOYER DANS LE CAS DE MESURE DE PROFONDEUR ( SUIVI HOMME )
    //control.typeMeasure = MEASURE_DEPTH;
    //control.depthMin = 0.6;
    //control.depthMax = 2;
    // A EMPLOYER DANS LE CAS DE MESURE DE COULEUR ( SUIVI LIGNE )
    control.typeMeasure = MEASURE_RGB;
    //--------------------------------------------
    // NE PAS MODIFIER
    //-------------------------------------------
    // constantes
    control.R = 0.07 / 2;                   // rayon des roues en m
    control.Ly = 0.23;                      // largeur essieu en m ( dist entre milieu des roues )
    control.Lx =  0;               // distance mesure camera RGB suivant l'axe Xe du robot
    double reduction = 6545.0 / 132.0;      // rapport de reduction
    double tickByTour = 52 * reduction;     // nb pas codeur / tour de roue
    control.encToRad = 2 * PI / tickByTour; // angles roues  gauche, droite= control.encToRad *  left_encoder , control.encToRad *  right_encoder
    control.TECH=0.02; control.time=0; // init sample time and current tiem, do not change
    #ifdef USE_OPTITRACK_CAMERAS
      control.printOptitrackData=false;
    #endif
    // valeurs initiales des signaux
    control.ref_vx = 0; // ref_vx from key board
    control.vx = 0;     // vx applied to kobuki
    control.ref_wz = 0; // ref_wz from keyboard
    control.wz = 0;     // wz applied to kobuki
    control.measure = 0;
    control.measureOk = false;
    //-----------------------------------------------------------------------
    // initialisation de vos variables ( a declarer dans control.hpp ) ici
    //-------------------------------------------------------------------------
    // exemple : init lowpass filter example : (1+0.p)/ (1+0.1.p) = ( n0+n1.p)/(d0 +d1.p)  , with sample time control.TECH
    bool ok = initFilter(control.lowpassExample, 1, 0, 1, 0.1, control.TECH);
    // user measures exemple, a changer a votre guise
    control.nbUserMeasure = 0;
}

void oneStepControl(ControlStruct &control)
{

    //---------------------------------------------------------------------------------------
    // VOTRE CODE DE COMMANDE ICI
    // you have to compute control.vx and and control.wz to follow a line, corresponding to control.measure=0;
    // ControlStruct is defined in control.hpp file and can be modified
    //---------------------------------------------------------------------------------------------
    // exemple : les commande vx et wz recopient les valeurs rentrees au clavier 
    double vxFiltered = oneStepFilter(control.lowpassExample, control.ref_vx); // compute the output of the lowpass filter with input ref_vx
    control.vx = control.ref_vx;                                               // vx applied to kobuki <- ref_vx from keyboard
    control.wz = control.ref_wz;                                               // wz applied to kobuki <- ref_wz from keyboard
 
 
    //------------------------------------------
    // affichage des donnees a la console
    //------------------------------------------
    bool okPrint=control.measureOk;
    #ifdef USE_OPTITRACK_CAMERAS
    okPrint =okPrint && (!control.printOptitrackData);
    #endif
    if (okPrint)
    {
        if (control.typeMeasure==MEASURE_RGB) {
           printf("frame_count=%lld, ecartY = %.3f m at X=%.3f m \n", control.frame_count,control.measure, control.Lx);
        };
        if (control.typeMeasure==MEASURE_DEPTH) {
           printf("frame_count=%lld, mean depth = %.3f at angle =%.3f degres \n", control.frame_count,control.measure, control.measureAngleDepth * 180 / PI);
        };

    }

#ifdef USE_OPTITRACK_CAMERAS
    structOptitrack *so = &control.optitrackData;
    if (control.printOptitrackData) {
        for (int i = 0; i < so->nbMarkers; i++)
        {
            printf("  %d,%.3f,%.3f,%.3f,%.2f \n",
                   i+1, so->xm[i], so->ym[i], so->zm[i], control.time);
        }
    }
#endif
control.time+=control.TECH;
}
void endControl(ControlStruct &control)
{
    control.vx = 0;
    control.wz = 0;
}
/*------------------------------------------------------------------------------
 Key board reactions : increment or decrement speeds
 ------------------------------------------------------------------------------*/
#define SPEED_MAX 0.75 /* max speed in m/s */
bool setSpeedRef(ControlStruct &control, double ref_vx, double ref_wz)
{
    double left_speed = ref_vx - ref_wz * control.Ly / 2;
    if (left_speed < -SPEED_MAX)
        return false;
    if (left_speed > SPEED_MAX)
        return false;
    double right_speed = ref_vx + ref_wz * control.Ly / 2;
    if (right_speed < -SPEED_MAX)
        return false;
    if (right_speed > SPEED_MAX)
        return false;
    control.ref_vx = ref_vx;
    control.ref_wz = ref_wz;
    return true;
}
#define STEP_WZ (0.1 * SPEED_MAX / control.Ly) /* rotation speed step */
#define STEP_VX (0.1 * SPEED_MAX)              /* linear speed step */
void handleRobotArm(ControlStruct &control) {
    
}

void handleKeyboard(ControlStruct &control, char key)
{
    bool ok;
    switch (key)
    {
    case '4': // kobuki_msgs::KeyboardInput::KEYCODE_LEFT:
    {
        ok = setSpeedRef(control, control.ref_vx, control.ref_wz + STEP_WZ);
        break;
    }
    case '6': // kobuki_msgs::KeyboardInput::KEYCODE_RIGHT:
    {
        ok = setSpeedRef(control, control.ref_vx, control.ref_wz - STEP_WZ);
        break;
    }
    case '8': // kobuki_msgs::KeyboardInput::KEYCODE_UP:
    {
        ok = setSpeedRef(control, control.ref_vx + STEP_VX, control.ref_wz);
        break;
    }
    case '2': // kobuki_msgs::KeyboardInput::KEYCODE_DOWN:
    {
        ok = setSpeedRef(control, control.ref_vx - STEP_VX, control.ref_wz);
        break;
    }
    case ' ': // kobuki_msgs::KeyboardInput::KEYCODE_SPACE:
    {
        // resetVelocity();
        ok = setSpeedRef(control, 0, 0);
        control.vx = 0;
        control.wz = 0;
        break;
    }
    #ifdef USE_OPTITRACK_CAMERAS
    case 'm':control.printOptitrackData=!control.printOptitrackData;
    if (control.printOptitrackData) {
       system("clear"); // effacement de la console
    }
    break;
    #endif
    }
}