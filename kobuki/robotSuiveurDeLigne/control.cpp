/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "tustin.hpp"
#include "control.hpp"
#include <cstdio>                 // to use printf as in c, because std:cout is redirected to file and is not usable
#define PI 3.14159265358979323846 /* pi */

void initControl(ControlStruct &control)
{
    // undocumented kobuki internal pid parameters, initialized to default values
    control.p_gain = 1 * 100 * 1000;
    control.i_gain = 0.1 * 1000;
    control.d_gain = 2 * 1000;
    // emergencyStop is false when starting control
    control.emergencyStop = false;

    // A EMPLOYER DANS LE CAS DE MESURE DE PROFONDEUR ( SUIVI HOMME )
    // control.typeMeasure = MEASURE_DEPTH;
    // control.depthMin = 0.6;
    // control.depthMax = 1.2;
    // A EMPLOYER DANS LE CAS DE MESURE DE COULEUR ( SUIVI LIGNE )
    control.typeMeasure = MEASURE_RGB;

    // constantes
    control.R = 0.07 / 2;                   // rayon des roues en m
    control.Ly = 0.23;                      // largeur essieu en m ( dist entre milieu des roues )
    control.Lx = 0.25 + 0.17;               // distance mesure camera RGB suivant l'axe Xe du robot
    double reduction = 6545.0 / 132.0;      // rapport de reduction
    double tickByTour = 52 * reduction;     // nb pas codeur / tour de roue
    control.encToRad = 2 * PI / tickByTour; // angles roues  gauche, droite= control.encToRad *  left_encoder , control.encToRad *  right_encoder
    // valeurs initiales des signaux
    control.ref_vx = 0; // ref_vx from key board
    control.vx = 0;     // vx applied to kobuki
    control.ref_wz = 0; // ref_wz from keyboard
    control.wz = 0;     // wz applied to kobuki
    control.measure = 0;
    control.measureOk = false;
    float Tech = 0.02;
    // init lowpass filter example : 1/ (1+0.1.p) = ( n0p+n1p.p)/(d0p +d1p.p)  , with sample time Te=0.02 s
    bool ok = initFilter(control.lowpassExample, 1, 0, 1, 0.1, Tech);
    // user measures exemple
    control.nbUserMeasure = 0;
    // modele 1/p
    float wu = 5, wf = 5 * wu;
    ok = initFilter(control.regAngle, wu, 0, 1, 1 / wf, Tech);
}

void oneStepControl(ControlStruct &control)
{
    static long long lastFrameCount = -1;
#ifdef USE_OPTITRACK_CAMERAS
    static int optFrame = 0;
#endif
    // you have to compute control.vx and and control.wz to follow a line, corresponding to control.measure=0;
    // ControlStruct is defined in control.hpp file and can be modified
    double vxFiltered = oneStepFilter(control.lowpassExample, control.ref_vx); // compute the output of the lowpass filter with input ref_vx
    control.vx = control.ref_vx;                                               // vx applied to kobuki <- ref_vx from keyboard
    control.wz = control.ref_wz;                                               // wz applied to kobuki <- ref_wz from keyboard
                                                                               // control.vx=vxFiltered ;
                                                                               /*if (control.measureOk)
                                                                               {
                                                                                   printf("measure = %.3f m, angle =%.3f degres \n", control.measure, control.measureAngleDepth * 180 / PI);
                                                                               }
                                                                               */
    if (control.measureOk)
    {
        float ref_angle=0,angle = -control.measure / control.Lx;

        float wzAuto = oneStepFilter(control.regAngle, ref_angle- angle);
        if (control.frame_count != lastFrameCount)
        {
            lastFrameCount = (long long)control.frame_count;
            printf("wz=%.2f , wZAuto=%.3f, frame  =%.8ld , angle=%.3f, meanY =%.3f \n", control.wz * 180 / PI, wzAuto *180/PI, (long)control.frame_count, angle, control.measure);
        }

        control.wz += wzAuto;
    }
#ifdef USE_OPTITRACK_CAMERAS

    structOptitrack *so = &control.optitrackData;
    if (so->frame != optFrame)
    {
        optFrame = so->frame;
        printf(" opt frame=%d , nb rigid bodies=%d , nb markers =%d\n", so->frame, so->nbRigidBodies, so->nbMarkers);
        for (int i = 0; i < so->nbRigidBodies; i++)
        {
            printf("  rigid[%d] =[%.3f,%.3f,%.3f] , q=[%.3f,%.3f,%.3f,%.3f]\n",
                   i, so->xb[i], so->yb[i], so->zb[i], so->qxb[i], so->qyb[i], so->qzb[i], so->qwb[i]);
        }
        for (int i = 0; i < so->nbMarkers; i++)
        {
            printf("  marker[%d] =[%.3f,%.3f,%.3f] \n",
                   i, so->xm[i], so->ym[i], so->zm[i]);
        }
    }
#endif
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
void handleKeyboard(ControlStruct &control, char key)
{
    bool ok;
    switch (key)
    {
    case 68: // kobuki_msgs::KeyboardInput::KEYCODE_LEFT:
    {
        ok = setSpeedRef(control, control.ref_vx, control.ref_wz + STEP_WZ);
        break;
    }
    case 67: // kobuki_msgs::KeyboardInput::KEYCODE_RIGHT:
    {
        ok = setSpeedRef(control, control.ref_vx, control.ref_wz - STEP_WZ);
        break;
    }
    case 65: // kobuki_msgs::KeyboardInput::KEYCODE_UP:
    {
        ok = setSpeedRef(control, control.ref_vx + STEP_VX, control.ref_wz);
        break;
    }
    case 66: // kobuki_msgs::KeyboardInput::KEYCODE_DOWN:
    {
        ok = setSpeedRef(control, control.ref_vx - STEP_VX, control.ref_wz);
        break;
    }
    case 32: // kobuki_msgs::KeyboardInput::KEYCODE_SPACE:
    {
        // resetVelocity();
        ok = setSpeedRef(control, 0, 0);
        control.vx = 0;
        control.wz = 0;
        break;
    }
    }
}