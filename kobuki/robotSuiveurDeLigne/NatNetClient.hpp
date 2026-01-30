


#define NB_MAX_BODIES 100
#define NB_MAX_MARKERS 100
typedef struct
{
  int type;
  int frame;
  //-----------------------
  // rigidBodies Data
  //-----------------------
  int nbRigidBodies; // number of bodies
  // position of each body
  float xb[NB_MAX_BODIES];
  float yb[NB_MAX_BODIES];
  float zb[NB_MAX_BODIES];
  // orientation quaternion of each body
  float qxb[NB_MAX_BODIES];
  float qyb[NB_MAX_BODIES];
  float qzb[NB_MAX_BODIES];
  float qwb[NB_MAX_BODIES];
  //-----------------------
  // markers Data
  //-----------------------
  int nbMarkers;
  float xm[NB_MAX_MARKERS];
  float ym[NB_MAX_MARKERS];
  float zm[NB_MAX_MARKERS];
  
} structOptitrack;
#ifndef IN_NATNET_CPP
extern bool connectToOptitrack(void *handleData,void *params);
#endif