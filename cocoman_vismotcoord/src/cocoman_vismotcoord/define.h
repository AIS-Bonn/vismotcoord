#ifndef DIFINE_H
#define DIFINE_H

#define DEBUG_MAIN 0
#define DEBUG_ALGORITHM 0

#define USENORM 1

//#define GRIDSIZE 0.002
//#define GRIDSIZE 0.003
//#define GRIDSIZE 0.004
#define GRIDSIZE 0.005  // best
//#define GRIDSIZE 0.010

//#define NPARTICLE 128
//#define NPARTICLE 256
//#define NPARTICLE 512
#define NPARTICLE 1024  // best

// 5%
//#define VARTRANS 0.0025f     // max: 0.5
//#define VARROT 0.9f          // max: 180
//#define VARTRANS 0.0035f     // max: 0.5
//#define VARROT 1.2f          // max: 180
// 10%: best
#define VARTRANS 0.005f     // max: 0.5
#define VARROT 1.8f          // max: 180
// 20%
//#define VARTRANS 0.01f     // max: 0.5
//#define VARROT 3.6f              // max: 180
// 30%
//#define VARTRANS 0.015f     // max: 0.5
//#define VARROT 5.4f          // max: 180

#define PUB_EVAL 0
#define PUB_VISMODEL 0
#define PUB_TIME 0
//#define PUB_
#endif // DEFINE_H
