#ifndef COMMON_H
#define COMMON_H

#include "define.h"
// pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudN;
typedef CloudN::Ptr CloudNPtr;


#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI 3.141592
#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    ++count; \
    if (pcl::getTime() - last >= 1.0) \
{ \
    double now = pcl::getTime (); \
    std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
    count = 0; \
    last = now; \
    } \
    }while(false)
#else
#define FPS_CALC(_WHAT_) \
    do \
{ \
    }while(false)
#endif

inline void rgbTohsv(int r, int g, int b, float &hr, float &sr, float &vr)
{
  float rd, gd, bd, h, s, v, max, min, del, rc, gc, bc;

  /* convert RGB to HSV */
  rd = (float)r / 255.0;            /* rd,gd,bd range 0-1 instead of 0-255 */
  gd = (float)g / 255.0;
  bd = (float)b / 255.0;

  /* compute maximum of rd,gd,bd */
  if (rd>=gd) { if (rd>=bd) max = rd;  else max = bd; }
         else { if (gd>=bd) max = gd;  else max = bd; }

  /* compute minimum of rd,gd,bd */
  if (rd<=gd) { if (rd<=bd) min = rd;  else min = bd; }
         else { if (gd<=bd) min = gd;  else min = bd; }

  del = max - min;
  v = max;
  if (max != 0.0) s = (del) / max;
             else s = 0.0;

  if (s != 0.0) {
    rc = (max - rd) / del;
    gc = (max - gd) / del;
    bc = (max - bd) / del;

    if      (rd==max) h = bc - gc;
    else if (gd==max) h = 2 + rc - bc;
    else if (bd==max) h = 4 + gc - rc;

    h = h * 60;
    if (h<0) h += 360;
  }
  h /=360.f;

  hr = h;  sr = s;  vr = v;
}

inline float colordist(PointNT a, PointNT b)
{
    float h1,s1,v1,h2,s2,v2;
    rgbTohsv(a.r,a.g,a.b,h1,s1,v1);
    rgbTohsv(b.r,b.g,b.b,h2,s2,v2);

//    if(s1>0.3 && v1>0.3){ v1=0; }
//    else{h1 = -2.f;}
//    if(s2>0.3 && v2>0.3){ v2=0; }
//    else{h2 = -2.f;}
//    float hdist = fabs(h1-h2);
//    while (hdist>0.5f) {hdist -= 0.5f;}
//    float vdist = fabs(v1-v2);
//    return (hdist*2 + vdist)/2.f;
    float hdist = fabs(h1-h2);
    while (hdist>0.5f) {hdist -= 0.5f;}
    return(hdist/0.5f);

}


inline void transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
    double mv[12];
    bt.getBasis ().getOpenGLSubMatrix (mv);

    tf::Vector3 origin = bt.getOrigin ();

    out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
    out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
    out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

    out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
    out_mat (0, 3) = origin.x ();
    out_mat (1, 3) = origin.y ();
    out_mat (2, 3) = origin.z ();
}

#endif // COMMON_H

