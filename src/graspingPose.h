#ifndef GRASPING_POSE_H_
#define GRASPING_POSE_H_
#include "myGeometryUtils.h"

class graspingPose
{
  public:
    double getX();
    double getY();
    double getZ();
    double getA();
    double getB();
    double getC();
    void setMarkerNearestPose(struct marker &myMarker);
    void setMarker(struct marker &myMarker,int i);
    double getError();
    void update(AffineTransformation &input);
    graspingPose(AffineTransformation &input);
    graspingPose();

  private:
    AffineTransformation transfo;
    AffineTransformation globalTransfo;
    double x,y,z,a,b,c;
};
#endif