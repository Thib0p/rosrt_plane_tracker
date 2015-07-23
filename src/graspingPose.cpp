#include "quaternions.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_unix_common.h"
#include "utility_macros.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_common.h"
#include "GeometryUtils.h"
#include "SL_common.h"
#include <math.h>
#include "graspingPose.h"
#include "myGeometryUtils.h"
#include "mycube.h"

graspingPose::graspingPose()
{
}
graspingPose::graspingPose(AffineTransformation &input)
{
transfo = AffineTransformation(input);
}

void graspingPose::update(AffineTransformation &input)
{

input.composeTo(transfo, globalTransfo);
}

double graspingPose::getError()
{
  double Ko = 1.0;
  Vector stablePose = my_vector(1,3);
  stablePose[1]=0.3;
  stablePose[2]=0.7;
  stablePose[3]=-0.3;
  Vector quaternions = my_vector(1,4);
  Vector posHand = my_vector(1,3);
  posHand = globalTransfo.getTranslation();
  matToQuat(globalTransfo.getMatrix(),quaternions[1],quaternions[2],quaternions[3],quaternions[4]);

  Eigen::Quaterniond currentQuat(cart_orient[1].q[_Q0_],cart_orient[1].q[_Q1_],cart_orient[1].q[_Q2_],cart_orient[1].q[_Q3_]);
  Eigen::Quaterniond targetQuat(quaternions[4],quaternions[1],quaternions[2],quaternions[3]);
  targetQuat.normalize();
  Eigen::Vector3d eulerAngles;
  inverse_kinematics::quatLogError(targetQuat,currentQuat,eulerAngles);

  return (posHand[1]-stablePose[1])*(posHand[1]-stablePose[1])+4*(posHand[2]-stablePose[2])*(posHand[2]-stablePose[2])+(posHand[3]-stablePose[3])*(posHand[3]-stablePose[3])+
  0.05*((cart_state[1].x[1]-posHand[1])*(cart_state[1].x[1]-posHand[1])
       + (cart_state[1].x[2]-posHand[2])*(cart_state[1].x[2]-posHand[2])
       + (cart_state[1].x[3]-posHand[3])*(cart_state[1].x[3]-posHand[3])

       + Ko*(eulerAngles[0]*eulerAngles[0] +eulerAngles[1]*eulerAngles[1]+ eulerAngles[2]*eulerAngles[2]));

}

void graspingPose::setMarkerNearestPose(struct marker &myMarker)
{
  Vector out = my_vector(1,3);
   out = globalTransfo.apply(0,0,0);
   myMarker.xhand=out[1];
   myMarker.yhand=out[2];
   myMarker.zhand=out[3];

  matToQuat(globalTransfo.getMatrix(),myMarker.xqhand,myMarker.yqhand,myMarker.zqhand,myMarker.wqhand);


  // printf("position : %f\n%f\n%f\n",myMarker.xhand,myMarker.yhand,myMarker.zhand);
}
void graspingPose::setMarker(struct marker &myMarker,int i)
{
  Vector out = my_vector(1,3);
   out = globalTransfo.apply(0,0,0);
   myMarker.xpose[i]=out[1];
   myMarker.ypose[i]=out[2];
   myMarker.zpose[i]=out[3];

  matToQuat(globalTransfo.getMatrix(),myMarker.xqpose[i],myMarker.yqpose[i],myMarker.zqpose[i],myMarker.wqpose[i]);
}