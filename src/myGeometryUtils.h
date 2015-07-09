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
#include "quaternions.h"
#include "GeometryUtils.h"
#include "SL_common.h"
#include <math.h>

static void matToQuat(Matrix cMo, double &x, double &y, double &z, double &w);


class RotationMatrix{
public:
  RotationMatrix();
  RotationMatrix(double angle, int axis);
  Matrix getMatrix();
private:
  Matrix m_matrix;
};

class AffineTransformation{
public:
  Vector getTranslation();
  Matrix getMatrix();
  AffineTransformation();/*angles to go from the old frame to the new one*/
  AffineTransformation(Vector v, Eigen::Matrix3d m);/*angles to go from the old frame to the new one*/
  AffineTransformation(Vector translation, double alpha,double beta, double gamma);/*angles to go from the old frame to the new one*/
  AffineTransformation(double tx, double ty, double tz, double alpha,double beta, double gamma);/*angles to go from the old frame to the new one*/
  AffineTransformation(AffineTransformation &a);/*angles to go from the old frame to the new one*/
  Vector apply(Vector X);
  Vector apply(double a1,double a2,double a3);
  void compose(AffineTransformation &a);
private:
  Vector m_translation;
  Matrix m_matrix;
};

class cameraToRobot{
public:
  cameraToRobot();
  void update(struct marker &myMarker);
private:
  AffineTransformation camera;
  AffineTransformation cameraNeck;
  AffineTransformation necky;
  AffineTransformation neckx;
  AffineTransformation neckz;
  AffineTransformation neckzBase;
  AffineTransformation neck;
  AffineTransformation book;
  AffineTransformation transfon;
  AffineTransformation transfornp1;
  AffineTransformation Rx;
  AffineTransformation patch;
  Vector out;
};