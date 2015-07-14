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
#include "myGeometryUtils.h"
#include "mycube.h"

  RotationMatrix::RotationMatrix()
  {
    m_matrix= my_matrix(1,3,1,3);
  }
  RotationMatrix::RotationMatrix(double angle, int axis)
  {
   m_matrix= my_matrix(1,3,1,3);
   switch(axis){
    case 1:
    m_matrix[1][1]=1;
    m_matrix[1][2]=0;
    m_matrix[1][3]=0;

    m_matrix[2][1]=0;
    m_matrix[2][2]=cos(angle);
    m_matrix[2][3]=sin(angle);

    m_matrix[3][1]=0;
    m_matrix[3][2]=-sin(angle);
    m_matrix[3][3]=cos(angle);
    break;
    case 2:
    m_matrix[1][1]=cos(angle);
    m_matrix[1][2]=0;
    m_matrix[1][3]=-sin(angle);

    m_matrix[2][1]=0;
    m_matrix[2][2]=1;
    m_matrix[2][3]=0;

    m_matrix[3][1]=sin(angle);
    m_matrix[3][2]=0;
    m_matrix[3][3]=cos(angle);
    break;
    case 3:
    m_matrix[1][1]=cos(angle);
    m_matrix[1][2]=-sin(angle);
    m_matrix[1][3]=0;

    m_matrix[2][1]=sin(angle);
    m_matrix[2][2]=cos(angle);
    m_matrix[2][3]=0;

    m_matrix[3][1]=0;
    m_matrix[3][2]=0;
    m_matrix[3][3]=1;
    break;
  }
}
Matrix RotationMatrix::getMatrix(){
  return m_matrix;
}



  Vector AffineTransformation::getTranslation()
  {
    return m_translation;
  }
  Matrix AffineTransformation::getMatrix()
  {
    return m_matrix;
  }
    AffineTransformation::AffineTransformation()/*angles to go from the old frame to the new one*/
  {
    m_matrix = my_matrix(1,3,1,3);
    m_translation = my_vector(1,3);
  }

      AffineTransformation::AffineTransformation(Vector v, Eigen::Matrix3d m)/*angles to go from the old frame to the new one*/
  {
    m_matrix = my_matrix(1,3,1,3);
    m_translation = my_vector(1,3);

    for (int i = 0; i < 3; ++i)
    {
     for (int j = 0; j < 3; ++j)
     {
      m_matrix[i+1][j+1]=m(i,j);
    } 
    m_translation[i+1]=v[i+1];
  }
}

      AffineTransformation::AffineTransformation(double tx,double ty,double tz, Eigen::Matrix3d m)/*angles to go from the old frame to the new one*/
  {
    m_matrix = my_matrix(1,3,1,3);
    m_translation = my_vector(1,3);
m_translation[1]=tx;
m_translation[2]=ty;
m_translation[3]=tz;
    for (int i = 0; i < 3; ++i)
    {
     for (int j = 0; j < 3; ++j)
     {
      m_matrix[i+1][j+1]=m(i,j);
    } 
  }
}
  AffineTransformation::AffineTransformation(Vector translation, double alpha,double beta, double gamma)/*angles to go from the old frame to the new one*/
{
  m_matrix = my_matrix(1,3,1,3);
  m_translation=translation;
  RotationMatrix Rx(alpha,1);
  RotationMatrix Ry(beta,2);
  RotationMatrix Rz(gamma,3);
  Matrix buff = my_matrix(1,3,1,3);
  mat_mult(Rz.getMatrix(),Ry.getMatrix(),buff);
  mat_mult(buff,Rx.getMatrix(),m_matrix);
}
    AffineTransformation::AffineTransformation(double tx, double ty, double tz, double alpha,double beta, double gamma)/*angles to go from the old frame to the new one*/
{
  m_matrix = my_matrix(1,3,1,3);
  m_translation = my_vector(1,3);
  m_translation[1]=tx;
  m_translation[2]=ty;
  m_translation[3]=tz;

  RotationMatrix Rx(alpha,1);
  RotationMatrix Ry(beta,2);
  RotationMatrix Rz(gamma,3);
  Matrix buff = my_matrix(1,3,1,3);
  mat_mult(Rz.getMatrix(),Ry.getMatrix(),buff);
  mat_mult(buff,Rx.getMatrix(),m_matrix);
}
  AffineTransformation::AffineTransformation(AffineTransformation &a)/*angles to go from the old frame to the new one*/
{
  m_matrix = my_matrix(1,3,1,3);
  m_translation = my_vector(1,3);
  Matrix tempMatrix = my_matrix(1,3,1,3);
  Vector tempTranslation = my_vector(1,3);
  tempMatrix = a.getMatrix();
  tempTranslation = a.getTranslation();
  m_translation[1]=tempTranslation[1];
  m_translation[2]=tempTranslation[2];
  m_translation[3]=tempTranslation[3];
  for (int i = 1; i <= 3; ++i)
  {
    for (int j = 1; j <= 3; ++j)
    {
      m_matrix[i][j]=tempMatrix[i][j];
    }
  }

}
Vector AffineTransformation::apply(Vector X)
{
  Vector out = my_vector(1,3);
  mat_vec_mult(m_matrix,X,out);
  out[1] += m_translation[1];
  out[2] += m_translation[2];
  out[3] += m_translation[3];
  return out;
}
Vector AffineTransformation::apply(double a1,double a2,double a3)
{
  Vector in = my_vector(1,3);
  in[1]=a1;
  in[2]=a2;
  in[3]=a3;
  Vector out = my_vector(1,3);
  mat_vec_mult(m_matrix,in,out);
  out[1] += m_translation[1];
  out[2] += m_translation[2];
  out[3] += m_translation[3];
  return out;
}
void AffineTransformation::compose(AffineTransformation &a)
{
  Vector translationTemp=my_vector(1,3);
  mat_vec_mult(m_matrix,a.getTranslation(),translationTemp);
  m_translation[1] +=  translationTemp[1];
  m_translation[2] +=  translationTemp[2];
  m_translation[3] +=  translationTemp[3];
  mat_mult(m_matrix,a.getMatrix(),m_matrix);
}


 cameraToRobot::cameraToRobot()
 {
  cameraNeck=AffineTransformation(-0.02,-0.205, -0.02 ,0, 0, 0);
  camera=AffineTransformation(0,0,0,0.436, 0, 0);
  neck = AffineTransformation(0,0,0,0,-M_PI/2,0);
  patch=AffineTransformation(0,0, 0 ,0, M_PI, 0);
}
  void cameraToRobot::update(struct marker &myMarker)
  {
    
    necky = AffineTransformation(0,0,0,0,-joint_state[B_HR].th,0);
    neckx = AffineTransformation(0,0,0,-joint_state[B_HT].th,0,0);
    neckz = AffineTransformation(0,0,0,0,0,joint_state[B_HN].th);
    neckzBase = AffineTransformation(joint_origin_pos[B_HR][_X_],joint_origin_pos[B_HR][_Y_],joint_origin_pos[B_HR][_Z_],M_PI/2,0,M_PI/2);
    transfon = AffineTransformation(neckzBase);
    grasping_pose = AffineTransformation(0,-.2,+0.13,0,-M_PI/2,M_PI/2);
    transfon.compose(neckz);
    transfon.compose(neckx);
    transfon.compose(necky);
    transfon.compose(neck);
    transfon.compose(camera);
    transfon.compose(cameraNeck);
    out = transfon.apply(0,0,0);
    Vector zk = transfon.apply(0,0,1);
    zk[1] -= out[1];
    zk[2] -= out[2];
    zk[3] -= out[3];
    myMarker.xkin = out[1];
    myMarker.ykin = out[2];
    myMarker.zkin = out[3];
   double kxp=myMarker.x,kyp=myMarker.y,kzp=myMarker.z;
    out = transfon.apply(myMarker.x,myMarker.y,myMarker.z);
       myMarker.x=out[1];
   myMarker.y=out[2];
   myMarker.z=out[3];
    Eigen::Quaterniond targetQuat2(myMarker.wq,myMarker.xq,myMarker.yq,myMarker.zq);
    targetQuat2.normalize();
    Eigen::Matrix3d eigentransfo= targetQuat2.toRotationMatrix();
    movenp1 = AffineTransformation(kxp,kyp,kzp,0,0,0);
    transfornp1 = AffineTransformation(0,0,0, eigentransfo);

    matToQuat(transfon.getMatrix(),myMarker.xqkin,myMarker.yqkin,myMarker.zqkin,myMarker.wqkin);

    transfon.compose(movenp1);
    transfon.compose(transfornp1);
    /*Vector yp = transfon.apply(0,1,0);
    Vector zp = transfon.apply(0,0,1);
    Vector planeOrigin = transfon.apply(0,0,0);
    zp[1] -= planeOrigin[1];
    zp[2] -= planeOrigin[2];
    zp[3] -= planeOrigin[3];

    yp[1] -= planeOrigin[1];
    yp[2] -= planeOrigin[2];
    yp[3] -= planeOrigin[3];
    double dpz = zp[1]*zk[1]+zp[2]*zk[2]+zp[3]*zk[3];
    double dpy = yp[2]*zk[2];
    if(dpz>0)
    {
     if(dpy>0)
     {
       Rx=AffineTransformation(0,0,0,M_PI,0,0);
       transfon.compose(Rx);
     }
     else
     {
       Rx=AffineTransformation(0,0,0,0,M_PI,0);
       transfon.compose(Rx);
     }
   }*/
    
   matToQuat(transfon.getMatrix(),myMarker.xq,myMarker.yq,myMarker.zq,myMarker.wq);




/*    Vector yp = transfon.apply(0,1,0);
    Vector zp = transfon.apply(0,0,1);
    Vector planeOrigin = transfon.apply(0,0,0);
    zp[1] -= planeOrigin[1];
    zp[2] -= planeOrigin[2];
    zp[3] -= planeOrigin[3];

    yp[1] -= planeOrigin[1];
    yp[2] -= planeOrigin[2];
    yp[3] -= planeOrigin[3];
    double dpz = zp[1]*zk[1]+zp[2]*zk[2]+zp[3]*zk[3];
    double dpy = yp[2]*zk[2];*/










   transfon.compose(grasping_pose);
   out = transfon.apply(0,0,0);
   myMarker.xhand=out[1];
   myMarker.yhand=out[2];
   myMarker.zhand=out[3];

  matToQuat(transfon.getMatrix(),myMarker.xqhand,myMarker.yqhand,myMarker.zqhand,myMarker.wqhand);
  
  }  



namespace inverse_kinematics
{

  bool normalizeQuaternion(SL_quat& q)
  {
    double denom = 0.0;
    for (int i=1; i<=N_QUAT; ++i)
      denom += sqr(q.q[i]);

  //check that the denom is close to one
    if( (denom < 0.5) || (denom > 1.5) )
    {
      printf("\n");
      printf("Quaternion denom = %f\n", denom);
      return false;
    }

    double mult = 1.0 / denom;

    for (int i=1; i<=N_QUAT; ++i)
    {
      q.q[i] *= mult;
      q.qd[i] *= mult;
      q.qdd[i] *= mult;
    }

    return true;
  }

  void quatToAngularAcceleration(SL_quat& q)
  {
    double Q[4+1][3+1];
    double Qd[4+1][3+1];
    double halfQdW[4+1];

    Q[1][1] = -q.q[_Q1_];
    Q[1][2] = -q.q[_Q2_];
    Q[1][3] = -q.q[_Q3_];

    Q[2][1] =  q.q[_Q0_];
    Q[2][2] =  q.q[_Q3_];
    Q[2][3] = -q.q[_Q2_];

    Q[3][1] = -q.q[_Q3_];
    Q[3][2] =  q.q[_Q0_];
    Q[3][3] =  q.q[_Q1_];

    Q[4][1] =  q.q[_Q2_];
    Q[4][2] = -q.q[_Q1_];
    Q[4][3] =  q.q[_Q0_];

    Qd[1][1] = -q.qd[_Q1_];
    Qd[1][2] = -q.qd[_Q2_];
    Qd[1][3] = -q.qd[_Q3_];

    Qd[2][1] =  q.qd[_Q0_];
    Qd[2][2] =  q.qd[_Q3_];
    Qd[2][3] = -q.qd[_Q2_];

    Qd[3][1] = -q.qd[_Q3_];
    Qd[3][2] =  q.qd[_Q0_];
    Qd[3][3] =  q.qd[_Q1_];

    Qd[4][1] =  q.qd[_Q2_];
    Qd[4][2] = -q.qd[_Q1_];
    Qd[4][3] =  q.qd[_Q0_];

    for (int i=1; i<=N_QUAT; i++)
    {
      halfQdW[i]=0.0;
      for (int j=1; j<=N_CART; j++)
        halfQdW[i] += 0.5*Qd[i][j]*q.ad[j];
    }

    for (int i=1; i<=N_CART; i++)
    {
      q.add[i]=0.0;
      for (int j=1; j<=N_QUAT; j++)
        q.add[i] += 2.0*Q[j][i]*(q.qdd[j] - halfQdW[j]);
    }

  }

  void quatToAngularVelAcc(SL_quat& q)
  {
    quatToAngularVelocity(&q);
    quatToAngularAcceleration(q);
  }

  void fixQuaternionSign(const SL_quat &q1, SL_quat &q2)
  {
    double dot=0.0;
    for (int j=1; j<=N_QUAT; j++)
    {
      dot+=q1.q[j]*q2.q[j];
    }
    if (dot<0)
    {
      for (int j=1; j<=N_QUAT; j++)
      {
        q2.q[j]=-q2.q[j];
        q2.qd[j]=-q2.qd[j];
        q2.qdd[j]=-q2.qdd[j];
      }
      quatToAngularVelAcc(q2);
    }
  }

  void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2)
  {
    Eigen::Vector3d w1,w2;
    w1.setZero();
    w2.setZero();
    fixQuaternionSign(q1,q2,w1,w2);
  }

  void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
   const Eigen::Vector3d& w1, Eigen::Vector3d& w2)
  {
    Eigen::Vector3d a1,a2;
    a1.setZero();
    a2.setZero();
    fixQuaternionSign(q1,q2,w1,w2,a1,a2);
  }



  void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
   const Eigen::Vector3d& w1, Eigen::Vector3d& w2,
   const Eigen::Vector3d& a1, Eigen::Vector3d& a2)
  {
    SL_quat sl_q1, sl_q2;
    floating_base_utilities::GeometryUtils::EigQuaternionToSL(q1, sl_q1.q);
    floating_base_utilities::GeometryUtils::EigQuaternionToSL(q2, sl_q2.q);

    Eigen::Map<Eigen::Vector3d>(&(sl_q1.ad[1])) = w1;
    Eigen::Map<Eigen::Vector3d>(&(sl_q2.ad[1])) = w2;
    Eigen::Map<Eigen::Vector3d>(&(sl_q1.add[1])) = a1;
    Eigen::Map<Eigen::Vector3d>(&(sl_q2.add[1])) = a2;

    quatDerivatives(&sl_q1);
    quatDerivatives(&sl_q2);

    fixQuaternionSign(sl_q1, sl_q2);

  //map back the values
    floating_base_utilities::GeometryUtils::SLQuaternionToEigen(sl_q2.q, q2);
    w2 = Eigen::Map<Eigen::Vector3d>(&(sl_q2.ad[1]));
    a2 = Eigen::Map<Eigen::Vector3d>(&(sl_q2.add[1]));
  }

  void quatLogError(const Eigen::Quaterniond& q_desired, const Eigen::Quaterniond& q_actual, Eigen::Vector3d& error)
  {
    Eigen::Quaterniond q1 = q_desired;
    Eigen::Quaterniond q2 = q_actual;

    q1.normalize();
    q2.normalize();
    inverse_kinematics::fixQuaternionSign(q1, q2);

    Eigen::Quaterniond delta_q = q1 * q2.inverse();

  //make sure the q0 > 0 (to make the sine approx correct)
    if(delta_q.w()<0)
    {
      delta_q.w() = -delta_q.w();
      delta_q.x() = -delta_q.x();
      delta_q.y() = -delta_q.y();
      delta_q.z() = -delta_q.z();
    }

  //use a linear approximation
    if(fabs(1.0-delta_q.w()) < 0.01)
    {
      error = 2.0 * delta_q.vec();
    }
  else //or the sine formulae
  {
    double alpha_norm = 2.0 * acos(delta_q.w());
    error = alpha_norm/sin(alpha_norm/2) * delta_q.vec();
  }
}


}

double setPositionGain(double Ko,double t,double alpha,double lambda)
{
  double a = (tanh(lambda*(-t+alpha))/2+1)*(-Ko)-0.01;
  if(a<0.0)
    return a;
  else
    return 0.0;
}

static void matToQuat(Matrix cMo, double &x, double &y, double &z, double &w)
{
  double trace =cMo[1][1]+cMo[2][2]+cMo[3][3]+1;
  double S;
  if(trace>0)
  {
    S=0.5/sqrt(trace);
    x=(cMo[3][2]-cMo[2][3])*S;
    y=(cMo[1][3]-cMo[3][1])*S;
    z=(cMo[2][1]-cMo[1][2])*S;
    w=0.25/S;
  }
  else
  {
    int max=1;
    for (int i = 2; i < 4; ++i)
    {
      if(cMo[i][i]>cMo[max][max])
      {
        max=i;
      }
    }

    switch(max)
    {
      case 1:
      S=sqrt(1+cMo[1][1]-cMo[2][2]-cMo[3][3])*2;
      x=0.25*S;
      y =(cMo[1][2]+cMo[2][1])/S;
      z = (cMo[1][3]+cMo[3][1])/S;
      w = (cMo[2][3]+cMo[3][2])/S;
      break;
      case 2:
      S=sqrt(1-cMo[1][1]+cMo[2][2]-cMo[3][3])*2;
      x=(cMo[1][2]+cMo[2][1])/S;
      y =0.25*S;
      z = (cMo[2][3]+cMo[3][2])/S;
      w = (cMo[1][3]+cMo[3][1])/S;
      break;
      case 3:
      S=sqrt(1-cMo[1][1]-cMo[2][2]+cMo[3][3])*2;
      x=(cMo[1][3]+cMo[3][1])/S;
      y =(cMo[2][3]+cMo[3][2])/S;
      z = 0.25*S;
      w = (cMo[1][2]+cMo[2][1])/S;
      break;
    }
  }
}