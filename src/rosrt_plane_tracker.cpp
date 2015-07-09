/*============================================================================
==============================================================================

                              orientation.c
 
==============================================================================
Remarks:

ekeleton to create the mygotocart task

============================================================================*/

/* ROS includes*/
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "rosrt/rosrt.h"
// system headers
#include "quaternions.h"
#include "SL_system_headers.h"

/* SL includes */
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
#include "mycube.h"

/*Xenomai includes*/
#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif



#define RATE 1500
#define TOPIC "visualization_marker"
/* defines */
extern "C" void
add_rosrt_plane_tracker( void );
/* local variables */
static double start_time = 0.0;
static double freq;
static double amp;
static SL_DJstate  target[N_DOFS+1];
static int         use_invdyn = TRUE;
static double K = 5.;
static double time_step;
static int countermrj2=-1;
// ros variables
static ros::NodeHandle *handle;
static rosrt::Subscriber<visualization_msgs::Marker> *subscriber;
static visualization_msgs::MarkerConstPtr msg;
/* Here is the desired cartesian states of the 2 hands */
static double targetState[6];
      //static double targetOrientation[6];
static struct marker myMarker;
static Vector targetOrientation = my_vector(1,3);
static Vector targetOrientation2 = my_vector(1,3);
/* Which hands are going to be controlled*/
static int handstatus[2];
static int cstatus[N_ENDEFFS*6];
static double     *cart;
Vector out;
/* global functions */

/* local functions */
static void
drawCoordSystem(double length, double **A, char *name);
      // local functions

static int  init_rosrt_plane_tracker(void);
static bool visualizePole(double angle_measured_);
static int  run_rosrt_plane_tracker(void);
static int  change_rosrt_plane_tracker(void);
int  compute_jacobian(SL_DJstate *state, SL_endeff *eff, Matrix Jacobian,Matrix Jacobian_transpose);
double setPositionGain(double Ko,double t,double alpha,double lambda);
int  setJoints2(void);
static void matToQuat(Matrix cMo, double &x, double &y, double &z, double &w);
void       drawArrow(double *sp, double *ep, double width);

static void init_ros()
{
#ifdef __XENO__
  mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
  
  int argc = 1;
  char name[] = "SL";
  char* argv[1];
  argv[0] = name;
  
  ros::init(argc,argv,"ros_rt_test_subscriber");
  
  
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = "ros_rt_test_subscriber";
  
  handle = new ros::NodeHandle();
  
  rosrt::init(options);
  subscriber = new rosrt::Subscriber<visualization_msgs::Marker>(3, *handle, TOPIC);
   //subscriber->initialize(3,*handle,TOPIC);
}



class RotationMatrix{
public:
  RotationMatrix()
  {
    m_matrix= my_matrix(1,3,1,3);
  }
  RotationMatrix(double angle, int axis)
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
Matrix getMatrix(){
  return m_matrix;
}
private:
  Matrix m_matrix;
};

class AffineTransformation{
public:
  Vector getTranslation()
  {
    return m_translation;
  }
  Matrix getMatrix()
  {
    return m_matrix;
  }
    AffineTransformation()/*angles to go from the old frame to the new one*/
  {
    m_matrix = my_matrix(1,3,1,3);
    m_translation = my_vector(1,3);
  }

      AffineTransformation(Vector v, Eigen::Matrix3d m)/*angles to go from the old frame to the new one*/
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
  AffineTransformation(Vector translation, double alpha,double beta, double gamma)/*angles to go from the old frame to the new one*/
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
    AffineTransformation(double tx, double ty, double tz, double alpha,double beta, double gamma)/*angles to go from the old frame to the new one*/
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
  AffineTransformation(AffineTransformation &a)/*angles to go from the old frame to the new one*/
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
Vector apply(Vector X)
{
  Vector out = my_vector(1,3);
  mat_vec_mult(m_matrix,X,out);
  out[1] += m_translation[1];
  out[2] += m_translation[2];
  out[3] += m_translation[3];
  return out;
}
Vector apply(double a1,double a2,double a3)
{
  Vector in = my_vector(1,3);
  in[1]=a1;
  in[2]=a2;
  in[3]=a3;
  Vector out = my_vector(1,3);
    // for (int i = 1; i < 4; ++i)
    // {
    //   for (int j = 1; j < 4; ++j)
    //   {
    //     printf("%f", m_matrix[i][j]);
    //   }
    //   printf("\n");
    // }
    // printf("\n");
    // printf("\n");
    // printf("\n");
  mat_vec_mult(m_matrix,in,out);
  out[1] += m_translation[1];
  out[2] += m_translation[2];
  out[3] += m_translation[3];
  return out;
}
void compose(AffineTransformation &a)
{
  Vector translationTemp=my_vector(1,3);
  mat_vec_mult(m_matrix,a.getTranslation(),translationTemp);
  m_translation[1] +=  translationTemp[1];
  m_translation[2] +=  translationTemp[2];
  m_translation[3] +=  translationTemp[3];
  mat_mult(m_matrix,a.getMatrix(),m_matrix);
}
private:
  Vector m_translation;
  Matrix m_matrix;
};


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

/*****************************************************************************
******************************************************************************
Function Name	: add_rosrt_plane_tracker_visp
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
static void ros_run()
{
  if (ros::ok()){
    msg = subscriber->poll();
    if (msg)
    {
      targetState[0] = msg->pose.position.x;
      targetState[1] = msg->pose.position.y;
      targetState[2] = msg->pose.position.z;
      myMarker.xq = msg->pose.orientation.x;
      myMarker.yq = msg->pose.orientation.y;
      myMarker.zq = msg->pose.orientation.z;
      myMarker.wq = msg->pose.orientation.w;



      myMarker.id = msg->id;
      Matrix H = my_matrix(1,4,1,4);
      cameraNeck=AffineTransformation(-0.02,-0.205, -0.02 ,0, 0, 0);
      camera=AffineTransformation(0,0,0,0.436, 0, 0);
      neck = AffineTransformation(0,0,0,0,-M_PI/2,0);
      necky = AffineTransformation(0,0,0,0,-joint_state[B_HR].th,0);
      neckx = AffineTransformation(0,0,0,-joint_state[B_HT].th,0,0);
      neckz = AffineTransformation(0,0,0,0,0,joint_state[B_HN].th);
      neckzBase = AffineTransformation(joint_origin_pos[B_HR][_X_],joint_origin_pos[B_HR][_Y_],joint_origin_pos[B_HR][_Z_],M_PI/2,0,M_PI/2);
      transfon = AffineTransformation(neckzBase);
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
      out = transfon.apply(targetState[0],targetState[1],targetState[2]);




      Eigen::Quaterniond targetQuat2(myMarker.wq,myMarker.xq,myMarker.yq,myMarker.zq);
      targetQuat2.normalize();
      Eigen::Matrix3d eigentransfo= targetQuat2.toRotationMatrix();
      transfornp1 = AffineTransformation(out, eigentransfo);

     // Rx=AffineTransformation(0,0,0,0, 0/2, 0/2);
      neckz = AffineTransformation(0,0,0,0,joint_state[B_HR].th,0);
      necky = AffineTransformation(0,0,0,0,0,-joint_state[B_HT].th);
      neckx = AffineTransformation(0,0,0,joint_state[B_HN].th,0,0);


      


      
      // Rx.compose(transfornp1);
      matToQuat(transfon.getMatrix(),myMarker.xqkin,myMarker.yqkin,myMarker.zqkin,myMarker.wqkin);
      
      transfon.compose(transfornp1);
      Vector yp = transfon.apply(0,1,0);
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
      }
      patch=AffineTransformation(0,0, 0 ,0, M_PI, 0);
      transfon.compose(patch);
      matToQuat(transfon.getMatrix(),myMarker.xq,myMarker.yq,myMarker.zq,myMarker.wq);






      //orientation_book2 = camera.apply(orientation_book[0],orientation_book[1],orientation_book[2]);

 /*     targetOrientation[1]=orientation_book2[1];
      targetOrientation[2]=orientation_book2[2]; 
      targetOrientation[3]=orientation_book2[3];*/

      /*targetOrientation[1]=atan2(-2*myMarker.yq*myMarker.zq+2*myMarker.xq*myMarker.wq,1-2*myMarker.xq*myMarker.xq-2*myMarker.yq*myMarker.yq);
      targetOrientation[2]=-asin(2*myMarker.xq*myMarker.zq+2*myMarker.yq*myMarker.wq); 
      targetOrientation[3]=-atan2(-2*myMarker.xq*myMarker.yq+2*myMarker.zq*myMarker.wq,1-2*myMarker.yq*myMarker.yq-2*myMarker.zq*myMarker.zq);*/

/*      printf("eulerAngles:\n");
      printf("x :%f\n",targetOrientation[1]);
      printf("y :%f\n",targetOrientation[2]);
      printf("z :%f\n",targetOrientation[3]);
      eulerToQuat(targetOrientation,&cart_target_orient[1]);*/
    }
  }
}

void
add_rosrt_plane_tracker( void )

{
  int i, j;
  cart    = my_vector(1,6);
  addTask("Orientation task using SL", init_rosrt_plane_tracker, 
   run_rosrt_plane_tracker, change_rosrt_plane_tracker);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_sample_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

none

 *****************************************************************************/
static int 
init_rosrt_plane_tracker(void)
{

 int j, i;
 int ans;
 time_step = 1./(double)task_servo_rate;
 init_ros();
  // prepare going to the default posture
 bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));

 targetState[0]=0.;
 targetState[1]=0.6;
 targetState[2]=0.;
 targetState[3]=0.;
 targetState[4]=0.;
 targetState[5]=0.;


 targetOrientation[1]=-M_PI/2;
 targetOrientation[2]=0.0;
 targetOrientation[3]=0.0;

 targetOrientation2[1]=0.;
 targetOrientation2[2]=0.;
 targetOrientation2[3]=0.;

 for (int i = 0; i < 3*N_ENDEFFS; ++i)
 {
  if(i<6)
    cstatus[i+1]=1;
  else
    cstatus[i+1]=0;
}
eulerToQuat(targetOrientation,&cart_target_orient[1]);
eulerToQuat(targetOrientation,&cart_target_orient[2]);

handstatus[0]=1;
handstatus[1]=0;
for (i=1; i<=N_DOFS; i++)
{
  target[i] = joint_default_state[i];
  if(i==4)
    target[i].th = 0;
}
  // go to the target using inverse dynamics (ID)
get_int("\nWould you like to st the initial orientation of the left elbow ?",handstatus[0],&handstatus[0]);
if(handstatus[0]==1)
{
  get_double("Desired angle of the elbow ? ",target[4].th,&(target[4].th));
}
  // ready to go
int confirm=0;
get_int("\nMOVE TO THE STARTING POSITION ?",confirm,&confirm);
if(confirm==1)
{
  if (!go_target_wait_ID(target)) 
    return FALSE;
}
ans = 999;
while (ans == 999) {
  if (!get_int("\nEnter 1 to start or anthing else to abort ...",ans,&ans))
    return FALSE;
}

  // only go when user really types the right thing
if (ans != 1) 
  return FALSE;

start_time = task_servo_time;
printf("start time = %.3f, task_servo_time = %.3f\n", 
  start_time, task_servo_time);
addVarToCollect((char *)&(myMarker.x),"ppx","rad",DOUBLE,TRUE);
addVarToCollect((char *)&(myMarker.y),"ppy","rad",DOUBLE,TRUE);
addVarToCollect((char *)&(myMarker.z),"ppz","rad",DOUBLE,TRUE);
updateDataCollectScript();
return TRUE;
}

/*****************************************************************************
******************************************************************************
Function Name	: run_rosrt_plane_tracker
Date		: Dec. 1997

Remarks:

run the task from the task servo: REAL TIME requirements!

******************************************************************************
Paramters:  (i/o = input/output)

none

 *****************************************************************************/
static int 
run_rosrt_plane_tracker(void)
{


  double Ko=1.0,Kp=-1.0;
  double xt=0,yt=0,zt=0,wt=0;
  

  
  ros_run();

  if (myMarker.id==-1)
  {
    Ko=0.0;
    Kp=0.0;
  }
  //eulerToQuat(targetOrientation,&cart_target_orient[1]);
  double task_time = task_servo_time - start_time;
  Eigen::Quaterniond currentQuat(cart_orient[1].q[_Q0_],cart_orient[1].q[_Q1_],cart_orient[1].q[_Q2_],cart_orient[1].q[_Q3_]);
  Eigen::Quaterniond targetQuat(myMarker.wq,myMarker.xq,myMarker.yq,myMarker.zq);
  targetQuat.normalize();
  Eigen::Vector3d eulerAngles;
  inverse_kinematics::quatLogError(targetQuat,currentQuat,eulerAngles);



  cart[1] = Kp*(cart_state[1].x[1]-myMarker.x);
  cart[2] = Kp*(cart_state[1].x[2]-myMarker.y);
  cart[3] = Kp*(cart_state[1].x[3]-myMarker.z);

  cart[4] = Ko*eulerAngles[0];
  cart[5] = Ko*eulerAngles[1];
  cart[6] = Ko*eulerAngles[2];
  visualizePole(0);
  return setJoints2();
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_mygotocart_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
  static int 
  change_rosrt_plane_tracker(void)
  {
    int    ivar;
    double dvar;

    get_int("This is how to enter an integer variable",ivar,&ivar);
    get_double("This is how to enter a double variable",dvar,&dvar);

    return TRUE;

  }


  int
  setJoints2(void)
  {
    int i;
    double task_time;


    task_time = task_servo_time - start_time;

    for (i=1; i<=n_dofs; ++i) {
      target[i].th = joint_des_state[i].th;
    }
    
    if (!inverseKinematics(target,endeff,joint_opt_state,
     cart,cstatus,time_step)) {
      freeze();
    return FALSE;
  }

  /* prepare inverse dynamics */
  int countermrj=0;
  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].thdd = target[i].thdd;
    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].th   = target[i].th;
    if(joint_range[i][MAX_THETA]<joint_des_state[i].th )
    {
      countermrj++;
    }
    if(joint_range[i][MIN_THETA]>joint_des_state[i].th )
    {
      countermrj++;
    }
  }
 /* if(countermrj!=countermrj2)
    printf("\n%d joints have reached their limit\n", countermrj);*/
  countermrj2=countermrj;
   /*for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].thdd = 0;
    joint_des_state[i].thd  = 0;
    joint_des_state[i].th   = target[i].th;
  }
*/
  SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  return TRUE;
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

static bool visualizePole(double angle_measured_)
{
  myMarker.x = out[1];
  myMarker.y = out[2];
  myMarker.z = out[3];
  sendUserGraphics("cube",&myMarker, sizeof(myMarker));
  return TRUE;
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
