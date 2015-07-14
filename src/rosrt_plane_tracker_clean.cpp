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

/*My includes*/
#include "mycube.h"
#include "rosInterface.h"
#include "myGeometryUtils.h"

/*Xenomai includes*/
#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif


#define RATE 1500
#define TOPIC "tracker/object_model"
#define KO 4.0
#define KP -4.0
#define MAX_JOINTS_SPEED 0.2
#define LBX -0.1
#define UBX 0.8
#define LBY 0.4
#define UBY 1.2
#define LBZ -0.8
#define UBZ 0.5
#define THRESHOLD_POSITION 3
#define THRESHOLD_ORIENTATION 0.3

/* defines */
extern "C" void
add_rosrt_plane_tracker( void );


/* local variables */
static double start_time = 0.0;
static rosInterface myMarker_getter = rosInterface(TOPIC); // The ros interface to retrieve data from the tracker topic
static struct cameraToRobot frameMapping;

/* Which hands are going to be controlled*/
static int cstatus[N_ENDEFFS*6];

/* local functions */
static int  init_rosrt_plane_tracker(void);
static bool visualizePole(struct marker);
static int  run_rosrt_plane_tracker(void);
static int  change_rosrt_plane_tracker(void);
double setPositionGain(double Ko,double t,double alpha,double lambda);
static int  setJoints2(Vector cart);



// return the desired initial angle of the right arm elbow angle (asks the user)
static void get_config(double &elbow_angle){
  int controlElbow=1;
  double test=1.0;
 // go to the target using inverse dynamics (ID)
  get_int("\nWould you like to set the initial orientation of the left elbow ?",controlElbow,&controlElbow);
  if(controlElbow==1)
  {
    get_double("Desired angle of the elbow ? ",elbow_angle,&elbow_angle);
  }
}

// ask confirmation to user and moves robot to initial posture
// returns FALSE if fails to move or users refuses motion
static bool init_posture_and_start(double desired_elbow_angle){
  SL_DJstate target[N_DOFS+1];
  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (int i=1; i<=N_DOFS; i++)
  {
    target[i] = joint_default_state[i];
  }
  target[4].th = desired_elbow_angle;
  // ready to go
  int confirm=0;
  get_int("\nMOVE TO THE STARTING POSITION ?",confirm,&confirm);
  if(confirm==1)
  {
    if (!go_target_wait_ID(target)) 
      return FALSE;
  }
  int ans = 999;
  while (ans == 999) {
    if (!get_int("\nEnter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;
  return TRUE;
}

// update_status is a mask for endeffectors:
// 6 values per endeffector (position and orientation)
// 1 indicate the DOF should be controlled, 0 that it should not
// this function sets 1 for position and orientation of right hand, 0 to left hand
static void activate_right_hand(int *update_status){
 for (int i = 0; i < 3*N_ENDEFFS; ++i)
 {
  if(i<6)
    update_status[i+1]=1;
  else
    update_status[i+1]=0;
}
}

// set all the vector entries to 0
static void set_to_zero(Vector &update){
  for (int i=1;i<=6;i++){
    update[i]=0;
  }
}

// computes the weighten error between a target object (myMarker) and the current position of the hand (cart_orient) 
static void right_arm_controller(struct marker &myMarker,SL_quat *cart_orient, double Ko, double Kp, Vector &update_cart){

  Eigen::Quaterniond currentQuat(cart_orient[1].q[_Q0_],cart_orient[1].q[_Q1_],cart_orient[1].q[_Q2_],cart_orient[1].q[_Q3_]);
  Eigen::Quaterniond targetQuat(myMarker.wqhand,myMarker.xqhand,myMarker.yqhand,myMarker.zqhand);
  targetQuat.normalize();
  Eigen::Vector3d eulerAngles;
  inverse_kinematics::quatLogError(targetQuat,currentQuat,eulerAngles);

  update_cart[1] = Kp*(cart_state[1].x[1]-myMarker.xhand);
  update_cart[2] = Kp*(cart_state[1].x[2]-myMarker.yhand);
  update_cart[3] = Kp*(cart_state[1].x[3]-myMarker.zhand);

  update_cart[4] = Ko*eulerAngles[0];
  update_cart[5] = Ko*eulerAngles[1];
  update_cart[6] = Ko*eulerAngles[2];

}

static bool visualizePole(struct marker myMarker)
{
  sendUserGraphics("cube",&myMarker, sizeof(myMarker));
  return TRUE;
}


void check_moving_area(Vector &cart,struct marker &myMarker)
{
double lbx,ubx,lby,uby,lbz,ubz;
lbx =LBX;
ubx =UBX;
lby =LBY;
uby =UBY;
lbz =LBZ;
ubz = UBZ;

myMarker.xaz=(lbx+ubx)/2;
myMarker.yaz=(lby+uby)/2;
myMarker.zaz=(lbz+ubz)/2;

myMarker.wx = ubx-lbx;
myMarker.wy = uby-lby;
myMarker.wz = ubz-lbz;

bool stop=false;
if(cart_state[1].x[1]<lbx)
{
  stop=true;
  if(cart[1]<0)
  {
    cart[1]=0;
  }
}
else if(cart_state[1].x[1]>ubx)
{
  stop=true;
  if(cart[1]>0)
  {
    cart[1]=0;
  }
}
if(cart_state[1].x[2]<lby)
{
  stop=true;
  if(cart[2]<0)
  {
    cart[2]=0;
  }
}
else if(cart_state[1].x[2]>uby)
{
  stop=true;
  if(cart[2]>0)
  {
    cart[2]=0;
  }
}
if(cart_state[1].x[3]<lbz)
{
  stop=true;
  if(cart[3]<0)
  {
    cart[3]=0;
  }
}
else if(cart_state[1].x[3]>ubz)
{
  stop=true;
  if(cart[3]>0)
  {
    cart[3]=0;
  }
}
if(stop)
{
  for (int i = 3; i < 6; ++i)
  {
    cart[i+1]=0;
  }
}
}

void check_object_position(Vector &cart,struct marker &myMarker)
{
double lbx,ubx,lby,uby,lbz,ubz;
lbx =LBX;
ubx =UBX;
lby =LBY;
uby =UBY;
lbz =LBZ;
ubz = UBZ;
bool stop=false;
if(myMarker.x<lbx)
{
  stop=true;
}
else if(myMarker.x>ubx)
{
  stop=true;
  }
if(myMarker.y<lby)
{
  stop=true;
 }
else if(myMarker.y>uby)
{
  stop=true;
  }
if(myMarker.z<lbz)
{
  stop=true;
}
else if(myMarker.z>ubz)
{
  stop=true;
}
if(stop)
{
  for (int i = 0; i < 6; ++i)
  {
    cart[i+1]=0;
  }
}
}

void safety_checking(Vector &cart,struct marker &myMarker)
{
  check_moving_area(cart, myMarker);
  check_object_position(cart, myMarker);
}
/*****************************************************************************
******************************************************************************
  Function Name : setJoints2
  Date    : Dec. 1997

  Remarks:

  Set the joints

******************************************************************************
  Paramters:  (i/o = input/output)

  [in]Vector cart : input for the inverse kinematics function.

 *****************************************************************************/

  // when this function is called, sl moves the robot to correct the error between right hand and object(cart)
  static bool
  set_to_sl(Vector &cart,struct marker &myMarker)
  {
    SL_DJstate target[N_DOFS+1];
  // prepare going to the default posture
    bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
    double time_step = 1./(double)task_servo_rate;
    static int previous_countermrj=-1;
    int i;
    double task_time;

    safety_checking(cart, myMarker);
    task_time = task_servo_time - start_time;

    for (i=1; i<=n_dofs; ++i) {
      target[i].th = joint_des_state[i].th;
    }


    if (!inverseKinematicsClip(target,endeff,joint_opt_state,
     cart,cstatus,time_step,MAX_JOINTS_SPEED,MAX_JOINTS_SPEED)) {
      freeze();
    return FALSE;
  }
  //clip_velocity(target,MAX_JOINTS_SPEED);

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
  previous_countermrj = countermrj;

  SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  return TRUE;
}


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


void
add_rosrt_plane_tracker( void )

{
  int i, j;
  addTask("Orientation task using SL", init_rosrt_plane_tracker, 
   run_rosrt_plane_tracker, change_rosrt_plane_tracker);

}    




/*****************************************************************************
******************************************************************************
  Function Name	: init_rosrt_plane_tracker
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


// cstatus is a mask for hand effector, which indicates with dof should be controlled
// setting cstatus for controlling position and orientation of right hand
 activate_right_hand(cstatus);

// getting from user desired initial angle of elbow
 double desired_elbow_angle=0.0;
 get_config(desired_elbow_angle);
 if(!init_posture_and_start(desired_elbow_angle))
  return FALSE;

start_time = task_servo_time;
printf("start time = %.3f, task_servo_time = %.3f\n", 
  start_time, task_servo_time);

// adding data to collect
/*addVarToCollect((char *)&(myMarker.x),"ppx","rad",DOUBLE,TRUE);
addVarToCollect((char *)&(myMarker.y),"ppy","rad",DOUBLE,TRUE);
addVarToCollect((char *)&(myMarker.z),"ppz","rad",DOUBLE,TRUE);
updateDataCollectScript();*/

// moving to initial posture and starting
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

  static struct marker myMarker;

  // getting updated marker in camera's frame
  if (myMarker_getter.update(myMarker)){
  // transforming to robots frame
    frameMapping.update(myMarker);
  }

  //displaying marker
  visualizePole(myMarker);

  // error in position and orientation weighten by gains
  Vector cart =my_vector(1,6);
  set_to_zero(cart);
  if (myMarker.id!=-1){
    right_arm_controller(myMarker,cart_orient, KO, KP, cart);
  }

  // sl magic: robot motion to correct error
  return set_to_sl(cart,myMarker);
  
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_rosrt_plane_tracker
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

