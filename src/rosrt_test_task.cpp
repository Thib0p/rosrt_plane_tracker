/*============================================================================
==============================================================================
                      
                              rosrt_test_task.cpp
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/


#include "ros/ros.h"
#include "ros/param.h"
#include "std_msgs/UInt32.h"
#include "rosrt/rosrt.h"


// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"



#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

// defines
#define TOPIC "rt_topic"

// local variables
static double start_time = 0.0;
static double freq;
static double amp;
static SL_DJstate  target[N_DOFS+1];

// ros variables
static ros::NodeHandle *handle;
static rosrt::Subscriber<std_msgs::UInt32> *subscriber;
static std_msgs::UInt32ConstPtr msg;

// global functions 
extern "C" void
add_rosrt_test_task( void );

// local functions
static int  init_rosrt_test_task(void);
static int  run_rosrt_test_task(void);
static int  change_rosrt_test_task(void);

/*****************************************************************************
******************************************************************************
Function Name	: add_rosrt_test_task
Date		: Feb 2015
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_rosrt_test_task( void )
{
  int i, j;
  
  addTask("apollo rosrt test", init_rosrt_test_task, 
	  run_rosrt_test_task, change_rosrt_test_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_rosrt_test_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/

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
   subscriber = new rosrt::Subscriber<std_msgs::UInt32>(3, *handle, TOPIC);
   //subscriber->initialize(3,*handle,TOPIC);
}

static int 
init_rosrt_test_task(void)
{

  int j, i;
  int ans;
  static int firsttime = TRUE;

  init_ros();


  if (firsttime){
    firsttime = FALSE;
    freq = 0.1; // frequency
    amp  = 0.5; // amplitude
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
	 start_time, task_servo_time);

  return TRUE;
}


static void ros_run()
{
    if (ros::ok()){
    msg = subscriber->poll();
    if (msg)
      std::cout << msg->data << std::endl;
  }
  }

/*****************************************************************************
******************************************************************************
  Function Name	: run_rosrt_test_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_rosrt_test_task(void)
{
  int j, i;

  double task_time;
  double omega;
  int    dof;

  ros_run();

  // NOTE: all array indices start with 1 in SL

  task_time = task_servo_time - start_time;
  omega     = 2.0*PI*freq;

  // osciallates one DOF 
  dof = 1;
  for (i=dof; i<=dof; ++i) {
    target[i].th   = joint_default_state[i].th +
      amp*sin(omega*task_time);
    target[i].thd   = amp*omega*cos(omega*task_time);
    target[i].thdd  =-amp*omega*omega*sin(omega*task_time);
  }

  // the following variables need to be assigned
  for (i=1; i<=N_DOFS; ++i) {
    joint_des_state[i].th   = target[i].th;
    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].thdd = target[i].thdd;
    joint_des_state[i].uff  = 0.0;
  }

  // compute inverse dynamics torques
  SL_InvDynNE(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_rosrt_test_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_rosrt_test_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}
