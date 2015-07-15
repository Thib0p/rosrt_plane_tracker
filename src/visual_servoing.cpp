

class Tracked_object {
public:
  double position[3];
  double orientation[4];
  bool lost;
};

class Tracked_object_getter {
public:
  virtual void get(Tracked_object &get_object)=0;
};

class RosRT_tracked_object_getter : public Tracked_object_getter {
public:
  RosRT_tracked_object_getter(std::string node_name, std::string topic);
  bool get(Tracked_object &get_object);
private:
  visualization_msgs::MarkerConstPtr msg;
  ros::NodeHandle *handle;
  rosrt::Subscriber<visualization_msgs::Marker> *subscriber;
};

RosRT_tracked_object_getter::RosRT_tracked_object_getter(std::string node_name, std::string topic) {
  int argc = 1;
  char name[] = "SL";
  char* argv[1];
  argv[0] = name;
  ros::init(argc,argv,node_name);
  struct rosrt::InitOptions options;
  options.pubmanager_thread_name = node_name;
  this->handle = new ros::NodeHandle();
  rosrt::init(options);
  this->subscriber = new rosrt::Subscriber<visualization_msgs::Marker>(3, *handle, topic);
}
bool RosRT_tracked_object_getter::get(Tracked_object &get_object){
  if (ros::ok()){
    this->msg = subscriber->poll();
    if (this->msg)
      {
	get_object.position[0]=msg->pose.position.x;
	get_object.position[1]=msg->pose.position.y;
	get_object.position[2]=msg->pose.position.z;
	get_object.orientation[0]=msg->pose.orientation.x;
	get_object.orientation[1]=msg->pose.orientation.y;
	get_object.orientation[2]=msg->pose.orientation.z;
	get_object.orientation[3]=msg->pose.orientation.w;
	get_object.lost = msg->id;
        return true;
      }
    return false;
  }
}


class Referencial_transform {
public:
  virtual void transform(Tracked_object &to_update)=0;
};

class Apollo_kinect_to_right_arm : public Referencial_transform {
public: 
  Apollo_kinect_to_right_arm();
  void transform(Tracked_object &to_update);
private:
  AffineTransformation cameraNeck;
  AffineTransformation camera;
  AffineTransformation neck;
  AffineTransformation patch;
};


Apollo_kinect_to_right_arm::Apollo_kinect_to_right_arm(){
  this->cameraNeck=AffineTransformation(-0.02,-0.205, -0.02 ,0, 0, 0);
  this->camera=AffineTransformation(0,0,0,0.436, 0, 0);
  this->neck = AffineTransformation(0,0,0,0,-M_PI/2,0);
  this->patch=AffineTransformation(0,0, 0 ,0, M_PI, 0);
  this->grasping_pose = AffineTransformation(0,-.2,+0.13,0,-M_PI/2,M_PI/2);
}

Apollo_kinect_to_right_arm::transform(Tracked_object &to_udpdate){
  AffineTransformation necky = AffineTransformation(0,0,0,0,-joint_state[B_HR].th,0);
  AffineTransformation neckx = AffineTransformation(0,0,0,-joint_state[B_HT].th,0,0);
  AffineTransformation neckz = AffineTransformation(0,0,0,0,0,joint_state[B_HN].th);
  AffineTransformation neckzBase = AffineTransformation(joint_origin_pos[B_HR][_X_],joint_origin_pos[B_HR][_Y_],joint_origin_pos[B_HR][_Z_],M_PI/2,0,M_PI/2);
  AffineTransformation transfon = AffineTransformation(neckzBase);
  transfon.compose(neckz);
  transfon.compose(neckx);
  transfon.compose(necky);
  transfon.compose(this->neck);
  transfon.compose(this->camera);
  transfon.compose(this->cameraNeck);
  AffineTransformation out = transfon.apply(0,0,0);
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

}


class Compute_error {
public:
  virtual void compute(const Tracked_object &object, Tracked_object &get_error)=0;
};

class Conditions_checker {
public:
  Conditions_checker();
  virtual bool has_warning(const Tracked_object &object);
  virtual bool has_error(const Tracked_object &object);
  std::string get_warning();
  std::string get_error();
private:
  std::string warning;
  std::string error;
};

Condition_checker::Conditions_checker(){
  this->warning="";
  this->error="";
}

Condition_checker::get_warning(){ return this->warning; }
Condition_checker::get_error(){ return this->error; }

class Visual_servoing {
public:
  Visual_servoing(Tracked_object_getter *getter,
		  std::vector<Condition_checker> *preconditions,
		  Referencial_transform *transform,
		  std::vector<Condition_checker> *postconditions,
		  Compute_error *compute_error);
  void get_error(Tracked_object &error);
  bool has_warnings();
  bool has_errors();
  std::vector<string> get_warnings;
  std::vector<string> get_errors;
private:
  void check_conditions(const std::vector<Condition_checker> *conditions);
  Tracked_object object;
  Tracked_object_getter *getter;
  Referencial_transform *transform;
  Compute_error *compute_error;
  std::vector<Condition_checker> *preconditions;
  std::vector<Condition_checker> *postconditions;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};


Visual_servoing::Visual_servoing(Tracked_object_getter *getter,
		  std::vector<Condition_checker> *preconditions,
		  Referencial_transform *transform,
		  std::vector<Condition_checker> *postconditions,
		 Compute_error *compute_error){
  this->getter = getter;
  this->transform = transform;
  this->compute_error = compute_error;
  this->preconditions = preconditions;
  this->postconditions = postconditions;
}
bool Visual_servoing::has_errors(){
  if (this->errors.size()==0) return false;
  return true;
}
bool Visual_servoing::has_warnings(){
  if (this->warnings.size()==0) return false;
  return true;
}
std::vector<std::string> Visual_servoing::get_errors(){ return this->errors; }
std::vector<std::string> Visual_servoing::get_warnings(){ return this->warnings; }
void Visual_servoing::check_conditions(const std::vector<Condition_checker> *conditions, const Tracked_object &object){
  if (conditions->size()==0) return;
  for(int i=0;i<conditions->size();i++){
    if ((*conditions)[i].has_warning(object)){
      this->warnings.push_back((*conditions)[i].get_warning());
    }
    if ((*conditions)[i].has_errors(object)){
      this->errors.push_back((*conditions)[i].get_error());
    }
  }
}
void Visual_servoing::get_error(Tracked_object &get_error){
  this->warnings.clear();
  this->errors.clear();
  if (! this->getter.get(this->object) ) {
    this->errors.push_back("tracker stopped");
    return;
  }
  if (this->object.lost){
    this->warnings.push_back("tracked object lost");
    return 
  }
  this->check_conditions(this->preconditions,this->object);
  if (this->has_errors()) return;
  try {
    this->transform->transform(this->object);
  } catch (exception &e){
    this->errors.push_back(e.what());
    return;
  }
  this->check_conditions(this->postconditions,this->object);
  if (this->has_errors()) return;
  try {
    this->compute_error->compute(this->object,get_error);
  } catch (exception &e){
    this->errors.push_back(e.what());
    return;
  }
}
