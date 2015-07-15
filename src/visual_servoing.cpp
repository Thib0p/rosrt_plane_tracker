

class Tracked_object {
public:
  void clear();
  void clear(double value);
  double position[3];
  double orientation[4];
  bool lost;
  
};

void Tracked_object::clear(double value){
  int i;
  for(i=0;i<3;i++) this->position[i]=value;
  for(i=0;i<4;i++) this->orientation[i]=value;
}

void Tracked_object::clear(){ this->clear(0); }


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
  Vector out = transfon.apply(0,0,0);
  Vector zk = transfon.apply(0,0,1);
  for (int i=0;i<3;i++) zk[i+1] -= out[i+1];
  Tracked_object kinect;
  for (int i=0;i<3;i++) kinect.position[i]=out[i+1];
  double kxp=kinect.position[0],kyp=kinect.position[1],kzp=kinect.position[2];
  out = transfon.apply(kinect.position[0],kinect.position[1],position[2]);
  for (int i=0;i<3;i++) kinect.position[i]=out[i+1];
  Eigen::Quaterniond targetQuat2(kinect.orientation[0],kinect.orientation[1],kinect.orientation[2],kinect.orientation[3]);
  targetQuat2.normalize();
  Eigen::Matrix3d eigentransfo= targetQuat2.toRotationMatrix();
  movenp1 = AffineTransformation(kxp,kyp,kzp,0,0,0);
  transfornp1 = AffineTransformation(0,0,0, eigentransfo);
  matToQuat(transfon.getMatrix(),kinect.orientation[0],kinect.orientation[1],kinect.orientation[2],kinect.orientation[3]);
  transfon.compose(movenp1);
  transfon.compose(transfornp1);
  matToQuat(transfon.getMatrix(),myMarker.xq,myMarker.yq,myMarker.zq,myMarker.wq);
  transfon.compose(grasping_pose);
  out = transfon.apply(0,0,0);
  for (int i=0;i<3;i++) to_update.position[i]=out[i+1];
  matToQuat(transfon.getMatrix(),to_update.orientation[0],to_update.orientation[1],to_update.orientation[2],to_update.orientation[3]);
}


class Compute_error {
public:
  virtual void compute(const Tracked_object &object, Vector &get_error)=0;
};

class Right_arm_controller : public Compute_error {
public: 
  Right_arm_controller(SL_quat *cart_orient, double ko, double kp);
  void compute(const Tracked_object &object, Vector &get_error);
private:
  SL_quat *cart_orient;
  SL_Cstate *cart_state;
  double ko,kp;
};

Right_arm_controller::Right_arm_controller(SL_quat *cart_orient, SL_Cstate *cart_state, double ko, double kp){
  this->cart_orient = cart_orient;
  this->cart_state = cart_state;
  this->ko = ko;
  this->kp = kp;
}

void Right_arm_controller::compute(const Tracked_object &object, Vector &get_error){
  Eigen::Quaterniond currentQuat(this->cart_orient[1].q[_Q0_],this->cart_orient[1].q[_Q1_],this->cart_orient[1].q[_Q2_],this->cart_orient[1].q[_Q3_]);
  Eigen::Quaterniond targetQuat(object.orientation[0],object.orientation[1],object.orientation[2],object.orientation[3]);
  targetQuat.normalize();
  Eigen::Vector3d eulerAngles;
  inverse_kinematics::quatLogError(targetQuat,currentQuat,eulerAngles);
  for(int i=0;i<3;i++){
    get_error[i+1] = this->kp*(cart_state[1].x[i+1]-object.position[i]);
    get_error[i+4] = this->ko*eulerAngles[i];
  }
}

class Conditions_checker {
public:
  Conditions_checker();
  virtual bool has_warning(const Tracked_object &object);
  virtual bool has_error(const Tracked_object &object);
  std::string get_warning();
  std::string get_error();
  Tracked_object get_recommandation_mask();
private:
  Tracked_object recommandation_mask;
  std::string warning;
  std::string error;
};

Condition_checker::Conditions_checker(){
  this->warning="";
  this->error="";
}

Condition_checker::get_warning(){ return this->warning; }
Condition_checker::get_error(){ return this->error; }


class Rectangle {
public:
  Rectangle(double* center, double * size);
  bool is_inside(const double* point);
  bool is_inside(const double* point, const int dimension);
private:
  double center[3];
  double size[3]
};

Rectangle::Rectangle(double *center, double *size){
  for (int i=0;i<3;i++){
    this->center[i] =  center[i];
    this->size[i] = size[i];
  }
}
bool Rectangle::is_inside(const double *point){
  for (int i=0;i<3;i++){
    if (point[i]<this->center[i]-this->size[i]/2.0) return false;
    if (point[i]<this->center[i]+this->size[i]/2.0) return false;
  }
  return true;
}
bool Rectangle::is_inside(const double *point, const int dimension){
    if (point[dimension]<this->center[dimension]-this->size[dimension]/2.0) return false;
    if (point[dimension]<this->center[dimension]+this->size[dimension]/2.0) return false;
    return True;
}

class Moving_area_condition : public Conditions_checker {
public:
  Moving_area_condition(SL_Cstate *cart_state, double* center, double* size);
  ~Moving_area_condition();
  bool has_warning(const Tracked_object &object);
  bool has_error(const Tracked_object &object);
private:
  Vector *cart_state;
  Rectangle *rectangle;
};

class Object_area_condition : public Conditions_checker {
public:
  Object_area_condition(double* center, double* size);
  ~Object_area_condition();
  bool has_warning(const Tracked_object &object);
  bool has_error(const Tracked_object &object);
private:
  Rectangle *rectangle;
};


Moving_area_condition::Moving_area_condition(SL_Cstate *cart_state, double *center, double *size){
  this->cart_state = cart_state;
  this->rectangle = new Rectangle(center,size);
}
Moving_area_condition::~Moving_area_condition(){
  delete (this->rectangle)
}
bool Moving_area_condition::has_error(const Tracked_object &object){ return false; }
bool Moving_area_condition::has_warning(const Tracked_object &object){
  this->recommandation_mask.clear(1);
  bool at_least_one_outside = false;
  for (int i=0;i<3;i++){
    if (!this->rectangle->is_inside(this->cart_state[1].x[i+1],i)){
	at_least_one_outside=true;
	this->recommandation_mask.position[i]=0;
      }
  }
  if (at_least_one_outside){
    for(int i=0;i<4;i++) thus->recommandation_mask.orientation[i]=0;
  }
  if (at_least_ine_outside) {
    this->warning = "endeffector outside specified space";
    return true;
  }
  return false;
}    


class Object_area_condition : public Conditions_checker {
public:
  Object_area_condition(double* center, double* size);
  ~Object_area_condition();
  bool has_warning(const Tracked_object &object);
  bool has_error(const Tracked_object &object);
private:
  Rectangle *rectangle;
};


Object_area_condition::Object_area_condition(double *center, double *size){
  this->rectangle = new Rectangle(center,size);
}
Object_area_condition::~Object_area_condition(){
  delete (this->rectangle)
}
bool Object_area_condition::has_error(const Tracked_object &object){ return false; }
bool Object_area_condition::has_warning(const Tracked_object &object){
  this->recommandation_mask.clear(1);
  if (!this->rectangle->is_inside(object.position)){
    this->recommandation_mask.clear(0);
    this->warning = "tracked object outside specified space";
    return true;
  }
  return false;
}    


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
