#ifndef MY_CUBE
#define MY_CUBE
struct marker{
//Object
double x,y,z;
double xq,yq,zq,wq;
int id;

//Hand
double xhand,yhand,zhand;
double xqhand,yqhand,zqhand,wqhand;
//kinect
double xkin,ykin,zkin;
double xqkin,yqkin,zqkin,wqkin;

//Allowed zone
double xaz,yaz,zaz; //cartesian position of the center of the allowed box
double wx,wy,wz; //size of the allowed box
};
#endif 