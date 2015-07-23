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

int nbre_poses;
double xpose[40],ypose[40],zpose[40];
double xqpose[40],yqpose[40],zqpose[40],wqpose[40];
};
#endif 