// PumaClient.cpp : Defines the entry point for the console application.
//
#include "RobotCom.h"
#include <iostream>
//#include <tchar.h>
using namespace std;
#define DOF 6
#define QREACHEDTOL 10 // fine tune this parameter
#define QREACHEDITER 100 // fine tune this parameter
bool posReached(float *qd, float *q)
{
 float qTolerance = 0;
 static int reachedCntr = 0;
for(int i = 0; i < DOF; i ++)
 {
 qTolerance += (qd[i]-q[i])*(qd[i]-q[i]);
 }
if(qTolerance < QREACHEDTOL)
 reachedCntr ++;
 else
 reachedCntr = 0;
 
 if(reachedCntr >= QREACHEDITER)
 {
 reachedCntr = 0;
 return true;
 }
return false;
}
// Move to joint position via jgoto
void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq)
{
 // Output the joint command
 Robot->jointControl(JGOTO, qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
// Wait for the robot to finish motion
 do
 {
 Robot->getStatus(GET_JPOS,q);
 Robot->getStatus(GET_JPOS,dq);
 }while(!posReached(qd,q));
}
int main(int argc, char**argv) {
 // declarations
 RobotCom* PumaRobot = new RobotCom();
 float q_desired_[DOF] = {0,0,0,0,0,0}; // in degrees

 float q_mid_[6] = {-92.9813, -46.7017, 187.181, -0.648862, -41.6881, -540.012};
 float q_top_[6] = {-93.2803, -76.474, 140.319, -13.4982, 27.8973, -552.762};
 float q_bottom_[6] = {-93.2688, -15.1146, 187.168, -1.56295, -78.5715, -533.383};
 float dq_[DOF], q_[DOF];
 char key = 'a';
// float the robot right after connection is established
 PumaRobot->_float();
 cout << "Floating..." << endl;
 cout << "Disengage E-stop and hit 's'" << endl;
 while(1)
 {
 cin >> key;
 if(key == 's')
 break;
 }
 cout << "Disengaged..." << endl;
cout << "Starting tasks..." << endl;
 // start tasks
 while(1)
 {
 /********************************************************/
 // first motion
 /********************************************************/
 //q_desired_[2] = 30;
 MoveJGOTO(PumaRobot,q_mid_,q_,dq_);
 cout << "Motion 1 done" << endl;
 MoveJGOTO(PumaRobot,q_top_,q_,dq_);
 cout << "Motion 1 done" << endl;
 MoveJGOTO(PumaRobot,q_bottom_,q_,dq_);
 cout << "Motion 1 done" << endl;
 /********************************************************/
 // second motion
 /********************************************************/
 //q_desired_[2] = 0;
 //MoveJGOTO(PumaRobot,q_desired_,q_,dq_);
 //cout << "Motion 2 done" << endl;
 break;
 }
// float the before disconnecting
 PumaRobot->_float();
 string ss;
 cin >> ss;
 PumaRobot->~RobotCom();
return 0;
}
