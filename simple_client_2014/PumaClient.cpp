// PumaClient.cpp : Defines the entry point for the console application.
//
#pragma once

#include "RobotCom.h"
#include "TetrisCom.h"
#include <iostream>
//#include <omp.h>

#ifdef WIN32
#include "Magnet.h"
#else
#include "MagnetStub.h"
#endif
//#include <tchar.h>

//#include "PrVector.h"
//#include "Prvector3.h"
//#include "PrMatrix.h"
//#include "PrMatrix3.h"
#include <fstream>
#include <sstream>
//#include "param.h"
#include <string>
#include <stdlib.h>
#include "math.h"

// opencv libraries
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
//using namespace cv;

// constants of game board
const int NUM_SQUARES_HIGH = 20;
const int NUM_SQUARES_WIDE = 10;
const float SQUARE_SIZE = 0.05;

const string BLOCK_POS_FILENAME = "block_pos.txt";

#define XREACHEDTOL 0.2
#define QREACHEDTOL 20 // fine tune this parameter
#define DQREACHEDTOL 20 // fine tune this parameter
#define REACHEDITER 100 // fine tune this parameter

// number of arguments describing the end-effector
const int X_DOF = 7; // task space
const int J_DOF = 6; // Joint space

// home position
float HOME_POS[X_DOF] = {0, -0.7, 0, 0.5, 0.5, 0.5,-0.5};

// The positions of the tetrominos. Needs calibration.
float squarePos[X_DOF];
float linePos[X_DOF];
float lPos[X_DOF];
float reverseLPos[X_DOF];
float tPos[X_DOF];
float sPos[X_DOF];
float zPos[X_DOF];


enum Position {
	X,
	Y,
	Z,
	W,
	VX,
	VY,
	VZ
};

float orientations[4][4] = {
	{0.5, 0.5, 0.5, -0.5},
	{0.71, 0.71, 0, 0},
	{0.5, 0.5, -0.5, 0.5},
	{0, 0, 0.71, -0.71}
};

float default_gains[12] = { 400, 400, 400, 400, 400, 400, 40, 40, 40, 40, 40, 40 };
float high_gains_last_joint[12] = { 400, 400, 400, 400, 400, 4000, 40, 40, 40, 40, 40, 80 };
float low_gains[12] = { 100, 100, 100, 100, 100, 100, 10, 10, 10, 10, 10, 10 };
float verylow_gains[12] = { 40, 40, 40, 40, 40, 40, 10, 10, 10, 10, 10, 10 };
float high_gains[12] = { 1000, 1000, 1000, 1000, 1000, 1000, 100, 100, 100, 100, 100, 100 };

float kp[20], kv[20];

void getGains(RobotCom* PumaRobot) {

	//PumaRobot->_float();

	PumaRobot->getStatus(GET_KP, kp);
	cout << "Starting KP:";
	for(int i=0; i<12; i++) cout << " " << kp[i]; cout << endl;

	PumaRobot->getStatus(GET_KV, kv);
	cout << "Starting KV:";
	for(int i=0; i<12; i++) cout << " " << kv[i]; cout << endl;

}


void moveTo(float *x_goal, int target_x, int target_y, int &curr_x, int &curr_y, int rotation)
{
	curr_y = max(0, min(NUM_SQUARES_HIGH, target_y));
	curr_x = max(0, min(NUM_SQUARES_WIDE, target_x));

	x_goal[X] = SQUARE_SIZE*(curr_x-NUM_SQUARES_WIDE/2);
	x_goal[Z] = -SQUARE_SIZE*(curr_y-NUM_SQUARES_HIGH/2);
	assert(rotation>=0 && rotation<4);
	for(int i=0; i<4; i++) x_goal[3+i] = orientations[rotation][i];

	cout << "currx: " << curr_x << "  curr_y: " << curr_y << " curr_rotation:  " << rotation << endl;
}

void waitForStart(RobotCom *PumaRobot)
{
	PumaRobot->setGains(FLOATMODE, default_gains);
	getGains(PumaRobot);
	PumaRobot->_float();
	PumaRobot->setGains(FLOATMODE, default_gains);
	cout << "Float..." << endl;
	PumaRobot->setGains(FLOATMODE, default_gains);
	PumaRobot->setGains(GOTO, default_gains);
	PumaRobot->setGains(NO_CONTROL, default_gains);
	getGains(PumaRobot);
	cout << "Disengage E-stop and hit 's'" << endl;
	char key;
	while(1)
	{
		cin >> key;
		if(key == 's')
			break;
	}
	cout << "Disengaged..." << endl;
	cout << "Starting tasks..." << endl;
}

bool xposReached(float* xd, float *x)
{
	float xTolerance = 0;
	static int reachedCntr = 0;
	for(int i = 0; i < X_DOF; i ++)
	{
		xTolerance += (xd[i]-x[i])*(xd[i]-x[i]);
	}
	if(xTolerance < XREACHEDTOL)
		reachedCntr ++;
	else
		reachedCntr = 0;

	if(reachedCntr >= REACHEDITER)
	{
		reachedCntr = 0;
		return true;
	}
	return false;
}

bool jposReached(float *qd, float *q)
{
	float qTolerance = 0;
	static int reachedCntr = 0;
	for(int i = 0; i < J_DOF; i ++)
	{
		qTolerance += (qd[i]-q[i])*(qd[i]-q[i]);
	}
	if(qTolerance < QREACHEDTOL)
		reachedCntr ++;
	else
		reachedCntr = 0;

	if(reachedCntr >= REACHEDITER)
	{
		reachedCntr = 0;
		return true;
	}
	return false;
}


bool jposStopped(float *dq)
{
	float dqTolerance = 0;
	static int stoppedCnt = 0;
	for(int i = 0; i < J_DOF; i ++)
	{
		dqTolerance += dq[i]*dq[i];
	}
	if(dqTolerance < DQREACHEDTOL)
		stoppedCnt ++;
	else
		stoppedCnt = 0;

	if(stoppedCnt >= REACHEDITER)
	{
		stoppedCnt = 0;
		return true;
	}
	return false;
}


void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq, float *gains)
{
	cout << "Start JGOTO" << endl;
	Robot->setGains(JGOTO, gains);
	Robot->setGains(GOTO, gains);
	Robot->setGains(FLOATMODE, gains);
	Robot->setGains(NO_CONTROL, gains);
	// Output the joint command
	Robot->jointControl(JGOTO, qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
	//getGains(Robot);

	// Wait for the robot to finish motion
	do
	{
		Robot->getStatus(GET_JPOS,q);
		Robot->getStatus(GET_JVEL,dq);
	}while(!jposReached(qd,q));
	cout << "Complete JGOTO" << endl;
}

// Move towards the goal until you are stopped
void GentlyMoveGOTO(RobotCom *Robot, float *xd, float *gains) {
	cout << "Start gentle GOTO" << endl;

	// Set gains
	Robot->setGains(JGOTO, gains);
	Robot->setGains(GOTO, gains);
	Robot->setGains(FLOATMODE, gains);
	Robot->setGains(NO_CONTROL, gains);
	getGains(Robot);

	// Output the joint command
	Robot->control(GOTO, xd, 7);

        // Wait for the robot to stop
	float dq[6];
	do
	{
		Robot->getStatus(GET_JVEL,dq);
	}while(!jposStopped(dq));
	cout << "Complete gentle GOTO" << endl;

}

void MoveGOTO(RobotCom *Robot, float *xd, float *x, float *gains)
{
	cout << "Start GOTO" << endl;
	Robot->setGains(JGOTO, gains);
	Robot->setGains(GOTO, gains);
	Robot->setGains(FLOATMODE, gains);
	Robot->setGains(NO_CONTROL, gains);
	getGains(Robot);
	// Output the joint command
	Robot->control(GOTO, xd, 7);
	//getGains(Robot);

	// Wait for the robot to finish motion
	do
	{
		Robot->getStatus(GET_IPOS,x);
	}while(!xposReached(xd,x));
	cout << "Complete GOTO" << endl;
}

void pickUpBlock(RobotCom* bot, float *block_pos, HANDLE & serial)
{
	float x_[X_DOF];
	float pick_up_pos[X_DOF];
	for(int i = 0; i < X_DOF; i++) pick_up_pos[i] = block_pos[i];
	pick_up_pos[Z] += 0.05;

	MoveGOTO(bot, block_pos, x_, default_gains);
	 
	GentlyMoveGOTO(bot, pick_up_pos, verylow_gains);

	magnetOn(serial);

	GentlyMoveGOTO(bot, block_pos, verylow_gains);

	//go home after you have the block
	MoveGOTO(bot, HOME_POS, x_, default_gains);
}

void goHome(RobotCom* bot, float *x_goal)
{
	float xd_[X_DOF] = {};
	for (int i = 0; i < X_DOF; i++)
	{
		xd_[i] = HOME_POS[i];
	}
	
	for(int i=0; i<X_DOF; i++) x_goal[i] = xd_[i];
	x_goal = xd_;
	float x_[X_DOF];

	float qd_[J_DOF] = {-90,-45,180,0,-45,0}; // in degrees
	float dq_[J_DOF], q_[J_DOF]; 

	//MoveJGOTO(bot, qd_, q_, dq_, default_gains);
	//MoveGOTO(bot, xd_, x_, default_gains);
}

void moveToTop(RobotCom* bot, float *x_goal)
{
	float xd_[X_DOF] = {0, -0.7, 0.3, 0.5,0.5,0.5,-0.5};
	for(int i=0; i<X_DOF; i++) x_goal[i] = xd_[i];
	float x_[X_DOF];

	MoveGOTO(bot, xd_, x_, default_gains);
	cout << "top" << endl;
}

void recordBlockPos(float* position_array, string name, RobotCom* PumaRobot)
{
	PumaRobot->_float();
	cout << "Floating. Position over " << name << " block." << endl;
	cout << "Hit 's' to record the position" << endl;
	char key;
	while(1)
	{
		cin >> key;
		if(key == 's')
			break;
	}

	PumaRobot->getStatus(GET_IPOS, position_array);
}

void loadPositionArray(float *pos_arr, ifstream & fin)
{
	string line;
	getline(fin, line);
	istringstream iss(line);

	for(int i = 0; i < X_DOF; i++)
		iss >> pos_arr[i];

	for (int i = 0; i < X_DOF; i++) 
		cout << pos_arr[i] << " " << endl;

}

void writePositionArray(float *pos_arr, ofstream & fout)
{
	for(int i = 0; i < X_DOF; i++)
		fout << pos_arr[i] << " ";
	fout << endl;
}

void calibratePositions(RobotCom* PumaRobot)
{
	cout << "Recalibrate tetromino pick up locations? (y/n)" << endl;
	char key;
	cin >> key;

	// If no recalibration, just read the positions from file
	if(!(key == 'y' || key == 'Y'))
	{
		ifstream infile(BLOCK_POS_FILENAME);
		if (infile.fail()) 
		{
			cout << "error opening file" << endl;
			for (int i = 0; i < X_DOF; i++)
				squarePos[i] = HOME_POS[i];
			return;
		}

		// These arrays need to be in the same order as the ones
		// in the recordBlockPos below.
		loadPositionArray(squarePos, infile);
		//loadPositionArray(linePos, infile);
		//loadPositionArray(lPos, infile);
		//loadPositionArray(reverseLPos, infile);
		//loadPositionArray(tPos, infile);
		//loadPositionArray(sPos, infile);
		//loadPositionArray(zPos, infile);

		infile.close();
		return;
	}

	// recalibrate, the order of these needs to be the 
	// same as the order of the arrays above.
	recordBlockPos(squarePos, "SQUARE", PumaRobot);
	/*recordBlockPos(linePos, "LINE", PumaRobot);
	recordBlockPos(lPos, "L", PumaRobot);
	recordBlockPos(reverseLPos, "REVERSE L", PumaRobot);
	recordBlockPos(tPos, "T", PumaRobot);
	recordBlockPos(sPos, "S", PumaRobot);
	recordBlockPos(zPos, "Z", PumaRobot);*/

	// record the positions
	ofstream outfile(BLOCK_POS_FILENAME, ofstream::out | ofstream::trunc);

	// the order of these needs to be the 
	// same as the order of the arrays above.
	writePositionArray(squarePos, outfile);
	/*writePositionArray(linePos, outfile);
	writePositionArray(lPos, outfile);
	writePositionArray(reverseLPos, outfile);
	writePositionArray(tPos, outfile);
	writePositionArray(sPos, outfile);
	writePositionArray(zPos, outfile);*/

	outfile << endl;

	// Make sure you stick to this order!
	outfile << "The arrays are in the order: " << endl;
	outfile << "SQUARE, LINE, L, REVERSE L, T" << endl;

	outfile.close();

	cout << "Calibration finished. Enter any key to goHome." << endl;
	cin >> key;
}

int main(int argc, char **argv)
{
	HANDLE serial = magnetInit("COM14");

	// start up
	float x_goal[X_DOF];
	float x_[X_DOF];
	TetrisCom* TetrisServer = new TetrisCom();
	if(false) {
		string s;
		while(TetrisServer->readLine(s)) {
			cout << "<<" << s << ">>" << endl;
			TetrisServer->sendOK();
		}
	}
	RobotCom* PumaRobot = new RobotCom();

	getGains(PumaRobot);
	waitForStart(PumaRobot);

	calibratePositions(PumaRobot);

	goHome(PumaRobot, x_goal);
	cout << "home!" << endl;

	char key;
	cout << "press any key to continue";
	cin >> key;

	/// testing pickup
	pickUpBlock(PumaRobot, squarePos, serial); 
	return 0;


	// initialize game board variables
	int curr_x = NUM_SQUARES_WIDE/2; int curr_y = NUM_SQUARES_WIDE/2;
	int rotation=0;
	while(true) {
		// Handle input from the game
		std::string s;
		while(TetrisServer->readLine(s)) {
			if(s=="Q") break;
			if(s.size()>=4 && s.substr(0,4) == "SCL ") {
				istringstream is(s.substr(4,-1));
				int x,y,r;
				is >> x >> y >> r;
				cout << s << " -> " << x << " " << y << " " << r << endl;
				cout.flush();
				rotation = r;
				moveTo(x_goal, x, y, curr_x, curr_y, rotation);
			}
			if(s.size()>=2 && s.substr(0,2) == "Y ") {
				istringstream is(s.substr(2,-1));
				double y;
				is >> y;
				cout << "Set Y to " << y << endl;
				cout.flush();
				x_goal[Y] = y;
			}
			if(s.substr(0,5) == "PICK ") {
				string block_type = s.substr(6);
				if(block_type == "O")
					pickUpBlock(PumaRobot, squarePos, serial); 
				else if(block_type == "I")
					pickUpBlock(PumaRobot, linePos, serial); 
				else if (block_type == "J")
					pickUpBlock(PumaRobot, reverseLPos, serial);
				else if (block_type == "L")
					pickUpBlock(PumaRobot, lPos, serial); 
				else if (block_type == "T")
					pickUpBlock(PumaRobot, tPos, serial);
				else if (block_type == "S")
					pickUpBlock(PumaRobot, sPos, serial);
				else if (block_type == "Z")
					pickUpBlock(PumaRobot, zPos, serial);
				else
					pickUpBlock(PumaRobot, squarePos, serial); 
			}
			if(s=="PLACE") {
				x_goal[Y]=-0.8;
				GentlyMoveGOTO(PumaRobot, x_goal, verylow_gains);
				magnetOff(serial);
				_sleep(1000); 
				x_goal[Y]=-0.7;
				MoveGOTO(PumaRobot, x_goal, x_, default_gains);
				moveTo(x_goal, 5, 0, curr_x, curr_y, 0);
				MoveGOTO(PumaRobot, x_goal, x_, default_gains);
			} else {
				//MoveGOTO(PumaRobot, x_goal, x_, high_gains_last_joint);
				MoveGOTO(PumaRobot, x_goal, x_, default_gains);
			}
			TetrisServer->sendOK();
		}
	}

	PumaRobot->_float();
	//Sleep(2000);
	PumaRobot->~RobotCom();

	return 0;
}

