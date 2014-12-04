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
#define QREACHEDTOL 40 // fine tune this parameter
#define DQREACHEDTOL 20 // fine tune this parameter
#define REACHEDITER 100 // fine tune this parameter

// number of arguments describing the end-effector
const int X_DOF = 7; // task space
const int J_DOF = 6; // Joint space

// home position
float HOME_XPOS[X_DOF] = {0, -0.70, 0, 0.5, 0.5, 0.5,-0.5};
float HOME_JPOS[J_DOF] = {-107.6, -57.4, 207, -19.9,-61, 9.8};

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
float half_gains[12] = { 200, 200, 200, 200, 200, 200, 20, 20, 20, 20, 20, 20 };
float high_gains_last_joint[12] = { 400, 400, 400, 400, 400, 4000, 40, 40, 40, 40, 40, 80 };
float low_gains[12] = { 100, 100, 100, 100, 100, 100, 10, 10, 10, 10, 10, 10 };
float testlow_gains[12] = { 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 40 };
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

void setGains(RobotCom *Robot, float *gains) {
	Robot->setGains(JGOTO, gains);
	Robot->setGains(GOTO, gains);
	Robot->setGains(FLOATMODE, gains);
	Robot->setGains(NO_CONTROL, gains);
	getGains(Robot);
}


void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq, float *gains)
{
	cout << "Start JGOTO" << endl;
        setGains(Robot, gains);
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


void MoveGOTO(RobotCom *Robot, float *xd, float *x, float *gains)
{
	cout << "Start GOTO" << endl;
        setGains(Robot, gains);
	// Output the joint command
	Robot->control(GOTO, xd, 7);

	// Wait for the robot to finish motion
	do
	{
		Robot->getStatus(GET_IPOS,x);
	}while(!xposReached(xd,x));
	cout << "Complete GOTO" << endl;
}


void GentlyMoveAlongAxisUntilStopped(RobotCom *Robot, float *xi, int AXIS, float distance, float stepSize) {
	cout << "Start move along axis" << endl;

        float x[X_DOF], xd[X_DOF], dq[J_DOF];
        float signed_step = (distance>0 ? stepSize : -stepSize);
        bool increasing = (distance>0);

        for(int i=0; i<X_DOF; i++) xd[i] = xi[i];

	MoveGOTO(Robot, xi, x, default_gains);

	// Set gains
        setGains(Robot, default_gains);

        do {
                Robot->getStatus(GET_IPOS, x);
                xd[AXIS] = x[AXIS] + signed_step;
                if(increasing)  xd[AXIS] = min(xd[AXIS], xi[AXIS]+distance);
                if(!increasing) xd[AXIS] = max(xd[AXIS], xi[AXIS]-distance);
                Robot->control(GOTO, xd, X_DOF);
		Robot->getStatus(GET_JVEL,dq);
        } while(!jposStopped(dq));

	cout << "Complete move along axis" << endl;
}

// Move towards the goal until you are stopped
void GentlyMoveGOTO(RobotCom *Robot, float *xd, float *gains) {
	cout << "Start gentle GOTO" << endl;

	// Set gains
        setGains(Robot, gains);

	// Output the joint command
	Robot->control(GOTO, xd, X_DOF);

    // Wait for the robot to stop
	float dq[J_DOF];
	do
	{
		Robot->getStatus(GET_JVEL,dq);
	}while(!jposStopped(dq));
	cout << "Complete gentle GOTO" << endl;
}

void prepPickUp(RobotCom* bot, HANDLE & serial)
{
	float prep_pick_pos[J_DOF];
	for(int i = 0; i < J_DOF; i++) prep_pick_pos[i] = HOME_JPOS[i];
	prep_pick_pos[0] += 90;
	prep_pick_pos[4] += 90;

	float dq_[J_DOF], q_[J_DOF]; 

	MoveJGOTO(bot, prep_pick_pos, q_, dq_, half_gains);
}

void pickUpBlock(RobotCom* bot, float *block_pos, HANDLE & serial)
{
	// Set up variables
	float x_[X_DOF];
	
	float pick_up_pos[X_DOF];
	for(int i = 0; i < X_DOF; i++) pick_up_pos[i] = block_pos[i];
	pick_up_pos[Z] -= 0.1;

	// Go home first
	float dq_[J_DOF], q_[J_DOF]; 
	MoveJGOTO(bot, HOME_JPOS, q_, dq_, half_gains);
	// Go to prepare for pick up position
	prepPickUp(bot, serial);

	// Pick up block
	MoveGOTO(bot, block_pos, x_, default_gains);
	
	char k;
	cin >> k;

	GentlyMoveGOTO(bot, pick_up_pos, low_gains);

	magnetOn(serial);
	cout << "magnet On" << endl;

	GentlyMoveGOTO(bot, block_pos, low_gains);

	// Go back to to prepare for pick up position
	prepPickUp(bot, serial);

	//go home after you have the block
	MoveJGOTO(bot, HOME_JPOS, q_, dq_, half_gains);
	MoveGOTO(bot, HOME_XPOS, x_, default_gains);

	cin >> k;
}

void goHome(RobotCom* bot, float *x_goal)
{	
	for(int i=0; i<X_DOF; i++) x_goal[i] = HOME_XPOS[i];
	
	float x_[X_DOF];
	float dq_[J_DOF], q_[J_DOF]; 
	
	MoveJGOTO(bot, HOME_JPOS, q_, dq_, half_gains);
	MoveGOTO(bot, HOME_XPOS, x_, default_gains);
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

	for (int i = 0; i < X_DOF; i++) 
		cout << "read position " << position_array[i] << " " << endl;
}

void loadPositionArray(float *pos_arr, ifstream & fin)
{
	string line;
	getline(fin, line);
	istringstream iss(line);

	for(int i = 0; i < X_DOF; i++)
		iss >> pos_arr[i];

	for (int i = 0; i < X_DOF; i++) 
		cout << "loading from file " << pos_arr[i] << " " << endl;

}

void writePositionArray(float *pos_arr, ofstream & fout)
{
	for(int i = 0; i < X_DOF; i++)
		fout << pos_arr[i] << " ";
	fout << endl;
}

void calibratePositions(RobotCom* PumaRobot, HANDLE & serial)
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
				squarePos[i] = HOME_XPOS[i];
			return;
		}

		// These arrays need to be in the same order as the ones
		// in the recordBlockPos below.
		loadPositionArray(squarePos, infile);
		loadPositionArray(linePos, infile);
		loadPositionArray(lPos, infile);
		loadPositionArray(reverseLPos, infile);
		loadPositionArray(tPos, infile);
		loadPositionArray(sPos, infile);
		loadPositionArray(zPos, infile);

		infile.close();
		return;
	}

	prepPickUp(PumaRobot, serial);

	// recalibrate, the order of these needs to be the 
	// same as the order of the arrays above.
	recordBlockPos(squarePos, "SQUARE", PumaRobot);
	recordBlockPos(linePos, "LINE", PumaRobot);
	recordBlockPos(lPos, "L", PumaRobot);
	recordBlockPos(reverseLPos, "REVERSE L", PumaRobot);
	recordBlockPos(tPos, "T", PumaRobot);
	recordBlockPos(sPos, "S", PumaRobot);
	recordBlockPos(zPos, "Z", PumaRobot);

	// record the positions
	ofstream outfile(BLOCK_POS_FILENAME, ofstream::out | ofstream::trunc);

	// the order of these needs to be the 
	// same as the order of the arrays above.
	writePositionArray(squarePos, outfile);
	writePositionArray(linePos, outfile);
	writePositionArray(lPos, outfile);
	writePositionArray(reverseLPos, outfile);
	writePositionArray(tPos, outfile);
	writePositionArray(sPos, outfile);
	writePositionArray(zPos, outfile);

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

	RobotCom* PumaRobot = new RobotCom();

	waitForStart(PumaRobot);

	calibratePositions(PumaRobot, serial);

	goHome(PumaRobot, x_goal);
	cout << "home!" << endl;

	char key;
	cout << "press any key to continue";
	cin >> key;

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
				string block_type = s.substr(5);
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
				GentlyMoveGOTO(PumaRobot, x_goal, half_gains);
				magnetOff(serial);
				_sleep(1000); 
				x_goal[Y]=-0.70;
				MoveGOTO(PumaRobot, x_goal, x_, half_gains);
			} else {
				//MoveGOTO(PumaRobot, x_goal, x_, high_gains_last_joint);
				MoveGOTO(PumaRobot, x_goal, x_, default_gains);
			}
			if(s == "M ON")
			{
				magnetOn(serial);
			}
			if(s == "M OFF")
			{
				magnetOff(serial);
			}

			TetrisServer->sendOK();
		}
	}

	PumaRobot->_float();
	Sleep(2000);
	PumaRobot->~RobotCom();

	return 0;
}

