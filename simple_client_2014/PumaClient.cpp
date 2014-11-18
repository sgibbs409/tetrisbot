// PumaClient.cpp : Defines the entry point for the console application.
//
#pragma once

#include "RobotCom.h"
#include <iostream>
#include <omp.h>
#include <tchar.h>

//you will need to change PrNetworkDefn and Robot.cpp based on the 
//QNX computer you are using
//Copy cs225a.h from your cs225asim directory
#include "prvector.h"
#include "prvector3.h"
#include "prmatrix.h"
#include "prmatrix3.h"
#include <fstream>
#include <sstream>
#include "param.h"
#include <string>
#include <stdlib.h>
#include "math.h"

using namespace std;

// constants of game board
const int NUM_SQUARES_HIGH = 20;
const int NUM_SQUARES_WIDE = 10;
const float SQUARE_SIZE = 0.035;

// number of arguments describing the end-effector - x,y,z,alpha,beta,gamma.
const int NUM_ARGS = 6;
enum Position {
	X,
	Y,
	Z,
	ALPHA,
	BETA,
	GAMMA
};

void moveTo(float *x_goal, int target_x, int target_y, int &curr_x, int &curr_y, int rotation)
{
  curr_y = max(0, min(NUM_SQUARES_HIGH, target_y));
  curr_x = max(0, min(NUM_SQUARES_WIDE, target_x));
  
  x_goal[Y] = SQUARE_SIZE*(curr_x-NUM_SQUARES_WIDE/2);
  x_goal[Z] = -SQUARE_SIZE*(curr_y-NUM_SQUARES_HIGH/2);

  x_goal[GAMMA] = (rotation * 90) % 360;
  
  cout << "currx: " << curr_x << "  curr_y: " << curr_y << " curr_rotation:  " << x_goal[GAMMA] << endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
	//RobotCom* PumaRobot = new RobotCom();

	//PumaRobot->_float();
	
	// initialize game board variables
	int curr_x = NUM_SQUARES_WIDE/2; int curr_y = NUM_SQUARES_WIDE/2;
	int rotation=0;
	float x_goal[NUM_ARGS];
	for (int i = 0; i < NUM_ARGS; i++)
		x_goal[i] = 0;
	x_goal[0] =  0.4; // set the end-effector forwad a bit in the x-axis direction

	// Threading stuffs
	omp_set_num_threads(2);
	int thread_id;
	#pragma omp parallel private(thread_id)
	  {
		thread_id = omp_get_thread_num();
		
		if(thread_id==1) 
		{
			while(true) {
				//PumaRobot->control(GOTO, x_goal, NUM_ARGS);
			}
		}
		else {
		  // Handle input from the game
		  std::string s;
		  while(getline(cin,s)) {
			  if(s.size()>=4 && s.substr(0,4) == "SCL ") {
				istringstream is(s.substr(4,-1));
				int x,y,r;
				is >> x >> y >> r;
				cout << s << " -> " << x << " " << y << " " << r << endl;
				cout.flush();
				rotation = r;
				moveTo(x_goal, x, y, curr_x, curr_y, rotation);
			  }
		  }
		}
	  }

  return 0;
}

