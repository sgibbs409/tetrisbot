// PumaClient.cpp : Defines the entry point for the console application.
//
#pragma once

#include "RobotCom.h"
#include <iostream>
#include <omp.h>
//#include <tchar.h>

//you will need to change PrNetworkDefn and Robot.cpp based on the 
//QNX computer you are using
//Copy cs225a.h from your cs225asim directory
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
const float SQUARE_SIZE = 0.03;

#define XREACHEDTOL 0.1
#define QREACHEDTOL 20 // fine tune this parameter
#define REACHEDITER 100 // fine tune this parameter

// number of arguments describing the end-effector
const int X_DOF = 7; // task space
const int J_DOF = 6; // Joint space

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
  
  x_goal[X] = SQUARE_SIZE*(curr_x-NUM_SQUARES_WIDE/2);
  x_goal[Z] = -SQUARE_SIZE*(curr_y-NUM_SQUARES_HIGH/2);

 // x_goal[GAMMA] = (rotation * 90) % 360;
  
  cout << "currx: " << curr_x << "  curr_y: " << curr_y << " curr_rotation:  " << x_goal[GAMMA] << endl;
}

void waitForStart(RobotCom *PumaRobot)
{
	PumaRobot->_float();
    cout << "Float..." << endl;
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

/*
// Thresholding constants
int iLowH = 166;
int iHighH = 178;

int iLowS = 201; 
int iHighS = 255;

int iLowV = 114;
int iHighV = 255;

int MAX_X;
int MAX_Y;


VideoCapture initCamera()
{
	VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return NULL;
    }

	cout << "initing" << endl;

	Mat tempMat;
	cap.read(tempMat);
	MAX_X = tempMat.cols;
	MAX_Y = tempMat.rows; 

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 360); //Hue (0 - 360)
	createTrackbar("HighH", "Control", &iHighH, 360);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	return cap;
}

Point GetCentroid(Mat &img)
{
	Point retval;

	//Calculate the moments of the thresholded image
	Moments oMoments = moments(img);
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 10000)
	{
		//calculate the position
		retval.x = dM10 / dArea;
		retval.y = dM01 / dArea;        
        
		//Draw a circle on the center point
		circle(img, retval, 3, Scalar(255,255,255), 2);

		// draw target, center of the image
		circle(img, Point(MAX_X/2, MAX_Y/2), 3, Scalar(100,100,255),2);
	}

	return retval;
}

bool UpdateCamera(VideoCapture &cap, Mat &img)
{
    Mat imgOriginal;

    bool bSuccess = cap.read(imgOriginal); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
        cout << "Cannot read a frame from video stream" << endl;
        return false;
    }

	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), img); //Threshold the image
      
	//morphological opening (remove small objects from the foreground)
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	GetCentroid(img);

	imshow("Thresholded Image", img); //show the thresholded image
	imshow("Original", imgOriginal); //show the original image

	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		cout << "esc key is pressed by user" << endl;
	}

	return true;
}

// Using a webcam on the end-effector.
void AimEndEffector(RobotCom* PumaRobot)
{
	//Point targetPoint = 
}
*/

// Move to joint position via jgoto
void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq)
{
    cout << "Start JGOTO" << endl;
	 // Output the joint command
	 Robot->jointControl(JGOTO, qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
	 
	 // Wait for the robot to finish motion
     do
	 {
	 Robot->getStatus(GET_JPOS,q);
	 Robot->getStatus(GET_JVEL,dq);
	 }while(!jposReached(qd,q));
    cout << "Complete JGOTO" << endl;
}

void MoveGOTO(RobotCom *Robot, float *xd, float *x)
{
    cout << "Start GOTO" << endl;
	 // Output the joint command
	 Robot->control(GOTO, xd, 7);
	 
	 // Wait for the robot to finish motion
     do
	 {
	 Robot->getStatus(GET_IPOS,x);
	 }while(!xposReached(xd,x));
    cout << "Complete GOTO" << endl;
}

void pickUpBlock()
{
	// fill in
}

void goHome(RobotCom* bot, float *x_goal)
{
	float xd_[X_DOF] = {0, -0.7, 0, 0.5,0.5,0.5,-0.5};
	for(int i=0; i<X_DOF; i++) x_goal[i] = xd_[i];
	x_goal = xd_;
	float x_[X_DOF];
	
	float qd_[J_DOF] = {-90,-45,180,0,-45,0}; // in degrees
	float dq_[J_DOF], q_[J_DOF]; 
	  
	MoveJGOTO(bot, qd_, q_, dq_);
	MoveGOTO(bot, xd_, x_);
}

void moveToTop(RobotCom* bot, float *x_goal)
{
	float xd_[X_DOF] = {0, -0.7, 0.3, 0.5,0.5,0.5,-0.5};
	for(int i=0; i<X_DOF; i++) x_goal[i] = xd_[i];
	float x_[X_DOF];
	
	MoveGOTO(bot, xd_, x_);
	cout << "top" << endl;
}

//int _tmain(int argc, _TCHAR* argv[])
int main(int argc, char **argv)
{
	// start up
	float x_goal[X_DOF];
	RobotCom* PumaRobot = new RobotCom();
	waitForStart(PumaRobot);
	goHome(PumaRobot, x_goal);
	cout << "home!" << endl;
	//cin << "press any key to continue";

	//moveToTop(PumaRobot, x_goal);

	// Initialize Camera
	//VideoCapture cap = initCamera();
	
	// initialize game board variables
	int curr_x = NUM_SQUARES_WIDE/2; int curr_y = NUM_SQUARES_WIDE/2;
	int rotation=0;

	// Threading stuffs
	#pragma omp parallel num_threads(2)
	  {
		int thread_id = omp_get_thread_num();
		
		if(thread_id==0) 
		{
			//Mat img;
			while(true) {
				float x_[X_DOF];
				MoveGOTO(PumaRobot, x_goal, x_);
				//if(!UpdateCamera(cap, img)) break;
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

	  PumaRobot->_float();
	  //Sleep(2000);
	  PumaRobot->~RobotCom();

  return 0;
}

