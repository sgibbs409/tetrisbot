// PumaClient.cpp : Defines the entry point for the console application.
//
#ifdef WIN32
#pragma once
#endif

#include "RobotCom.h"
#include <iostream>
using namespace std;

int main(int argc, const char* argv[])
{
	RobotCom* PumaRobot = new RobotCom();

	PumaRobot->_float();

    std::cout << "hello worlds" << std::endl;
   float q[6], x[10];
   string s;

	while(1)
	{
		PumaRobot->getStatus(GET_JPOS, q);
		for(int i=0; i<6; i++) cout << q[i] << " "; cout << "; ";
		PumaRobot->getStatus(GET_IPOS, x);
		for(int i=0; i<10; i++) cout << x[i] << " "; cout << endl;
	}

	return 0;
}

