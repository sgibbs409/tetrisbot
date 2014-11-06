/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file scl_tutorial5_multi_task.cpp
 *
 *  Created on: Aug 10, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
//scl functions to simplify dynamic typing and data sharing
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/HelperFunctions.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

using namespace std;

// constants of game board
const int NUM_SQUARES_HIGH = 20;
const int NUM_SQUARES_WIDE = 10;
const float SQUARE_SIZE = 0.035;

// Translation controls
void moveLeft(Eigen::VectorXd & x_goal, int &curr_x, int &curr_y)
{
  if(curr_x > 0)
    {
      x_goal(1) -= SQUARE_SIZE;
      curr_x--;
      cout << "moveLeft" << endl;
      cout << "currx: " << curr_x << "  curr_y: " << curr_y << endl;
    }
}

void moveRight(Eigen::VectorXd & x_goal, int &curr_x, int &curr_y)
{
  if(curr_x < NUM_SQUARES_WIDE)
    {
      x_goal(1) += SQUARE_SIZE;
      curr_x++;
      cout << "moveRight" << endl;
      cout << "currx: " << curr_x << "  curr_y: " << curr_y << endl;
    }
}

void moveUp(Eigen::VectorXd & x_goal, int &curr_x, int &curr_y)
{
  if(curr_y < NUM_SQUARES_HIGH)
    {
      x_goal(2) += SQUARE_SIZE;
      curr_y++;
      cout << "moveUp" << endl;
      cout << "currx: " << curr_x << "  curr_y: " << curr_y << endl;
    }
}

void moveDown(Eigen::VectorXd & x_goal, int &curr_x, int &curr_y)
{
  if(curr_y > 0)
    {
      x_goal(2) -= SQUARE_SIZE;
      curr_y--;
      cout << "moveDown" << endl;
      cout << "currx: " << curr_x << "  curr_y: " << curr_y << endl;
    }
}


void moveTo(Eigen::VectorXd & x_goal, int target_x, int target_y, int &curr_x, int &curr_y)
{
  curr_y = max(0, min(NUM_SQUARES_HIGH, target_y));
  curr_x = max(0, min(NUM_SQUARES_WIDE, target_x));
  x_goal(2) = SQUARE_SIZE*(curr_y-NUM_SQUARES_HIGH/2);
  x_goal(1) = SQUARE_SIZE*(curr_x-NUM_SQUARES_WIDE/2);
  cout << "moveTo" << endl;
  cout << "currx: " << curr_x << "  curr_y: " << curr_y << endl;
}




/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 4, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #5";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
  scl::CControllerMultiTask rctr;    //A multi-task controller
  std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
  std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
  //std::string must_use_robot;        //Used later for file error checks.
  std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
  scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.
  //scl::* rtask_hand;       //Will need to set hand desired positions etc.

  sutil::CSystemClock::start(); //Start the clock

  /******************************Set up Shared Memory (Database)************************************/
  // This will help scl find the graphics files. Usually they are specified wrt. some
  // specs dir. An alternate way is to set this dir as an environment variable (but we won't
  // do that for now). Instead we'll set the dir in a global shared memory "database".
  if(NULL == scl::CDatabase::getData()) { std::cout<<"\n ERROR : Could not initialize global data storage"; return 1; }
  scl_util::getCurrentDir(scl::CDatabase::getData()->cwd_);
  scl::CDatabase::getData()->dir_specs_ = scl::CDatabase::getData()->cwd_ + std::string("../../specs/");

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  const std::string fname("../../specs/Puma/PumaCfg.xml");
  bool flag = p.readRobotFromFile(fname,"../../specs/","PumaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);        //Set up the I/O data structure
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************Set up Controller Specification************************************/
  // Read xml file info into task specifications.
  flag = p.readTaskControllerFromFile(fname,"opc",rtasks,rtasks_nc,ctrl_params);
  //if(must_use_robot != "PumaBot") {flag = false;} //Error check for file consistency
  flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
  // Tasks are initialized after find their type with dynamic typing.
  flag = flag && scl_registry::registerNativeDynamicTypes();
  flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
  flag = flag && rctr.init(&rctr_ds,&dyn_scl);        //Set up the controller (needs parsed data and a dyn object)
  if(false == flag){ return 1; }            //Error check.

  rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
  if(NULL == rtask_hand)  {return 1;}       //Error check

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile(fname,"PumaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the robot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long long iter = 0; double dt=0.0001;

  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    bool is_button_pressed = false; // for debouncing
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      tstart = sutil::CSystemClock::getSysTime(); iter = 0;

      // initialize
      rtask_hand->x_goal_(0) = 0.2;
      rtask_hand->x_goal_(1) = 0;
      rtask_hand->x_goal_(2) = 0;
      int curr_x = NUM_SQUARES_WIDE/2; int curr_y = NUM_SQUARES_WIDE/2;

      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // translate on awsd key press
        if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active['a'])
         {
            if(!is_button_pressed)
              moveLeft(rtask_hand->x_goal_, curr_x, curr_y);
            is_button_pressed = true; // for debouncing
         }
        else if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active['s'])
        {
            if(!is_button_pressed)
		moveDown(rtask_hand->x_goal_, curr_x, curr_y);
	    is_button_pressed = true; // for debouncing
	} else if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active['d'])
	{
	    if(!is_button_pressed)
		moveRight(rtask_hand->x_goal_, curr_x, curr_y);
	    is_button_pressed = true; // for debouncing
	} else if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active['w'])
	{
	    if(!is_button_pressed)
		moveUp(rtask_hand->x_goal_, curr_x, curr_y);
	    is_button_pressed = true; // for debouncing
	} else if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active['z'])
	{
	    if(!is_button_pressed)
		moveUp(rtask_hand->x_goal_, curr_x, curr_y);
	    is_button_pressed = true; // for debouncing
	} else {
	    is_button_pressed = false;
	}

	// Set up the keypad 1 through 9 keys to go to the corners and centers
	// I don't think we need to care about bouncing in this case
	double xf[] = {0,1,2,0,1,2,0,1,2};
	double yf[] = {0,0,0,1,1,1,2,2,2};
	// actually, no.  for some reason the number keys don't work well?? use yuihjknm, instead
	// actually, no.  those are already used for something-ish, and also seems to not work.
	char letters[] = "nm,hjkyui"; // actually, yes, these do work
	for(int i=0; i<9; i++) {
	  if(scl_chai_glut_interface::CChaiGlobals::getData()->keys_active[letters[i]]) {
	    std::cout << "Huh" << std::endl;
	    std::cout.flush();
	    moveTo(rtask_hand->x_goal_, NUM_SQUARES_WIDE*xf[i]/2, NUM_SQUARES_HIGH*yf[i]/2, curr_x, curr_y);
	  }
	}

        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();
        rctr.computeControlForces();

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++;
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendered robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
  std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
