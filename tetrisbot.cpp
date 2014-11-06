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
/* \file scl_tutorial4_control_gc_op.cpp
 *
 *  Created on: Jul 30, 2014
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
#include <scl/util/DatabaseUtils.hpp>
//scl functions to simplify dynamic typing and data sharing
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/util/HelperFunctions.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <fstream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

using namespace std;

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 3, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */
int main(int argc, char* argv[])
{
//  std::cout<<"\n***************************************\n";
//  std::cout<<"Standard Control Library Tutorial #4";
//  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...
  sutil::CSystemClock::start();

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
  bool flag = p.readRobotFromFile(fname,"PumaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile(fname,"PumaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  //std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long iter = 0, n_iters=200000; double dt=0.0001;

  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;
  Eigen::VectorXd hpos(6); //control position of op-point wrt. hand
  hpos.setZero();
  Eigen::MatrixXd J;
  Eigen::VectorXd x(6);
  Eigen::VectorXd x_des(6);
  Eigen::VectorXd x_init(6);
  Eigen::VectorXd dx(6);
  scl::SRigidBodyDyn *ee = rgcm.rbdyn_tree_.at("end-effector");

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      tstart = sutil::CSystemClock::getSysTime(); iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // Compute kinematic quantities
        dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
        dyn_scl.computeJacobianWithTransforms(J,*ee,rio.sensors_.q_,hpos.segment(0,3));

        // update current end-effector position and orientation
        if(false == flag) {
            Eigen::Vector3d temp =  hpos.segment(0,3);
	     x_init.segment(0,3) = ee->T_o_lnk_ * temp;
	     x_init.segment(2,3) = ee->T_o_lnk_.rotation() * hpos.segment(2,3);
	     flag = true;
        }
        Eigen::Vector3d temp =  hpos.segment(0,3);
        x.segment(0,3)  = ee->T_o_lnk_ * temp;
        x.segment(2,3)  = ee->T_o_lnk_.rotation() * hpos.segment(2,3);

        dx = J * rio.sensors_.dq_;

	float kp = 1;
	float kv = 1;

	x_des.setZero();
        x_des(0) = x_init(0)+0.1;
        x_des(1) = x_init(1)+0.1;
        rio.actuators_.force_gc_commanded_ = J.transpose() * (kp*(x_des-x) - kv * dx) + rgcm.force_gc_grav_;

        cout << "x " << x << endl;
        cout << "dx " << dx << endl;
        cout << "x_des " << x_des << endl;

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL);

      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
//  std::cout<<"\n\nExecuted Successfully";
//  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
