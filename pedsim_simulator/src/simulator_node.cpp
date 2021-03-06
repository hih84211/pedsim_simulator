/**
* Copyright 2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#include <signal.h>
#include <QApplication>

#include <pedsim_simulator/simulator.h>
#include <iostream>
#include <string.h>
using namespace std;

int main(int argc, char** argv)
{
    if(argc == 1) // For IDE debug
    {
        argc = 3;
        argv[0] = (char*)"/home/peter/my_ros_ws/devel/lib/pedsim_simulator/pedsim_simulator";
        argv[1] = (char*)"__name:=pedsim_simulator";
        argv[2] = (char*)"__log:=/home/peter/.ros/log/8a63eef8-b4d2-11ec-999c-d45d64b3c741/pedsim_simulator-1.log";
    }

    QApplication app(argc, argv);

    // initialize resources
    ros::init(argc, argv, "pedsim_simulator");
    ros::NodeHandle node("~");
    Simulator sm(node);

    // use default SIGINT handler so CTRL+C works
    signal(SIGINT, SIG_DFL);

    if (sm.initializeSimulation()) {
        ROS_INFO("node initialized, now running ");
        sm.runSimulation();
    } else {
        ROS_WARN("Could not initialize simulation, aborting");
        return EXIT_FAILURE;
    }

    return app.exec();
}
