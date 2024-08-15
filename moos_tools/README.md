# Project Title

Simple overview of use/purpose.

## Description

An in-depth paragraph about your project and overview of use.

To use the missions, you need to put the latitude,longitude origin in the .moos mission file that you
are using. Then in the behavior file, if you are just doing waypoints, just put your x,y (meters) waypoints
that you want to follow.

mission_start_processes.sh just starts all the moos apps

mission_deploy.sh actually allows the behaviors to begin by setting DEPLOY = true

mission_kill.sh cleans up all processes 