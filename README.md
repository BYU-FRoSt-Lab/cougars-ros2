This repo contains ROS 2 packages, mission launch files, and testing scripts used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)**. 
If you're doing code development, feel free to clone this repo directly, make a new branch, and start coding and pushing some changes.
Otherwise (if you're looking to set up a new Coug-UV from scratch), see this repo for instructions on how to get our custom Docker image running instead: https://github.com/snelsondurrant/CougarsSetup

A quick high-level overview of the repo:
- **src/** - contains the ROS 2 packages we use to run the Coug-UV.
Of note are "src/cougars/" (runs high-level controller logic, kind of a master package) and "src/frost_interfaces/" (contains custom ROS message and service declarations).
The ROS launch file is included in "src/cougars/launch/".
- **scripts (agent.sh, msg_sync.sh, etc)** - automates helpful software tasks on the AUV.
For example, running "bash start.sh" will load in the local config values, start the microROS agent, run the ROS launch file, and start logging data.
A description of what each script does is included as a header comment in the file.
