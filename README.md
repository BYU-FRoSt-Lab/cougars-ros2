This repo contains ROS 2 packages, mission launch files, and testing scripts used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)**. 
If you're doing code development, feel free to clone this repo directly, make a new branch, and start coding and pushing some changes.
Otherwise (if you're looking to set up a new Coug-UV from scratch), see this repo for instructions on how to get our custom Docker image running instead: https://github.com/snelsondurrant/CougarsSetup

A quick high-level overview of the repo:
- **src/** - contains the ROS 2 packages we use to run the Coug-UV.
Of note are "src/cougars_cpp/" and "src/cougars_py" (miscellaneous custom nodes running MOOS-ROS 2 bridges, PID controls, odometry conversions, etc), as well as "src/frost_interfaces/" (contains custom ROS message and service declarations).
The ROS launch files for different start configurations are included in "src/cougars_py/launch/".
- **moos_tools/** - start scripts, mission files, and behavior files for MOOS-IvP mission planning and execution.
TODO: Add a more helpful description here in the future.
- **microros_tools/** - helpful scripts for launching the microROS agent ("agent.sh"), syncing custom message changes ("msg_sync.sh"), and testing Teensy board connections ("test.sh")
A description of what each script does is included as a header comment in the file.
