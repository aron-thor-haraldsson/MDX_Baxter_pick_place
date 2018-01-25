Introduction:

This is a package that connects to a Baxter robot and performs a pick and place routine.
The code uses image processing and machine learning to recognise shapes
that the Baxter arm then picks up and relocates.



Requirements/Installation:

- a Baxter robot that is fully set up and functional (2)
- a computer to run the code from
- both of these need to be connected to the same router to be assigned IP addresses
- the computer should be using Ubuntu 16.04 (1)
- the computer needs to have ROS (3)
- the computer needs to have Baxter Workstation on it (2)
- the 'baxter.sh' from the workspace root directory should be copied to your workspace
- the 'baxter.sh' needs to be modified to include the address of your computer and Baxter
- the package 'baxter_pick_place_pkg' should also be added to the src directory of your workspace
- the 'devel' and 'build' folders of the workspace do not need to be copied
- your workspace needs to be built using 'catkin_make'

- your computer needs to have Python 2.x installed with the following packages installed:
    numpy, scipy, scikit-learn, cv-bridge adn opencv
- the rest of the dependencies, ie. rospy, requires you to be running the .py files through ROS
- make sure that the files in 'baxter_pick_place_pkg' have executable permissions

(1) https://askubuntu.com/questions/666631/how-can-i-dual-boot-windows-10-and-ubuntu-on-a-uefi-hp-notebook
(2) http://sdk.rethinkrobotics.com/wiki/Baxter_Setup
(3) http://wiki.ros.org/kinetic/Installation/Ubuntu



Maintainers:

Aron Haraldsson - ronnamura@gmail.com
