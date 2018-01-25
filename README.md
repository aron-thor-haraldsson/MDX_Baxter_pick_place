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



Usage:

When everything is set up as described in the above steps,
perform the following steps to use the package:
- go to your workspace
- execute 'baxter.sh'
- rosrun 'enable.py -e' from the package 'baxter_tools'
- rosrun 'tuck.py -u' from the package 'baxter_tools'
- roslaunch the 'baxter_pick_place.launch' from the package 'baxter_pick_place_pkg'
- enjoy

Here are a couple of youtube video links that demonstrate how the code should work:
https://youtu.be/VWPVNY-5a0w
https://youtu.be/y2IjncEcM5o



Maintainers:

Aron Haraldsson - ronnamura@gmail.com



FAQ/Troubleshooting:

Problem:
    The robot arm starts a routine, but gets stuck during centering or descent.
Solution:
    The centering process is too precise to be carried out by your Baxter.
    Edit the 'move_message_generator.py' lines 111 and 113
    so that the values (default is 0.0055) are increased slightly.
    If you increase these values too much,
    the centering process will become increasingly inaccurate.

Problem:
    The shape is incorrectly classified.
Solution:
    You may have to relearn the ML algorithm.
    You can run the 'relearn_model.py' to do this.
    Note that every time you do this, results vary.

Problem:
    There is one huge contour (larger than the actual shape on the video)
    or there are multiple contours detected.
Solution:
    There is too much clutter on the work area.
    Try to remove or cover up all coloured parts so that the work area is white.
    Try to eliminate shadows cast on the table by provide better lighting
    from multiple angles.
    Alternatively, if you are feeling confident and have read and understood
    how image preprocessing is performed in 'process_images.py', you can
    start to tweak the parameters passed in the function call in line 62
    in the 'track_shape.py' file.

Problem:
    The cube is outside the field of view of the Baxter camera arm.
Solution:
    Move the cube inside the field of view
    or pull the Baxter arm to a position where it can see the cube.

Problem:
    During descent, the Baxter arms moves to far or too short
    to successfully pick up the cube.
Solution:
    Edit the 'cartesian_movement.py' file.
    There, find the pick_place() function.
    In it you can tweak the two move_down() function arguments.
    The argument in first call should be 0.1 less than the argument in the second call.
    The argument in the second call should be the distance, in meters,
    between the table and the tip of the gripper on the Baxter arm.

Problem:
    The gripper consistently misses the cube or collides with it.
Solution:
    Edit the 'move_message_generator.py' file.
    In line 86 and 87 'y_diff' and 'x_diff',
    the numerical values can be tweaked a bit.
    This affects how the arm centres on the target.
    Be careful, since this is a very fiddly setting to manipulate.

Problem:
    You want the Baxter to use the opposite arm instead.
Solution:
    Edit the 'cartesian_movement.py' file.
    Go to where 'cartesian_move()' is declared in line 71.
    Change the default value of the 'limb_arg' argument
    to either "left" or "right".

Problem:
    You want to change the 'home' position of the Baxter arm.
Solution:
    Edit the 'cartesian_movement.py' file.
    Go to where 'default_move' variable is assigned in line 26.
    It is an array containing the x, y and z coordinates
    of the default 'home' position.
    Change these values as you see fit.

Problem:
    You want the orientation of the baxter arm to be something
    else than pointing straight down.
Solution:
    Edit the 'cartesian_movement.py' file.
    You should really know what you are doing
    and be familiar with the quaternion number system.
    You need to modify the last few lines
    in the 'cartesian_move()' function.
