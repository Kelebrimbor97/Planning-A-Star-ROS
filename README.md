# Simulation of A Star algorithm in Gazebo/Rviz

This is an extension of a previous project, [Path Planning for a mobile Robot using A star](https://github.com/Kelebrimbor97/Path-Planning-for-a-mobile-robot-using-A-Star). In this rendition of the project we extend it to Gazebo.

## Instructions for running

For running the source code, user needs to just run the python file named "prj3_p2.py" in the Part1 folder.
This code will ask for the initial state and orientation, goal state amd 2 RPMs as input ( for e.g. 100, 200) and then give the visualization for the generated path. Before visualization, it will check the validity of the initial and goal state, and will prompt user to give inputs again untill correct inputs are provided.

For running the simulation, user needs to have turtlebot installed as the launch file used the 'turtlebot_description' package.
Then open a terminal and launch the turtlebot3_map_world.launch file using roslaunch command.
After that open another terminal in the project3_phase2/src folder (the one which contains the python scripts). First run the 'prj3_p2.py' file. This file generates the 'path_points.txt' file. After that, run the 'test.py' script which is our publisher for the turtlebot simulations.


## Link for the videos

Part1 - 
https://drive.google.com/file/d/1prnu1g-J7xPA9oQb4e5J5xQdx6Hv3Z0j/view?usp=share_link

Part2 - 
https://drive.google.com/file/d/1cpuK_QUZMuU6Pqd_sZwWJxfIZu9WEo2G/view?usp=share_link
