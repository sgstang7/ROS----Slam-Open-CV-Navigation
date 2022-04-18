# ROS----Slam-Open-CV-Navigation
When get the zip document of the code, unzip it in a working
environment. Start the terminal in the working window and build the target, by calling
“catkin_make” and “source devel/setup.bash”. Then, start simulation by calling “roslaunch
wpr_simulation wpb_gmapping.launch” and start to control the movement in a new tab by
calling “rosrun wpr_simulation keyboard_vel_ctrl”. When the map scan is completed, the
scanned map can be saved by command “rosrun map_server map_saver -f mymap”. After that, 
starting the navigation by calling “roslaunch wpr_simulation wpb_navigation.launch”, then 
start the face detection by calling at “rosrun wpr_simulation demo_cv_face_detect”.
