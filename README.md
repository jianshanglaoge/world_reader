# World_Reader

This is the repository for random world items generator program and reader for it.

## Before Using 

Because this work is base on the Ros. Please delect all the 'build','dev' files and 'CMakeLists.txt' in 'src' file. Then do the ```catkin_make``` and ```source devel/setup.bash``` to make the program fits your computer.

## Running the Programe

You can directly run the ```roslaunch arlo_gazebo wang_gazebo.launch world_num:=world_num``` to open the gazebo and load the world you want.

## TODO

### Random generator
Now the random generator part are not completly good enough. There is no rotate and no collision aviodance. We will add them in the following works.


**This work are contacted with the Aryaan's work [World_Generator_Script](https://github.com/dexterfire861/World_Generator_Script.git ).**
