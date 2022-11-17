# World_Reader

This is the repository for random world items generator program and reader for it.

## Before Using 

Because this work is base on the Ros. Please delect all the 'build','dev' files and 'CMakeLists.txt' in 'src' file. Then do the ```catkin_make``` and ```source devel/setup.bash``` to make the program fits your computer.

## Running the Programe

You can directly run the ```roslaunch arlo_gazebo wang_gazebo.launch world_num:=world_num``` to open the gazebo and load the world you want.

To generate world, you can open src/arlo_simulations-main/arlo_gazebo/scripts/world_generator and do the ```python main.py``` it will auto generate 100 worlds. You can change the range in main() func to add more worlds.

To get the camera output, you can open src/arlo_simulations-main/arlo_gazebo/scripts/world_reader and do the ```python image_reader.py worldnum``` world num is the world number you are openning .It will read camera output at once and you can then Ctrl+c to stop it. Data will be saved at src/arlo_simulations-main/arlo_gazebo/scripts/world_reader/data.

## TODO

### Random generator
Now the random generator part are not completly good enough. There is no rotate and no collision aviodance. We will add them in the following works.

### Image reader
Now image reader should be run manully instead of running with roslaunch func. We will make them run together.

**This work are contacted with the Aryaan's work [World_Generator_Script](https://github.com/dexterfire861/World_Generator_Script.git ).**
