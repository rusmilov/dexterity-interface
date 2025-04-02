# Dexterity Interface

## Setup LLM configs
1. First, you need to create a .env file in this folder with the OpenAI credentials. It should be in this format:
    ```bash
    OPENAI_API=YOUR_API_KEY_HERE
    ```



## Set up the repository and run the container
1. Check prerequisites for [panda-primitives](https://github.com/Wisc-HCI/panda-primitives) and [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control)
   
2. Bring in the submodules:
    ```bash
    git submodule update --init --recursive
    ```
3. Setup display forwarding:
    ```bash
    xhost +local:
    ```
4. Now  build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
    ```bash
    sudo docker build -t llm-control .

    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host llm-control
    ```

## Compile panda-primitives-control package(SKIP FOR SIMULATION)
This step will compile panda-primitives-control package that control Panda Robot in low level, you can skip it if you only need simulation

1. Add necessary environment variables: Replace with your Panda's IP
    ```bash
    export PANDA_IP=192.168.1.3
    ```
2. ### Compile non-ROS package (PandaController)

    If first time, first configure:
    ```bash
    cd panda-primitives-control/src/PandaController
    mkdir build
    cd build
    cmake ..
    cd ../../../..
    ```

    Anytime, run:
    ```bash
    cd panda-primitives-control/src/PandaController/build
    make install
    cd ../../../..
    ```

    ### Compile ROS package
    Compile individually each ros packages:
    ```bash
    cd panda-primitives-control
    # catkin build relaxed_ik --no-notify
    catkin build panda_ros_msgs --no-notify
    catkin build panda_ros --no-notify
    #catkin build dmp_deformations --no-notify
    catkin build inputs_ros --no-notify
    catkin build controller --no-notify
    cd ..
    ```
3. Run with ROS
    1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
    2. Run `source panda-primitives-control/devel/setup.bash` inside the root directory
    3. Start the launch files related to the application:
		```bash
		roslaunch controller mover_test.launch
		```
For more information, please refer to [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control)

## Run
Run the following:
```bash
cd dexterity-interface
source devel/setup.bash

roslaunch interface backend.launch

# Run this ina another terminal
rosrun interface llm_handler.py
```





Mya notes, please ignore:
```bash
# cd panda-primitives
# catkin build

cd dexterity-interface
catkin build


# source ../panda-primitives/devel/setup.bash
source devel/setup.bash

roslaunch interface backend.launch

rosrun interface llm_handler.py


# Troubleshooting
rosrun tf2_tools view_frames.py
```


TODO fix urdf funkiness 
TODO clean up package and workspace funkiness and duplication

https://github.com/cruise-automation/webviz
https://github.com/osrf/rvizweb

https://robotwebtools.github.io/

https://github.com/Mechazo11/interactive_marker_proxy_noetic
https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/interactive_marker_tutorials
http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality

