# Dexterity Interface

## Setup
### 1. Install Requirements
You will need:
* For the simulation/interface:
    * Ubuntu machine
    * [Docker Engine](https://docs.docker.com/engine/install/).
    * A live server. For VScode, we recommend [Live Server Extension](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer).

* For Running on the Panda:
    * Above requirements.
    * Franka Emika Panda Robotic arm with a Force Torque Sensor. Reference  [panda-primitives-control](https://github.com/wisc-HCI/panda-primitives-control) for the specifics.


### 2. Setup LLM configs
1. First, you need to create a .env file in this folder with the OpenAI credentials. It should be in this format:
    ```bash
    OPENAI_API=YOUR_API_KEY_HERE
    ```

### 3. Set up the repository and run the container
1. Bring in the submodules:
    ```bash
    git submodule update --init --recursive
    ```

    If you already initialized the submodules and need to make sure they are updated, run:
    ```bash
    git submodule update --remote
    ```

3. Now  build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.

    If you don't need the Kinect, run:
    ```bash
    sudo docker build -t llm-control .

    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host llm-control
    ```

    If you do need the Kinect, run:
    ```bash
    xhost +local:
    
    sudo docker build -t llm-control-kinect .

    sudo docker run -it --rm --gpus all --privileged -e DISPLAY=$DISPLAY -e PULSE_SERVER=unix:/run/user/1000/pulse/native -v /run/user/1000/pulse:/run/user/1000/pulse -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --device /dev/snd --device /dev/bus/usb --net=host llm-control-kinect
    ```
### 4. Compile ros packages

This is a bit over-complicated due to having multiple workspaces. Turning some of these workspaces into packages is a future TODO item.
```bash
cd panda-primitives
catkin build authoring
source devel/setup.bash

cd ../dexterity-interface
catkin build
source devel/setup.bash

```


### 4.5 Compile panda-primitives-control package (SKIP FOR JUST SIMULATION/INTERFACE)
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
    catkin build panda_ros_msgs --no-notify
    catkin build panda_ros --no-notify
    catkin build inputs_ros --no-notify
    catkin build controller --no-notify
    cd ..
    ```

3. Setup panda 
    1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
    2. Run `source panda-primitives-control/devel/setup.bash` inside the root directory
For more information, please refer to [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control) -->


## Running
1. Launch the Backend. Make sure you are in the `dexterity-interface/` directory when running these commands, each in a different terminal.

    1. If you want the program to run on the robot run these each: 
		```bash
		roslaunch controller mover_test.launch
        roslaunch interface backend.launch only_virtual:=false # Run in another terminal
		```

        Else if you just want to run in simulation,  run:
     	```bash
         roslaunch interface backend.launch only_virtual:=true 
		```

        
    2. If you are using the kinect run the following. Make sure this is on the computer with the Big Chonker GPU.
        ```bash
        roslaunch interface vision.launch use_kinect:=true
        ```
        
        Else, to simulate a couple objects, use:
        ```bash
        roslaunch interface vision.launch use_kinect:=false
        ```

    Note: If you run into an issue with catkin not being able to find the authoring package, please run `catkin clean -y` in the `dexterity-interface/` and the `panda-primitives/` directories and then redo Step 4 in the setup. After that, this command should work.


2. Launch a live server for `frontend/index.html`. If you are using VScode, you can do that by selecting that file to open it, and in the lower right of VSCode click "Go Live". This should launch the interface in your browser.
    

## Troubleshooting

```bash
# To view TF frames
rosrun tf2_tools view_frames.py
```



## Resources

* https://github.com/cruise-automation/webviz
* https://github.com/osrf/rvizweb
* https://robotwebtools.github.io/
* https://github.com/Mechazo11/interactive_marker_proxy_noetic
* https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/interactive_marker_tutorials
* http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
* https://robotwebtools.github.io/ros3djs/
* https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html