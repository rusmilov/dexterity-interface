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
1. Bring in the submodules and/or make sure they are updated:
    ```bash
    git submodule update --init --recursive
    ```

3. Now  build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.

    ```bash
    sudo docker build -t llm-control .
    ```

    If you don't need the Kinect, run:
    ```bash
    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host llm-control
    ```

    Else if you do need the Kinect, run:
    ```bash
    xhost +local:

    sudo docker run -it --rm --gpus all --privileged -e DISPLAY=$DISPLAY -e PULSE_SERVER=unix:/run/user/1000/pulse/native -v /run/user/1000/pulse:/run/user/1000/pulse -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --device /dev/snd --device /dev/bus/usb --net=host llm-control
    ```

### 3.5 Compile panda-primitives-control package (SKIP FOR JUST SIMULATION/INTERFACE)
This step will compile panda-primitives-control package that control Panda Robot in low level, you can skip it if you only need simulation

1. Add necessary environment variables: Replace with your Panda's IP
    ```bash
    export PANDA_IP=192.168.1.3
    ```
2. ### Compile non-ROS package (PandaController)

    If first time, first configure:
    ```bash
    cd backend-ros/src/PandaController/
    mkdir build
    cd build
    cmake ..
    cd ../../../..
    ```

    Anytime, run:
    ```bash
    cd backend-ros/src/PandaController/build
    make install
    cd ../../../..
    ```

3. Setup panda 
    1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
For more information, please refer to [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control) -->

### 4. Compile ros packages

```bash
cd backend-ros
catkin build
source devel/setup.bash
```

## Running
1. Launch the Backend. Make sure you are in the `backend-ros/` directory when running these commands, each in a different terminal.

    1. If you want the program to run on the robot run these each: 
		```bash
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



2. Launch a live server for `frontend/index.html`. If you are using VScode, you can do that by selecting that file to open it, and in the lower right of VSCode click "Go Live". This should launch the interface in your browser.


## Running across multiple computers
On both computers, run:
```bash
export ROS_MASTER_URI=http://<IP_ADDRESS_OF_MAIN_MACHINE>:11311
export ROS_IP=<IP_ADDRESS_OF_CURRENT_MACHINE>
```

For example in our setup, our "main" computer (with roscore) would be the laptop and 
we would run the following there:
```bash
export ROS_MASTER_URI=http://192.168.3.2:11311
export ROS_IP=192.168.3.2
```

And run the following on the secondary desktop computer:
```bash
export ROS_MASTER_URI=http://192.168.3.2:11311
export ROS_IP=192.168.3.3
```


On the "main" computer, run the following:
```bash
roscore
```

Now you are ready to run all the other commands in other terminals across your 2 machines.

## Troubleshooting

```bash
# To view submodule status
git submodule status


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