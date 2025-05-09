# assistive_robotics_thesis

This is the repo for my Senior Honors Thesis as an undergraduate student at University of Wisconsin - Madison for Computer Sciences.

```
xhost +local:

sudo docker build -t thesis-container .
# if changing Dockerfile dependencies, run
# sudo docker build --no-cache -t thesis-container .

sudo docker run -it --gpus all \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  -v /run/user/1000/pulse:/run/user/1000/pulse \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  --device /dev/snd \
  --device /dev/bus/usb \
  --net=host \
  thesis-container

cd src/florence_2_L

python3 main.py
```