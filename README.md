# leader-following-ros2

First clone the repo into home, then cd into the Docker folder

and build the image using:

docker build -t ros2-humble -f Dockerfile .

Then run this command for gui perms (you might have to run this everytime you launch the container, not sure i have it as a command in bashrc)

xhost +local:root

Once you build the image run this command to build the container:

docker run -it     --name ros2-humble-container     --privileged     --net=host     --ipc=host     --gpus=all     --env DISPLAY=$DISPLAY     --env NVIDIA_VISIBLE_DEVICES=all     --env NVIDIA_DRIVER_CAPABILITIES=all     --env QT_X11_NO_MITSHM=1     --env LD_LIBRARY_PATH=/usr/lib/wsl/lib     --env MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA     --volume /tmp/.X11-unix:/tmp/.X11-unix     --volume /mnt/wslg:/mnt/wslg     --volume /usr/lib/wsl:/usr/lib/wsl     --volume /dev/dxg:/dev/dxg     --volume ~/leader-following-ros2/leader_follower_ws:/root/leader_follower_ws     --device /dev/dxg     --user root     ros2-humble

This will open into the container
The workspace is located in root

If you want to exit from the container run

exit

If you want to start the container run

docker start -ai ros2-humble-container

if you want to open another terminal into an already running container

docker exec -it ros2-humble-container bash
