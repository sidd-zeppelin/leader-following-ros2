# leader-following-ros2

## Setup Instructions

### Clone the Repository

Clone the repository into your home directory, then navigate into the Docker folder:

```bash
cd ~/leader-following-ros2/docker
```

---

### Build the Docker Image

Build the image using:

```bash
docker build -t ros2-humble -f Dockerfile .
```

---

### Enable GUI Permissions (Required for Gazebo / RViz)

Run:

```bash
xhost +local:root
```

You may need to run this every time before launching the container (unless it has been added to your `.bashrc`).

---

### Create and Run the Container

After building the image, start the container using:

```bash
docker run -it     --name ros2-humble-container     --privileged     --net=host     --ipc=host     --gpus=all     --env DISPLAY=$DISPLAY     --env NVIDIA_VISIBLE_DEVICES=all     --env NVIDIA_DRIVER_CAPABILITIES=all     --env QT_X11_NO_MITSHM=1     --env LD_LIBRARY_PATH=/usr/lib/wsl/lib     --env MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA     --volume /tmp/.X11-unix:/tmp/.X11-unix     --volume /mnt/wslg:/mnt/wslg     --volume /usr/lib/wsl:/usr/lib/wsl     --volume /dev/dxg:/dev/dxg     --volume ~/leader-following-ros2/leader_follower_ws:/root/leader_follower_ws     --device /dev/dxg     --user $(id -u):$(id -g)     ros2-humble
```

This will open an interactive shell inside the container.

---

### Workspace Location

Inside the container, the workspace is located at:

```
/root/leader_follower_ws
```

---

### Exit the Container

To exit from the container:

```bash
exit
```

---

### Restart the Container

To start the container again after exiting:

```bash
docker start -ai ros2-humble-container
```

---

### Open Another Terminal in a Running Container

If the container is already running and you want to open another terminal inside it:

```bash
docker exec -it ros2-humble-container bash
```
