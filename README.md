# provant-simulator-docker
This repository provides easy-to-use Docker images and scripts for the ProVANT Simulator with graphical (with GPU) integration with the host system. This Docker image is tested on Archlinux, but it should work on Ubuntu and derivatives as well if Docker is properly installed. To install Docker on Ubuntu, follow the official [instructions](https://docs.docker.com/engine/install/ubuntu/).

## Usage
To build the image:
```bash
./build.sh
```

To run the image with graphical support:
```bash
./run.sh
```
This will start the container and open a bash shell inside it. You can then run the ProVANT Simulator from there. Or:
```bash
./run.sh -g
```

The container will have access to your host's GPU.

Lastly, to stop all containers and remove them:
```bash
./clean_docker.sh
```

## Persistence
The run.sh script mounts volumes to allow graphical integration with the host system, also mounts `./shared/catkin_ws` onto `/root/catkin_ws` in the container. This allows you to persist your workspace and any files you create or modify inside the workspace while running the container.

On your host system, run:
```bash
mkdir -p ./shared/catkin_ws/src
```
to create the necessary directories for your catkin workspace, if you haven't done so already.

## Notes
The image install dependencies for the ProVANT Simulator, but does not include the simulator itself. You will need to clone the ProVANT Simulator repository into the `./shared/catkin_ws/src` directory to use it. You can do this by running:
```bash
git clone https://github.com/ProVANT-Project/ProVANT_Simulator.git ./shared/catkin_ws/src/ProVANT_Simulator
```
Then, follow the install procedure in the README.md of the ProVANT Simulator repository to build and run the simulator.

For now, the image runs as root, which is not ideal. Future versions may include a non-root user for better security.

