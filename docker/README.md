# Docker Guide

# Instructions for building images on Host

## Building and Pushing images to Docker Hub repository
```bash
# Tag image
docker tag ORIGINAL_TAG NEW_TAG
docker push gestelt/mavoro:latest

# All-in-one build and push (add --no-cache for a clean rebuild)
docker build --platform linux/arm64 -t gestelt/mavoro_arm64:latest --push .
```

docker run -it --rm --privileged --network host  gestelt/px4_sitl_gz

ros2 launch gestelt_bringup mavros_sitl.py

ros2 launch gestelt_bringup execute_mission_single.py

ros2 launch gestelt_bringup gcs.py

## Running containers
```bash
# To use host USB devices, add "--privileged" flag or "--device=/dev/ttyAML1"
# Add --rm to remove the container after exiting
# -e is to specify environment variables
docker run -it --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro:latest
docker run -it  --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro_arm64:latest

# Find name of new machine 
docker ps -l
# List all docker containers
docker ps -a
# Start stopped container
docker start <container_id>
# Start additional bash sessions in same container
docker exec -it <container_id> bash
# Stop all containers
docker stop $(docker ps -a -q)
# Remove all containers
docker rm $(docker ps -a -q)
```

## Make changes to containers and save them as new images
```bash
# List all docker containers
docker ps -a
# Commit changes
docker commit CONTAINER_NAME gestelt/mavoro:latest
# push 
docker push gestelt/mavoro:latest
# Inspect the container
docker container inspect CONTAINER_NAME
```

# Instructions for Onboard computer

## Commands
```bash
# Pull Images
docker pull gestelt/mavoro:latest
docker run -it --rm --network host --privileged -e "DRONE_ID=$DRONE_ID" gestelt/mavoro:latest
```

# Installing docker
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9B98116C9AA302C7
sudo apt-get update
sudo apt-get install curl

curl -fsSL test.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -aG docker $USER 

# Test the installation
docker run hello-world 

# For building arm64 images, QEMU is required
docker run --privileged --rm tonistiigi/binfmt --install all
```

# Store docker images on another volume
1. Stop docker
```bash
sudo systemctl stop docker
sudo systemctl stop docker.*
sudo systemctl stop containerd

sudo systemctl status docker.service
sudo systemctl status docker.socket

# Create directory
sudo mkdir -p /media/john/gestelt/docker

# Copy files over
sudo rsync -avxP /var/lib/docker/ /media/john/gestelt/docker
```

## Method A
2. Create symbolic link 
```bash
sudo ln -s /media/visbot/gestelt/docker /var/lib/docker
```

## Method B
2. Update the daemon
```bash
sudo vim /etc/docker/daemon.json
# Add the following information
{
  "data-root": "/media/john/gestelt/docker"
}
```
3. Start docker services
```bash
sudo systemctl start docker
# Validate new docker root location
docker info -f '{{ .DockerRootDir}}'
```

[reference](https://www.ibm.com/docs/en/z-logdata-analytics/5.1.0?topic=software-relocating-docker-root-directory)

## Method C

2. As root, update `/lib/systemd/system/docker.service` to include `--data-root /path/to/new/location` parameter in the line starts with `ExecStart=`. 
```bash
# To configure the parameter
sudo vim /lib/systemd/system/docker.service

# Change the line from 
ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock
# to
ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock --data-root /media/john/gestelt/
```

3. Copy content over from `/var/lib/docker` to new path
```bash
sudo rsync -avxP /var/lib/docker/ /media/john/gestelt/docker
```

References:
1. [move-docker-images-and-volumes](https://wiki.casaos.io/en/guides/move-docker-images-and-volumes-to-a-diffferent-storage#:~:text=There%20are%20few%20options%20to,systemd%20with%20the%20new%20path.)


# Additional commands
```bash
# To run with GUI
xhost +
docker run -it --privileged \
            --env=LOCAL_USER_ID="$(id -u)" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
            -e DISPLAY=:0 \
            --network host \
            gestelt/mavoro:latest bash
            # --rm \
            # -p 14560:14570/udp \
```

# Repositories

- [gestelt/mavoro](https://hub.docker.com/repository/docker/gestelt/mavoro/general)
- [gestelt/mavoro_arm64](https://hub.docker.com/repository/docker/gestelt/mavoro_arm64/general)

- [ROS docker images](https://registry.hub.docker.com/_/ros/)

- [ARM64 ROS images](https://hub.docker.com/r/arm64v8/ros)

# Resources

- [Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).
- [A Guide to Docker and ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/)


