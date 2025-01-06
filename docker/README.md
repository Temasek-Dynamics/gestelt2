# Docker

# Official ROS docker images
They can be found on the [docker hub registry](https://registry.hub.docker.com/_/ros/). We are currently using `noetic-ros-core-focal`.

# Installing docker
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9B98116C9AA302C7
sudo apt-get update
sudo apt-get install curl

curl -fsSL test.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -aG docker $USER 

# Test the installation
docker run hello-world 
```

# Host

## Building and Pushing images to Docker Hub repository
```bash
docker build --platform linux/arm64 -t gestelt/sfc:latest .

# Tag image
docker tag ORIGINAL_TAG NEW_TAG
docker push gestelt/sfc:latest

# All-in-one build and push (add --no-cache for a clean rebuild)
docker build --platform linux/arm64 -t gestelt/sfc:latest --push .
```

## Running containers
```bash
# To use host USB devices, add "--privileged" flag or "--device=/dev/ttyAML1"
docker run -it --rm --network host --privileged -e "DRONE_ID=0" gestelt/sfc:latest

docker run -it --network host --privileged gestelt/sfc:latest

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
docker commit CONTAINER_NAME gestelt/sfc:latest
# push 
docker push gestelt/sfc:latest
# Inspect the container
docker container inspect CONTAINER_NAME
```

# Radxa

## Commands
```bash
# Pull Images
docker pull gestelt/sfc:latest
docker run -it --rm --network host --privileged -e "DRONE_ID=$DRONE_ID" gestelt/sfc:latest
```

# Repositories
[gestelt/sfc](https://hub.docker.com/r/gestelt/sfc)

# Resources
[Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).

[A Guide to Docker and ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/)