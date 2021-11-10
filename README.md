# :sauropod: nanosaur_perception

🐋 🖼️ Perception docker for nanosaur. Works with Isaac ROS

[![Docker Builder CI](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml/badge.svg)](https://github.com/rnanosaur/nanosaur_perception/actions/workflows/docker-build.yml)

nanosaur is a little tracked robot ROS2 enabled, made for an NVIDIA Jetson

Meet nanosaur:
* 🦕 Website: [nanosaur.ai](https://nanosaur.ai)
* 🦄 Do you need an help? [Discord](https://discord.gg/YvxjxEFPkb)
* 🧰 For technical details follow [wiki](https://github.com/rnanosaur/nanosaur/wiki)
* 🐳 nanosaur [Docker hub](https://hub.docker.com/u/nanosaur)
* ⁉️ Something wrong? Open an [issue](https://github.com/rnanosaur/nanosaur/issues)

# Usage

Docker hub [nanosar Perception](https://hub.docker.com/repository/docker/nanosaur/nanosaur_perception)

```
docker pull nanosaur/perception:latest
```

# Develop

## Build on Jetson device

Run this script to setup the enviroment

```
. nanosaur_perception/scripts/docker_builder.sh
```

## Run in test mode

Run docker passing test repository 

```
docker run -it --rm --network host -v ${HOME}/nanosaur_perception:/opt/ros_ws/src/nanosaur_perception -v /tmp/argus_socket:/tmp/argus_socket nanosaur/perception:main bash
```