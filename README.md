# Omni Mobile Manipulator

## Introduction
This README provides instructions on how to set up and run the `omni_mobile_manipulator` repository in a ROS workspace.

## Cloning the Repository
To clone the repository to the ROS workspace `~/mobile_manipulator_workspace/src`, use the following command:
```
git clone git@github.com:zhaoyintu/omni_mobile_manipulator.git
```

## Building the Docker Container
Navigate to the cloned repository and build the Docker image:
```
cd ~/mobile_manipulator_workspace/src/omni_mobile_manipulator
./build_docker.sh
```

## Starting the Docker Container
Use Docker Compose to start the Docker container:
```
./start_docker.sh
```

## Enable xservice forward
In another terminal
```
xhost +
```

## Accessing the Docker Container
To enter the Docker container, execute:
```
docker exec -it mobile_manipulator bash
```

## Building the Code
Inside the Docker container, build the code with the following commands:
```
cd ~/mobile_manipulator_workspace
catkin build
```

## Running the Code
To run the `omni_mobile_manipulator`, use the launch command:
```
source ~/mobile_manipulator_workspace/devel/setup.bash
roslaunch omni_mobile_manipulator omni_mobile_manipulator.launch
```