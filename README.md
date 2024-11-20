# swarm simulation using crazys

## Quick start (everything you need to get it running the first time)

* Execute command `id` to get your own uid and gid and set it in file `docker/Dockerfile` at lines 28 and 29. This is needed to correctly map the user and group inside the docker container to your user and group on the host system.

* Build the docker container:

    `cd docker; docker build . --tag crazys`

* Run the docker container (you might need to adjust the path `~/SWARM/crazys` to your correct path, where you checked out the repository)

    `cd ~/SWARM/crazys && docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/crazys crazys`

* Inside docker run the following command to initially build all nessacary ROS packages and source code:

    `crazys_init`

* Then run the simulation with 5 drones (this will start ROS and also the Gazebo GUI):

    `roslaunch rotors_gazebo crazyflie2_dist5.launch gui:=true swarm_params:=mpc1_params2b`

* To exit the simulation use `CTRL+C` and then `CTRL+D` to leave the screen-session.

For more information and examples also refer to `README.md` in folder `docker`.

