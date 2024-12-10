# Flock-Formation Control of Multi-Agent Systems using Imperfect Relative Distance Measurements

This implements the software as used in the following paper:
A. Brandstätter, S. A. Smolka, S. D. Stoller, A. Tiwari and R. Grosu, "Flock-Formation Control of Multi-Agent Systems using Imperfect Relative Distance Measurements," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 12193-12200, doi: [10.1109/ICRA57147.2024.10610147](https://doi.org/10.1109/ICRA57147.2024.10610147).

Please cite this work as follows:

```console
@INPROCEEDINGS{10610147,
  author={Brandstätter, Andreas and Smolka, Scott A. and Stoller, Scott D. and Tiwari, Ashish and Grosu, Radu},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Flock-Formation Control of Multi-Agent Systems using Imperfect Relative Distance Measurements}, 
  year={2024},
  volume={},
  number={},
  pages={12193-12200},
  keywords={Location awareness;Dictionaries;Control systems;Distance measurement;Stability analysis;Robotics and automation;Physics},
  doi={10.1109/ICRA57147.2024.10610147}}
}
```

## Acknowledgment

The software is based on **CrazyS** by Giuseppe Silano et al. Thanks for this awesome work!

For details please refer to the repository of CrazyS: [https://github.com/gsilano/CrazyS](https://github.com/gsilano/CrazyS)

## Quick start
*Everything you need to get it running initially...*

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

## Troubleshooting

* If Gazebo GUI is not showing up, you might check if the X forwarding from inside docker is working. Inside docker run the following command to check if X is working:

    `xeyes`
