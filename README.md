# VoronoiPlanner3D

VoronoiPlanner3D is a ROS 2 package for 3D planning using the Generalized Voronoi algorithm.

This work starts as a translation into C++ of the planning algorithm written in Python and described in:

    “An Efficient Framework for Autonomous UAV Missions in Partially-Unknown GNSS-Denied Environments”,
    Mugnai, Teppati Losé, Herrera-Alarcòn, Baris, Satler, Avizzano,
    Published in Drones 18 July 2023.
    
Such algorithm uses [generalized_voronoi_diagram](https://github.com/ross1573/generalized_voronoi_diagram) by [ross1573](https://github.com/ross1573) and [toppra](https://github.com/hungpham2511/toppra) by [hungpham2511](https://github.com/hungpham2511) and it works as follows:

- [GVD] creates a 2D Voronoi diagram of a planar scene and its corresponding graph;
- [GVD] finds the best path through A* algorithm;
- [TOPPRA] computes the time-optimal path parametrization for robots subject to kinematic and dynamic constraints.

VoronoiPlanner3D focuses on improving the first two points identified with [GVD] by using C++, obtaining a **speed improvement up to 30x**.
Moreover, a simple but effective 3D implementation is obtained by dividing the space into different slices, computing the GVD for each of them and building the 3D graph by stacking the 2D slices and connecting closer nodes on different layers. The 3D implementation is not perfect but, on the other hand, it is extremely fast.

In the end, the computed 3D path is given to TOPPRA's C++ library to obtain the desired parametrization.

The code was written and tested using ROS 2 Humble Hawksbill on Ubuntu 22.04.4 LTS (Jammy Jellyfish). Although it may work natively on other ROS 2 and Ubuntu distributions, it has never been tested so far outside this configuration.


## Installation

### 1. Install third-party libraries

VoronoiPlanner3D needs some libraries in order to compile:
- [toppra](https://github.com/hungpham2511/toppra);
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp);
- [qhull](https://github.com/qhull/qhull).

To build them from source, which is recommended, you can follow the general steps: 

```bash
git clone <repo_url>
cd <repo_name>
mkdir build; cd build
cmake ..
make
sudo make install
```

Pay attention, to build TOPPRA you need to do 

```bash
cd <repo_name>/cpp
```

instead of the second step written above.

### 2. Clone VoronoiPlanner3D and compile

```bash
git clone https://github.com/lorenzo-bianchi/VoronoiPlanner3D
cd VoronoiPlanner3D
colcon build --symlink-install --packages-select voronoi_planner
```

Then, source the local_setup file with

```bash
. install/local_setup.zsh
```

or

```bash
. install/local_setup.bash
```

Depending on the shell you are currently using.

### 3. Launch package

```bash
ros2 launch voronoi_planner voronoi_planner.launch.py
```

After launching, you should see some prints showing the path from starting to ending point, the optimized time computed by TOPPRA, the final position and the error obtained, as final check, by considering a point moving from starting position and subject to optimal velocities.

![Output](images/voronoi_output.png)

In yellow you can see the execution times of code's individual steps.

Moreover, the code publishes a MarkerArray message representing obstacles and optimal trajectory to be visualized in RViz. 

![RVIZ](images/voronoi_rviz.png)

The code is meant to run the algorithm once and to keep publishing the MarkerArray until the user stops the execution by pressing CTRL+C. This is just an example of how the code works, the user will have to implement its own callback that receives the different map layers and uses the functionalities of this library to compute the final trajectory.

## License
This work is licensed under the GNU General Public License v3.0. See the LICENSE file for details.

## Disclaimer

This software has been tested, but it may still contain bugs or issues. Use it at your own risk. The author(s) of this project are not responsible for any damage, loss, or other consequences resulting from the use of this software. We recommend thoroughly testing the software in your environment before using it in production.

If you encounter any bugs or issues, please report them by opening an issue on the [GitHub repository](https://github.com/lorenzo-bianchi/VoronoiPlanner3D).