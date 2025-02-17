# Quadruped manipulator framework

This repository provides all necessary resources to replicate a custom 3-DoF robotic arm designed for legged robots. The arm is built with a focus on being lightweight, easy to maintain, and capable of handling payloads without compromising functionality.

The framework is open-source and includes a script-like language for sending high-level interpolation commands, similar to commercial modules. The entire implementation is written in C++ and has been tested in ROS Melodic and Noetic.

## Packages/Nodes

 - **quadruped_igus_bringup** - launches the complete system, including both simulation and hardware setups;
 - **quadruped_igus_description** - contains everything related to the URDF, including a launch file to display the robot model in RViz with the `joint_state_publisher` GUI for workspace visualization.  
    - Robot model obtained from: [Unitree Robotics GitHub](https://github.com/unitreerobotics/unitree_ros)  
    - The arm model was created using Fusion 360 and then converted to URDF with [*fusion2urdf*](https://github.com/syuntoku14/fusion2urdf).
 - **aliengo_controller** - quadruped controller adepted from: [A1 Simulation Python Repository](https://github.com/lnotspotl/a1_sim_py)
 - **igus_arm_driver** - robotic arm controller implementing an IK-based motion planner with two movement interpolation types: `MOVJ` (joint space) and `MOVL` (linear). This package also includes the firmware for communication with IGUS Rebel joints.
 - **igus_rviz_panel** - custom RViz panel to facilitate sending arm movement instructions directly from the visualization interface

## Prequesites

* *trac_ik_lib* - `sudo apt install ros-noetic-trac-ik-lib`

  If you encounter the following error while compiling:

  ```bash
  /opt/ros/noetic/include/trac_ik/nlopt_ik.hpp:35:10: fatal error: nlopt.hpp: No such file or directory
    35 | #include <nlopt.hpp>
  ```

  Install the missing dependency with: `sudo apt install libnlopt-cxx-dev`

* *effort_controllers* (for simulation use only) - `sudo apt install ros-noetic-effort-controllers`

## Usage

### Simulation

 1. Run the environment:

 `roslaunch quadruped_igus_bringup run_simulation.launch`

 2. Unpause Gazebo to start the system.
 
### Hardware

 1. Change the specific information about the arm in `src/igus_arm_driver/config/arm_setup.yaml` file.
 2. Run the environment:

 `roslaunch quadruped_igus_bringup run_robot.launch`
 
 3. System is ready when the terminal says "All joints disabled".

### Send arm instructions

There are two ways of sending instructions to the arm:
- **RViz Panel**  
  Simply fill in the fields and press `Go`.

  ![rviz_panel](images/panel_rviz.png)

- **Publish a goal in `igus_arm_driver_skill/goal` target space**  
  Send one movement or a sequence of movements using instructions of the following type:
  
  moveT(p={a,b,c},v=s,a=$\ddot{\theta}_{max}$)

  Here's what each part of the command means:

  - `T`: Interpolation type. This defines how the movement should be performed. The available types are:
    - `j`: Joint space interpolation (goal defined in 3D space)
    - `jj`: Joint space interpolation (goal defined in joint space)
    - `l`: linear interpolation (goal defined in 3D space)
    - `lj`: linear interpolation (goal defined in joint space)
  - `(a,b,c)`: The goal for the movement.
  - `s`: The speed for the movement.
    - For joint goal movements (`jj` or `lj`), s is the maximum joint speed.
    - For 3D goal movements (`j` or `l`), s is the tool speed.
  - $\ddot{\theta}_{max}$: The maximum joint acceleration for the movement.

    A program ends when `;` is found, each instruction must be separated by a comma.

  **Example:** 4 sequenced joint-space movements:

  ```
  movejj(p={0.79, 1.87, 2.04},v=0.6,a=0.7,b=0),movejj(p={2.53,2.31, 1.43},v=0.6,a=0.7,b=0),movejj(p={0.55, 2.31, 1.43},v=0.6,a=0.7,b=0);
  ```

### Limits

The kinematic limits for IK are defined in `src/igus_arm_driver/src/igus_arm_driver/trac_kinematics.cpp` in order to keep the solutions inside the desired workspace. 

 * Maximum TCP velocity: `0.2 `m/s
 * Maximum joint velocity (`w_max`): `0.6` rad/s
 * Maximum joint acceleration (`a_max`): `0.7` rad/s²

All these values can be modified in `src/igus_arm_driver/config/arm_setup.yaml`. Additional configurable parameters include:

* `w_min`: The deadzone limit for joint velocity.
* `error_threshold_joints`: The acceptable stopping error in joint space.
* `error_threshold`: The acceptable task space error, measured in Euclidean distance.

These parameters influence motion control accuracy and can be adjusted based on system requirements.

<!-- ## License

Distributed under the [MIT License](https://choosealicense.com/licenses/mit/).
See [`LICENSE`](LICENSE) for more information. -->

## Contacts

If you have any questions or you want to know more about the work developed by
us, please contact one of the contributors of this project:

- Maria S. Lopes([email:feup](mailto:mslopes@efe.up.pt),
  [email:inesctec](mailto:maria.s.lopes@inesctec.pt),
  [orcid](https://orcid.org/0009-0001-7216-2469))
  _(corresponding author)_
- Alexandre Melo ([email](mailto:alexandre.mello@inesctec.pt),
  [orcid](https://orcid.org/0009-0005-2037-7678))
- João Pedro C. de Souza ([email](mailto:joao.p.souza@fe.up.pt),
  [orcid](https://orcid.org/0000-0003-1518-4984))
- Pedro Costa ([email](mailto:pedrogc@fe.up.pt),
  [orcid](https://orcid.org/0000-0002-0435-8419))
- Manuel F. Silva ([email](mailto:mss@isep.ipp.pt),
  [orcid](https://orcid.org/0000-0002-0593-2865))

## References



## Acknowledgements

- [CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://www.inesctec.pt/en/centres/criis)
  from
  [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/WEB_PAGE.INICIAL)
- [ISEP - School of Engineering of the Porto Polytechnic](https://www.isep.ipp.pt/)

## Funding

This work is financed by National Funds through the Portuguese funding agency, FCT - Fundação para a Ciência e a Tecnologia, within project LA/P/0063/2020. DOI [10.54499/LA/P/0063/2020](https://doi.org/10.54499/LA/P/0063/2020)