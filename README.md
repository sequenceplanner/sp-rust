# Sequence Planner
In many modern automation solutions, traditional off-line and manual programming is being replaced by online algorithms that dynamically perform tasks based on the state of the environment. Complexities of such systems are pushed even further with collaboration among robots and humans, where intelligent machines and learning algorithms are replacing more traditional automation solutions.

Sequence Planner (SP) is a control system written in rust for automation control based on ROS2 that tries to handle the increased complexity of these new systems using formal models and online planning algorithms to coordinate the actions of robots and other devices.

### SP and ROS2
SP connect to ROS2 using the rust bindings: https://github.com/sequenceplanner/r2r. It can also be built using colcon, see the examples project: https://github.com/sequenceplanner/dorna_example.


### Requirements:
1. [ROS2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)
2. [Rust](https://rustup.rs/)
3. [NuXmv](https://nuxmv.fbk.eu)
4. llvm and clang(https://rust-lang.github.io/rust-bindgen/requirements.html#clang)
5. [SP ROS messages](https://github.com/sequenceplanner/sp-ros) Download, colcon build and source before building this repo.

To SP, go to https://github.com/sequenceplanner/dorna_example/tree/master/src/sp_model for an example how to use SP.

### Acknowledgment
The development of SP as a control system has been developed in various research projects:
* Virtcom 2, FFI project funded by Vinnova, Sweden’s innovation agency
* Unficiation, Production 2030 project funded by Vinnova, Sweden’s innovation agency
* Unicorn, FFI project funded by Vinnova, Sweden’s innovation agency

***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
