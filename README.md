1. cd ~
2. git clone git@github.com:PickNikRobotics/dual_arm_mobile_ws.git
3. cd dual_arm_mobile_ws
4. git submodule update --init --recursive
5. touch src/dependencies/clearpath_common/clearpath_generator_common/COLCON_IGNORE
6. pwd && moveit_pro configure
7. moveit_pro build
8. moveit_pro run -c dual_arm_simdual_arm_mobile_sim

See [src/dual_arm_mobile_hw/README.md](src/dual_arm_mobile_hw/README.md) for details on running on hardware.