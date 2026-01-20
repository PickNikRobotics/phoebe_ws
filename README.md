1. `cd ~`
2. `git clone git@github.com:PickNikRobotics/phoebe_ws.git`
3. `cd phoebe_ws`
4. `git submodule update --init --recursive`
5. `touch src/dependencies/clearpath_common/clearpath_generator_common/COLCON_IGNORE`
6. `touch src/dependencies/ewellix_lift/ewellix_examples/COLCON_IGNORE`
7. `pwd && moveit_pro configure`
8. `moveit_pro build`
9. `moveit_pro run -v -c dual_arm_mobile_sim`

See [src/dual_arm_mobile_hw/README.md](src/dual_arm_mobile_hw/README.md) for details on running on hardware.
