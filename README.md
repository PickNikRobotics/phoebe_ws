1. `cd ~`
2. `git clone git@github.com:PickNikRobotics/phoebe_ws.git`
3. `cd phoebe_ws`
4. `git submodule update --init --recursive`
5. `touch src/dependencies/clearpath_common/clearpath_generator_common/COLCON_IGNORE`
6. `touch src/dependencies/ewellix_lift/ewellix_examples/COLCON_IGNORE`
7. `pwd && moveit_pro configure`
8. `moveit_pro build`
9. `moveit_pro run -v -c phoebe_sim`

See the [phoebe_hw](https://github.com/PickNikRobotics/phoebe_ws) package for details on running on hardware.

## Licensing

PickNik-authored material is available under the BSD 3-Clause license in
[`LICENSE`](LICENSE). This repository also contains separately licensed
third-party models and robot assets. See [`COPYING`](COPYING), [`NOTICE`](NOTICE),
and [`LICENSES/`](LICENSES/) for the complete license map and attribution.
