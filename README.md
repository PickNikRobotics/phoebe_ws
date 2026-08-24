1. `cd ~`
2. `git clone git@github.com:PickNikRobotics/phoebe_ws.git`
3. `cd phoebe_ws`
4. `git submodule update --init --recursive`
5. `touch src/dependencies/clearpath_common/clearpath_generator_common/COLCON_IGNORE`
6. `pwd && moveit_pro configure`
7. `moveit_pro build`
8. `moveit_pro run -v -c phoebe_sim`

See the phoebe_hw package for details on running on hardware.

## Licensing

PickNik-authored material is available under the BSD 3-Clause license in
[`LICENSE`](LICENSE). This repository also contains separately licensed
third-party robot assets. See [`COPYING`](COPYING), [`NOTICE`](NOTICE), and
[`LICENSES/`](LICENSES/) for the complete license map and attribution.
