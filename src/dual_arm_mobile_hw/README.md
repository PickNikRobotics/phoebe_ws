# Phoebe Hardware Guide

## Access

### SSH

`sshpass -p clearpath ssh -Y administrator@cpr-r100-0599.local`

### Teach pendants

The admin password for both URs is `password`.

## Clearpath Base Startup

1. To turn the Phoebe robot on, press the power button on the Clearpath base. You should hear the fans turn on and the
   power button should start blinking blue. After 2-3 minutes, the power button light should stop blinking and show
   solid blue, this means the Clearpath platform systemd services started successfully and were able to establish
   communication with the on-board microcontroller. \
   NOTE: sometimes the microcontroller does not boot properly and the power button light will continue blinking blue
   indefinitely, if the light is still blinking after ~4 minutes, reboot the Clearpath using the power button.
2. Once the power button is solid blue, the e-stop can be released. To do so, press the "START" button on the autec
   wireless e-stop controller.
3. The light around the "E-STOP RESET" button on the Clearpath base should begin blinking red. Press it to release the
   e-stop.
4. At this point the Clearpath base can be teleoperated with the wireless PS4 controller. To move the base with the PS4
   controller, first connect the controller, then hold the L1 trigger and move the joysticks to teleop the Clearpath
   base.

## MoveIt Pro Startup

1. Once the Clearpath base has been started and the e-stop has been released, power on the UR arms by pressing the
   "LEFT UR5" and "RIGHT UR5" buttons on on the platform behind the monitors. You should hear the fans briefly start up
   and then the Polyscope software should start booting on the monitors.
2. Once the URs are booted, release their e-stop by pressing the red button in the bottom left corner of the monitor,
   then press "ON". You may be asked to verify the robot mounting position since they are mounted at angles. Once that
   is complete and the robot is in "idle", press "START". You should hear the brakes release.
3. At this point all of the hardware is ready. SSH into the phoebe using
   `sshpass -p clearpath ssh -Y administrator@cpr-r100-0599.local`.
4. In the ssh shell, build and run MoveIt Pro with `moveit_pro build/run`. \
   NOTE: a `COLCON_IGNORE` may need to be placed in the clearpath_generator_common package to successfully build. \
5. To unlock the base hardware interface, the `/set_active` service needs to be called from the command line. This is an
   extra safety layer to ensure the base of the robot does not move unintentionally. \
   To activate the base run `ros2 service call /set_active std_srvs/srv/SetBool "{data: true}"` \
   To deactivate the base run `ros2 service call /set_active std_srvs/srv/SetBool "{data: false}"` \
   NOTE: there is also a `/reset_odom_offset` service that can be called if the base is moved with the controller while
   Pro is running and you wish to re-zero the position. If you do not re-zero the position after moving the robot with
   the controller, then the robot will be commanded to the previous zero odom position when the clearpath hardware
   interface is re-activated.

## Troubleshooting

### Ewellix lift drivers crash on startup

Sometimes the Ewellix lift drivers crash during startup. If you see the following in the drivers logs you will need to restart Pro:
```bash
drivers-1       | [ros2_control_node-5] [INFO] [1759336498.288919513] [resource_manager]: Successful 'configure' of hardware 'lift_right_hardware'                                                                                                 
drivers-1       | [ros2_control_node-5] [INFO] [1759336498.288948375] [resource_manager]: 'activate' hardware 'lift_right_hardware'                                                                                                                
drivers-1       | [ros2_control_node-5] [INFO] [1759336498.288957636] [EwellixHardwareInterface]: Activating...                                                                                                                                    
drivers-1       | [ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster-6] [INFO] [1759336498.967320003] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...                                                                                                                                                                                                                                   
drivers-1       | [ros2_control_node-5] bool ewellix_driver::EwellixSerial::receive(std::vector<unsigned char>, std::vector<unsigned char>&, std::vector<unsigned char>&): Message:   82    79    2    30    13                                    
drivers-1       | [ros2_control_node-5] bool ewellix_driver::EwellixSerial::receive(std::vector<unsigned char>, std::vector<unsigned char>&, std::vector<unsigned char>&): Response 0:                                                             
drivers-1       | [ros2_control_node-5] [FATAL] [1759336499.303402140] [EwellixHardwareInterface]: Failed to activate EwellixSerial remote control.                                                                                                
drivers-1       | [ros2_control_node-5] [INFO] [1759336499.304298463] [EwellixHardwareInterface]: Error!                                                                                                                                           
drivers-1       | [ros2_control_node-5] [INFO] [1759336499.305000319] [resource_manager]: Failed to 'activate' hardware 'lift_right_hardware'                                                                                                      
drivers-1       | [ros2_control_node-5] terminate called after throwing an instance of 'std::runtime_error'                                                                                                                                        
drivers-1       | [ros2_control_node-5]   what():  Failed to set the initial state of the component : lift_right_hardware to active                                                                                                                
drivers-1       | [ros2_control_node-5] Stack trace (most recent call last):                                                                                                                                                                       
drivers-1       | [ros2_control_node-5] #13   Object "", at 0xffffffffffffffff, in                                                                                                                                                                 
drivers-1       | [ros2_control_node-5] #12   Object "/opt/ros/humble/lib/controller_manager/ros2_control_node", at 0x55d847363ee4, in                                                                                                             
drivers-1       | [ros2_control_node-5] #11   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7f2071db4e3f, in __libc_start_main                                                                                                                
drivers-1       | [ros2_control_node-5] #10   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7f2071db4d8f, in                                                                                                                                  
drivers-1       | [ros2_control_node-5] #9    Object "/opt/ros/humble/lib/controller_manager/ros2_control_node", at 0x55d8473633e3, in                                                                                                             
drivers-1       | [ros2_control_node-5] #8    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0x7f20724cbdeb, in controller_manager::ControllerManager::ControllerManager(std::shared_ptr<rclcpp::Executor>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, rclcpp::NodeOptions const&)                                                                    
drivers-1       | [ros2_control_node-5] #7    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0x7f20724b0972, in                                                                                                                         
drivers-1       | [ros2_control_node-5] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7f20720824d7, in __cxa_throw                                                                                                            
drivers-1       | [ros2_control_node-5] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7f2072082276, in std::terminate()                                                                                                       
drivers-1       | [ros2_control_node-5] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7f207208220b, in                                                                                                                        
drivers-1       | [ros2_control_node-5] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7f2072076b9d, in                                                                                                                        
drivers-1       | [ros2_control_node-5] #2    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7f2071db37f2, in abort                                                                                                                            
drivers-1       | [ros2_control_node-5] #1    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7f2071dcd475, in raise                                                                                                                            
drivers-1       | [ros2_control_node-5] #0    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7f2071e219fc, in pthread_kill 
```

### EwellixHardwareInterface position error

Sometimes the Ewellix lifts can get into a bad state and need to be reset. You will see something like the following in the logs when a reset is required:
```bash
drivers-1       | [ros2_control_node-5] [FATAL] [1759338526.898258256] [EwellixHardwareInterface]: Position between drives too great. Only if synchronized parallel run is parameterized. Motion not started. If motion ins progress the motion is stopped (fast stop). Bit reset on next motion.              
drivers-1       | [ros2_control_node-5] [FATAL] [1759338526.898284140] [EwellixHardwareInterface]: Try to move lift with remote. If not moving, reset lift by power-cycling and holding both UP and DOWN buttons for 5+ seconds.
```
To reset the lifts, hold both the up and down button for 5 seconds until you hear the lift beep. Then move it all the way down until you hear it beep and then all the way up until you hear another beep.

## Other Notes

For the final demo, we will probably just want to disable the lifts. Their motion is not smooth or aligned with the rest of the system and they probably cause more problems than they are worth with all their failure modes.
