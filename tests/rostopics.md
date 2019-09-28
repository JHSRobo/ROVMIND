# ROS Topic List and Information

This document provides information on each current ROS Topic, with the purpose and message type. More information can be found on each topic inside their respective package.

* `/rov/camera_select`: Current selected camera from the copilot page
  * std_msgs/Float64
* `/rov/electromagnet_control`: Current selected camera from the copilot page
  * std_msgs/Bool
* `/rov/sensitivity`: Sensitivity settings for ROV drive control
  * rov_control_interface/rov_sensitivity
* `/rov/inverstion`: Inversion status depending on camera
  * std_msgs/UInt8
* `/rov/thrusters_enabled`: Whether or not thrusters are enabled
  * std_msgs/Bool

* `/rov/cmd_vel`: Command Velocity vectors
  * geometry_msgs/Twist
* `rov/cmd_horizontal_vdrive`: Thruster Settings for Horizontals
  * vector_drive/thrusterPercents
* `/rov/cmd_vertical_vdrive`: Thruster Settings for Verticals
  * vector_drive/thrusterPercents

* `/rov/humidity`: Outputs internal humidity as a relative percentage
* `/rov/pressure`: Outputs internal pressure as millibars
* `/rov/temperature`: Outputs interal temperature as C
* `/rov/ms5837`: Outputs external pressure, temperature, and depth
  * msg5837/ms5837_data

* `tcu/tcu_data`: Data from the TCU Arduino DUE
  * tcu_board_msgs/tcu_board_data
* `tcu/leds`: LED status for TCU
  * std_msgs/ColorRGBA
* `tcu/main_relay`
  * std_msgs/Bool
* `tcu/main_solenoid`
  * std_msgs/Bool


* `/raspicam_node/compressed`: Compressed JPG image from RPI cameras
  * sensor_msgs/CompressedImage


All default intel realsense topics are shown below:
* These can be name-spaced as desired

/camera/RGB_Camera/parameter_descriptions
/camera/RGB_Camera/parameter_updates
/camera/Stereo_Module/parameter_descriptions
/camera/Stereo_Module/parameter_updates
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/compressedDepth/parameter_descriptions
/camera/color/image_raw/compressedDepth/parameter_updates
/camera/color/image_raw/theora
/camera/color/image_raw/theora/parameter_descriptions
/camera/color/image_raw/theora/parameter_updates
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_infra1
/camera/extrinsics/depth_to_infra2
/camera/infra1/camera_info
/camera/infra1/image_rect_raw
/camera/infra1/image_rect_raw/compressed
/camera/infra1/image_rect_raw/compressed/parameter_descriptions
/camera/infra1/image_rect_raw/compressed/parameter_updates
/camera/infra1/image_rect_raw/compressedDepth
/camera/infra1/image_rect_raw/compressedDepth/parameter_descriptions
/camera/infra1/image_rect_raw/compressedDepth/parameter_updates
/camera/infra1/image_rect_raw/theora
/camera/infra1/image_rect_raw/theora/parameter_descriptions
/camera/infra1/image_rect_raw/theora/parameter_updates
/camera/infra2/camera_info
/camera/infra2/image_rect_raw
/camera/infra2/image_rect_raw/compressed
/camera/infra2/image_rect_raw/compressed/parameter_descriptions
/camera/infra2/image_rect_raw/compressed/parameter_updates
/camera/infra2/image_rect_raw/compressedDepth
/camera/infra2/image_rect_raw/compressedDepth/parameter_descriptions
/camera/infra2/image_rect_raw/compressedDepth/parameter_updates
/camera/infra2/image_rect_raw/theora
/camera/infra2/image_rect_raw/theora/parameter_descriptions
/camera/infra2/image_rect_raw/theora/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/pointcloud/parameter_descriptions
/camera/pointcloud/parameter_updates
