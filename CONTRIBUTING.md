# Contributing

When contributing to this repository, please first discuss the change based on the guidelines provided by the rovotics github workflow infographic: https://drive.google.com/drive/folders/1jU-FIVXCQNhYr2nQe4ZDVrosyF5O_hYB?usp=sharing

Also consult the phase diagram to better the priority of your project: https://drive.google.com/drive/folders/19IMiH6qa5t3C6XOmDr3fkemhwTR5T3LL?usp=sharing

## Pull Request Process

1. Ensure any install or build dependencies are removed before the end of the layer when doing a 
   build.
2. Update the pull request with details of changes to the interface, this includes new environment 
   variables, exposed ports, useful file locations and container parameters. 
3. Update package specific contributing and versioning documentation
4. You may merge the Pull Request in once you have the sign-off of two other developers, or if you 
   do not have permission to do that, you may request the second reviewer to merge it for you.
   
## Documnetation guidelines
1. Each ROS package and embedded C project (arduino, atmel studio, etc.) should have its own README.md that is based off the provided templates. This documentation should provide and overview of the packagem including setup, architecture (topics, publishers, subscirbers). and meta-information (versioning and contributing). 
2. Individual driver documentation is more relaxed and should be custom tailored to provide a detailed overview of the API and how to ude it.
3. Detailed documenation on the ROS packaged should be provided through rosdoc_lite and doxygen
4. Every file must be well commented

Review how to use rosdoc_lite: http://wiki.ros.org/rosdoc_lite
1. `sudo apt-get install ros-kinetic-rosdoc-lite`
2. Copy the `rosdoc.yaml` template (documenation_templates) file into your package and make the necessary edits
3. Go to the the src directory of your ros_workspace (ex. ~/Desktop/ROV_Test_Bench/ros_workspace/src/)
4. Run `rosdoc_lite <package_name>`
5. Veiw documentation by opening the `index.html` file stored in the `ros_workspace/src/doc/html/package_name` directory with your prefered web browser 

Document code using the javadoc style comments with JAVADOC_AUTOBRIEF=NO: http://www.stack.nl/~dimitri/doxygen/manual/docblocks.html

Good examples of doxygen comments can be found in the rov_control_interface and vector_drive packages for C++ and in the bmp280_driver.py and sht31_drive.py files for python
