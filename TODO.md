Add a section in the documentation that explains that in order to add a library (like pyvesc or sailboat_gym) that you may want to take out the __init__.py file. If __init__.py is there, colcon will pick up on it and then put it in the install folder, and if it is not there, then colcon will not touch it.

Also explain that in order to have a package like that, you need to add a section to the docker_dev_environment_setup.sh file. In the future, we should probably figure out a way to do it without first going through this file


add actual rudder and sail angles to the telemetry and not just the desired rudder and sail angles


Make the telemetry data usage better by compressing data


motorboat simulation


make it so that the telemetry server node knows what the default parameters are 

EXTREMELY HIGH PRIORITY: figure out how to handle the default parameters on the telemetry server node because right now, it only uses the sailboat default parameters