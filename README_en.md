# Behavior BehaviorFollowUAV

Read in [Polish]

This package creates behavior used by [Aerostack] (framework developed by [Vision4UAV].
This behavior enables following a given drone (based on its position).

### Instalation ###
1. Move the files of this repository into the
    `~/workspace/ros/aerostack_catkin_ws/src/`
   folder so that the catalogue tree looks as follows:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_follow_UAV
    		    -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_follow_uav.launch
    			-src
                    -include
                        -behavior_follow_uav.h
                    -source
                        -behavior_follow_uav.cpp
                        -behavior_follow_uav_main.cpp

2. Compile using catkin_make `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edit `simulated_quadrotor_basic.sh` file - in the startup script paste the following lines at the end:
    
	    `#----------------------------------------------` \
	    `# Behavior FollowUAV                                   ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior FollowUAV" --command "bash -c \"
	    roslaunch behavior_follow_uav behavior_follow_uav.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edit `behavior_catalog.yaml` file located in `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    Go to the `behavior_descriptors` section and paste the following lines:
#### NOTICE! It should be pasted in the `configs/droneX` folder of every drone that is going to utilize this behavior
		
          - behavior: FOLLOW_UAV
            timeout: 30
            incompatible_lists: [motion_behaviors]
            capabilities: [SETPOINT_BASED_FLIGHT_CONTROL, PATH_PLANNING]
            arguments:
              - argument: DRONE_ID
                allowed_values: [0,10]
              - argument: RELATIVE_POSITION
                allowed_values: [-100,100]
                dimensions: 3
              - argument: ANGLE
                allowed_values: [-360,360]
                

##### NOTICE! Make the indentation using the space bar, not the tab key!

### Arguments taken: ###
Behavior takes following arguments:
    
    droneID=x
    
It's the ID number of the drone that our drone of going to follow.
    
    relative_position==[x,y,z]
    
Relative position to the leader drone
    
    angle=x
    
This is the difference between the Yaw of the followed drone and our drone. If this argument is not given, the drone will observe the followed UAV.

Example of a call::
`api.activateBehavior('FOLLOW_UAV', relative_position=[-1.2, 1.2,1], angle = -60, droneID=2)`


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95/behavior_follow_uav/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95/behavior_follow_uav/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV/Aerostack>
   [Vision4UAV]: <https://github.com/Vision4UAV>
