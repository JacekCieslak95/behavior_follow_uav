# Behavior BehaviorFollowUAV

Read in [Polish]
This package creates behavior used by [Aerostack] (framework developed by [Vision4UAV].
This behavior allows one UAV to follow the other (based on leader's positon)

### Instalation ###
1. Files from this repository should be located in directory:
    `~/workspace/ros/aerostack_catkin_ws/src/`
    Desired directory tree:
    
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
3. Edit file `simulated_quadrotor_basic.sh` - Add following lines in the end of bash file containing starting script.
    
	    `#----------------------------------------------` \
	    `# Behavior FollowUAV                                   ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior FollowUAV" --command "bash -c \"
	    roslaunch behavior_follow_uav behavior_follow_uav.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edit `behavior_catalog.yaml` file. It's located in `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    In section `behavior_descriptors` add following lines:
#### Important!  `behavior_catalog.yaml` should be edited in config directory of every drone, which will use this behavior.
		
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
                
		    
				
##### Important! Intentadion should be done with space, not with tabulators.

### Arguments taken: ###
Behavior takes following arguments:
    
    droneID=x
    
It's the ID number of followed UAV
    
    relative_position==[x,y,z]
    
Relative position to the leader drone
    
    angle=x
    
Difference between Yaw angle of followed UAV and Yaw of drone using this behavior. If this argument is not given - UAV is "looking" at the leader

Example:
`api.activateBehavior('FOLLOW_UAV', relative_position=[-1.2, 1.2,1], angle = -60, droneID=2)`


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95/behavior_follow_uav/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95/behavior_follow_uav/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV>
   [Vision4UAV]: <https://github.com/Vision4UAV/Aerostack>
