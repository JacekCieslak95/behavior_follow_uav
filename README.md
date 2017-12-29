# Behavior BehaviorFollowUAV
Paczka tworzy behavior używany przez Aerostack (oprogramowanie grupy Vision4UAV: https://github.com/Vision4UAV/Aerostack)
Zachowanie umożliwiające podążanie za zadanym dronem (opierając się na jego pozycji).
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
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

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
	    `# Behavior FollowUAV                                   ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior FollowUAV" --command "bash -c \"
	    roslaunch behavior_follow_uav behavior_follow_uav.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
#### UWAGA! Należy to wkleić do folderu `configs/droneX` każdego drona, którego chcemy uruchamiać z danym zachowaniem.
#### Np. Używając tego w dronach 1 i 2 poniższy fragment należy dokleić do `behavior_catalog.yaml` w folderach `configs/drone1` oraz `configs/drone2`
	    
		
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
		    
				
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    droneID=x
    
Jest to numer drona, za którym dron będzie podążał
    
    relative_position==[x,y,z]
    
Jest to kąt pozycja wzgędem danego drona