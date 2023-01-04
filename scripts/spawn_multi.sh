#!/bin/bash
# Script to spawn multiple drones in (Ignition) Gazebo
# Assumes that firmware has already been built for (Ignition) Gazebo using: make px4_sitl gz_x500

# Author: Harvey Merton
# Date: 01/04/23


# SETUP
# Directories
FIRMWARE_DIR="~/repos/PX4-Autopilot"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Constants
PI=3.141592654

# Parameters
PX4_SYS_AUTOSTART=4001
PX4_GZ_MODEL=x500
PX4_GZ_MODEL_POSE_FIRST="0,0"

NUM_DRONES=3
SPAWN_CENTER_X=0
SPAWN_CENTER_Y=0
SPAWN_R=0.5

SPAWN_PTS_X=()
SPAWN_PTS_Y=()


# FUNCTIONS
# Kills processes
function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

# Generates X and Y co-ordinates to spawn NUM_DRONES equally spaced in a circle around the SPAWN_CENTER 
function generate_spawn_points() {
	
	# Use polar co-ordinate system to generate (x,y) points in circumscribing circle 
	for (( i=0;i<$NUM_DRONES;i++ )); do
		SPAWN_PTS_X+=($(echo "$SPAWN_R*c(2*$PI*$i/$NUM_DRONES)+$SPAWN_CENTER_X"| bc -l)) 
		SPAWN_PTS_Y+=($(echo "$SPAWN_R*s(2*$PI*$i/$NUM_DRONES)+$SPAWN_CENTER_Y"| bc -l))
		#echo "i: $i"
		#echo ${SPAWN_PTS_X[@]}
		#echo ${SPAWN_PTS_X[$i]}
	done	
}

# Spawns drones using defined spawn points
function spawn_drones() {
	# Spawn first drone
	gnome-terminal --tab -- bash -c "PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_POSE="${SPAWN_PTS_X[0]},${SPAWN_PTS_Y[0]}" PX4_GZ_MODEL=$PX4_GZ_MODEL ~/repos/PX4-Autopilot/build/px4_sitl_default/bin/px4"
	sleep 5

	# Spawn remaining drones
	for (( i=1;i<$NUM_DRONES;i++ )); do
		#echo "spawn drone: $i"
		#gnome-terminal --tab
		#gnome-terminal --tab -- bash -c "echo spawn drone: $i; exec bash"
		gnome-terminal --tab -- bash -c "PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_POSE="${SPAWN_PTS_X[$i]},${SPAWN_PTS_Y[$i]}" PX4_GZ_MODEL=$PX4_GZ_MODEL ~/repos/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $i"
	done
}


# RUN COMMANDS
cleanup
generate_spawn_points
spawn_drones
#cleanup

#"$@"
#exec "$SHELL"