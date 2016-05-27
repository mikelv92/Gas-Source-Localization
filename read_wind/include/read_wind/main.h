#ifndef MAIN_H_
#define MAIN_H_

#include "ros/ros.h"
#include "read_wind/WindData.h"

extern "C" {
#include <dispersion_simulation/flow_reader.h>
#include <dispersion_simulation/flow.h>
#include <dispersion_simulation/foamToFilament.h>

}


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <sys/stat.h>


#define DEFAULT_SNAPSHOTS					300

#define DEFAULT_SOURCE_POS_X				2
#define DEFAULT_SOURCE_POS_Y				2
#define DEFAULT_SOURCE_POS_Z				3


#define	DEFAULT_AREA_X						11
#define	DEFAULT_AREA_Y						6
#define	DEFAULT_AREA_Z						4
#define	DEFAULT_AREA_CELL_SIZE				0.5

#define	DEFAULT_WIND_DATA					"/home/mikel/catkin_ws/data/windData/small_simple/small_simple0."

#define	NODE_NAME 							"read_wind_server"

double		input_area_cell_size;
int			input_area_size_x;
int 		input_area_size_y;
int 		input_area_size_z;


int			input_snapshots;
int			input_gas_type;

std::string	input_wind_data;
char wind_files[200];
char output1 [256];
char output2 [256];
int n;
sFlow flow;
void loadNodeParameters(ros::NodeHandle private_nh);

#endif
