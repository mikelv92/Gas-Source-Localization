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


#define DEFAULT_SNAPSHOTS					1000

#define DEFAULT_SOURCE_POS_X				5
#define DEFAULT_SOURCE_POS_Y				10
#define DEFAULT_SOURCE_POS_Z				0.1


#define	DEFAULT_AREA_X						61
#define	DEFAULT_AREA_Y						21
#define	DEFAULT_AREA_Z						6
#define	DEFAULT_AREA_CELL_SIZE				0.5

#define	DEFAULT_WIND_DATA					"/home/mikel/catkin_ws/data/windData/original/t600s_030."

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
