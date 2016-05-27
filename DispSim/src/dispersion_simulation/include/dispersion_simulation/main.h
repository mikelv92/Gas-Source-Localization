#include <fstream>
#include <stdlib.h>
#include <sys/stat.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <msgs_and_srvs/SensorPosition.h>
#include <msgs_and_srvs/SensorPosition2.h>
#include <math.h>
#include "dispersion_simulation/algorithm.h"
#include "dispersion_simulation/configuration.h"
#include "dispersion_simulation/filament.h"
#include "dispersion_simulation/foamToFilament.h"
#include "dispersion_simulation/Filamentproperties.h"
#include "dispersion_simulation/global.h"
#include "dispersion_simulation/mtwist.h"
#include "dispersion_simulation/utility.h"



#define DEFAULT_SNAPSHOTS					100

#define DEFAULT_SOURCE_POS_X				5
#define DEFAULT_SOURCE_POS_Y				10
#define DEFAULT_SOURCE_POS_Z				0.1


#define	DEFAULT_AREA_X						31
#define	DEFAULT_AREA_Y						16
#define	DEFAULT_AREA_Z						6
#define	DEFAULT_AREA_CELL_SIZE				1

#define DEFAULT_SPREADING_WIDTH             2.5
#define DEFAULT_NUM_MOLECULES               8.3*pow(10,9)
#define	DEFAULT_GAS_TYPE					0
#define	DEFAULT_DELTA_T						1
#define	DEFAULT_WIND_DATA					"/home/victor/windData/single_obstacle/single_obstacle0."
//#define	DEFAULT_WIND_DATA					"/home/victor/ros_catkin/odour_simulation_ws/src/dispersion_simulation/CSV/Velocity0."
#define	DEFAULT_RESULT_LOCATION				"/home/han/gdm_simulator_catkin_ws/src/gas_dispersion_simulation/recordedData/fasterRandom_releaseRate/" // you can also set the beginnings of the file names here.
#define	DEFAULT_FILAMENT_NUMBER				100
#define	DEFAULT_FIXED_FRAME					"/map"
#define DEFAULT_WINDFLOW_RESOLUTION              1 // default map resolution is equal to 1m.

#define DEFAULT_TEMPERATURE                 298 //K
#define DEFAULT_PRESSURE                    1   // atm
#define DEFAULT_UNIT_CHOICE                 0 // 0 for #molecules per cm³ and 1 for ppm



#define	NODE_NAME 							"dispersal_simulation"


double		input_source_pos_x;
double		input_source_pos_y;
double		input_source_pos_z;


double		input_area_cell_size;
int			input_area_size_x;
int 		input_area_size_y;
int 		input_area_size_z;


int			input_snapshots;
int			input_gas_type;

double		input_delta_t;
std::string	input_wind_data;
int			input_filament_number;
std::string	input_fixed_frame;
std::string input_result_location;

double      input_spreading_width;
long long int         input_num_molecules;
long long int numMolecules;

double      input_temperature;
double      input_pressure;

int         input_unit_choice;
double       input_windflow_resolution;

void 		loadNodeParameters(ros::NodeHandle private_nh);
