/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2015, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors:
*********************************************************************/ 



//========================================================================================
//	Gas concentration visualization
//	gas_concentration_visualization.cpp
//	description: implements the KernelDM+V gas distribution mapping algorithm
//		
//		
//		
//	topics published: visualization_map

//	services:	/gas_concentration_visualization/get_loggers  /gas_concentration_visualization/set_logger_level

//				
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//	Revision log:
//	version: 1.0	2015/11/04
//========================================================================================

#include "gas_concentration_visualization/gas_concentration_visualization.h"
#include "msgs_and_srvs/SensorPosition2.h"
#include "regression/GetSnapshot.h"
#include "regression/SetSnapshot.h"
//#include "gas_concentration_visualization/SensorPositionViz.h"
#include "ros/ros.h"

sCellConcentration cellConcentration;

std::vector<float> vec_px;
std::vector<float> vec_py;
std::vector<float> vec_pz;
std::vector<float> vec_concentration;


int ReadSingleLogFile(int index, string input_result_location){

	float px, py, pz;
	double hitRateValue, flow_u, flow_v, flow_w;


	string filename;
	ostringstream ss;
	filename=input_result_location;
	//filename="/home/han/gdm_simulator_catkin_ws/src/simulated_tdlas/recordedData/results/gasType1_simulation"; // here the filename should be the absolute values of the folder that store gas distribution simulation results.

	ss.str(std::string());
	ss<<filename<<index;
	std::ifstream fin(ss.str().c_str());
	if (!fin)
	{
		ROS_INFO("Can not open the file %s", ss.str().c_str());
		exit(0);
	}

	ROS_INFO("READING File no. %d", index);
	char firstline[50];
	fin.getline(firstline, 49);
	//ROS_INFO("%s: first line.", firstline);
	while (fin >> px >> py >> pz >> hitRateValue >> flow_u >> flow_v >> flow_w){

		vec_px.push_back(px);
		vec_py.push_back(py);
		vec_pz.push_back(pz);
		vec_concentration.push_back(hitRateValue);

		//ROS_INFO("Got x : %d", x);
		//gasInfo.push_back(gasInformation(px, py, pz, hitRateValue, flow_u, flow_v, flow_w));
	}
	//system("pause");
	return 0;
}


//---------------service response----------------------//
bool pos(msgs_and_srvs::SensorPosition2::Request  &req,
		msgs_and_srvs::SensorPosition2::Response &res )
{
	int size = req.x.size();
	cout << "Inside service " << snapshot << endl;
	for(int i =0; i < size; i++){

		//concentration=concentration at(x,y,z):
		res.odor_r[i]=concentrationRecorded(req.x[i], req.y[i], req.z[i], cell_size, input_temperature, input_pressure, input_unit_choice);
	//res.odor_r=concentrationRecorded(req.x, req.y, req.z, cell_size, input_temperature, input_pressure, input_unit_choice);
		//cout << req.x[i] << " " << req.y[i]<< " " << req.z[i]<<" " << res.odor_r[i] << endl;;
		//res.odor_r1=concentration(req.x_r1, req.y_r1, req.z_r1);
		//res.odor_r2=concentration(req.x_r2, req.y_r2, req.z_r2);
	}


	return true;
}

bool getSnapshotCallback(regression::GetSnapshot::Request  &req,
		regression::GetSnapshot::Response &res )
{
	res.snapshot = snapshot;

	return true;
}

bool setSnapshotCallback(regression::SetSnapshot::Request  &req,
		regression::SetSnapshot::Response &res )
{
	snapshot = req.snapshot;
	return true;
}

/*bool pos(gas_concentration_visualization::SensorPositionViz::Request  &req,
		gas_concentration_visualization::SensorPositionViz::Response &res )
{
	//int size = req.x.size();
	cout << "Inside service " << endl;
	//for(int i =0; i < size; i++){

	//concentration=concentration at(x,y,z):
	//res.odor_r[i]=concentrationRecorded(req.x[i], req.y[i], req.z[i], cell_size, input_temperature, input_pressure, input_unit_choice);
	res.odor_r=concentrationRecorded(req.x, req.y, req.z, cell_size, input_temperature, input_pressure, input_unit_choice);

	//cout << req.x[i] << " " << req.y[i]<< " " << req.z[i]<<" " << res.odor_r[i] << endl;;
	//res.odor_r1=concentration(req.x_r1, req.y_r1, req.z_r1);
	//res.odor_r2=concentration(req.x_r2, req.y_r2, req.z_r2);
	//}

	return true;
}
*/


double concentrationRecorded(float x_in, float y_in, float z_in, float cell_size, double temperature, double pressure, int if_ppm) {
	float scale=1; //unit: 10^6 Molecules/cm^3

	// if one wants to calculate ppm, the conditions of temperature and pressure should be given. 1.66*pow(10,-18) is the number of moles of 1000,000 molecules of air using Avogadro’s numberand 0.08205746 is ideal gas constant.
	if(if_ppm!=0){
		double convert2ppm_scale;
		//condition parameters
		double kelvinTemperature=temperature; // unit K.
		double pressure_atm = pressure;       // unit atm.
		convert2ppm_scale = 1.66*pow(10,-18)*0.08205746*kelvinTemperature/pressure_atm*pow(10,-3);
		scale = convert2ppm_scale;
	}


	int xx,yy,zz;

	xx=(int)floor(x_in + cell_size/2);
	yy=(int)floor(y_in + cell_size/2);
	zz=(int)floor(z_in + cell_size/2);

	double conct;

	conct = cellConcentration.concentraion_value[xx][yy][zz]*scale;//
	//printf(" %f \n", conct);
	return conct;
	}


double*** arr3dAlloc(const int ind1, const int ind2, const int ind3)
{
  int i;
  int j;
  double*** array = (double***) malloc( (ind1 * sizeof(double*)) + (ind1*ind2 * sizeof(double**)) + (ind1*ind2*ind3 * sizeof(double)) );
  for(i = 0; i < ind1; ++i) {
    array[i] = (double**)(array + ind1) + i * ind2;
    for(j = 0; j < ind2; ++j) {
      array[i][j] = (double*)(array + ind1 + ind1*ind2) + i*ind2*ind3 + j*ind3;
    }
  }
  return array;
}


//==========================================================================================
//
//					MAIN
//
//==========================================================================================
int main(int argc, char **argv)
{

	ROS_INFO("\n=================================================================");
	ROS_INFO("\n=	The Visualization of Gas Concentration , Ver %d",NODE_VERSION);
	ROS_INFO("\n=================================================================\n");

	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle param_n;
	loadNodeParameters(param_n);
	//ros::NodeHandle param_n("~");

	ROS_INFO("parameters loaded");

	ros::Rate loop_rate(10);

	ros::ServiceServer getSnapshotService = param_n.advertiseService("getSnapshot", getSnapshotCallback);
	ros::ServiceServer setSnapshotService = param_n.advertiseService("setSnapshot", setSnapshotCallback);

	//ros::ServiceServer service = param_n.advertiseService("gas_concentration_value", pos); // Here the service for concentration information is created and advertised over ROS.
	ros::ServiceServer service = param_n.advertiseService("/odor_value", pos); // Here the service for concentration information is created and advertised over ROS.


	//----------------------------------------------------------------------------
	// Parameter initialization

/*
	param_n.param<std::string>("frame_id", frame_id, std::string(DEFAULT_FRAME_ID));

	param_n.param<double>("map_min_x", map_min_x, DEFAULT_MAP_MIN_X);
	param_n.param<double>("map_max_x", map_max_x, DEFAULT_MAP_MAX_X);
	param_n.param<double>("map_max_y", map_max_y, DEFAULT_MAP_MAX_Y);
	param_n.param<double>("map_min_y", map_min_y, DEFAULT_MAP_MIN_Y);
	param_n.param<double>("map_max_z", map_max_z, DEFAULT_MAP_MAX_Z);
	param_n.param<double>("map_min_z", map_min_z, DEFAULT_MAP_MIN_Z);
	param_n.param<double>("cell_size", cell_size, DEFAULT_CELL_SIZE);


	param_n.param<int>("publish_hz", publish_hz, DEFAULT_PUBLISH_HZ);
	param_n.param<std::string>("colormap", colormap, std::string(DEFAULT_COLORMAP));
	param_n.param<int>("n_points", n_points_map, DEFAULT_N_POINTS_MAP);
	param_n.param<double>("max_sensor_val", max_sensor_val, DEFAULT_MAX_SENSOR_VAL);
	param_n.param<double>("min_sensor_val", min_sensor_val, DEFAULT_MIN_SENSOR_VAL);
	param_n.param<double>("sensor_offset_x", sensor_offset_x, DEFAULT_SENSOR_OFFSET_X);
	param_n.param<double>("sensor_offset_y", sensor_offset_y, DEFAULT_SENSOR_OFFSET_Y);
	param_n.param<double>("sensor_offset_x", sensor_offset_z, DEFAULT_SENSOR_OFFSET_Z);



	ROS_INFO("Algorithm parameters: ");
	ROS_INFO("   - Fixed frame: %s",frame_id.c_str());
	ROS_INFO("   - Max X (map): %f",map_max_x);
	ROS_INFO("   - Min X (map): %f",map_min_x);
	ROS_INFO("   - Max Y (map): %f",map_max_y);
	ROS_INFO("   - Min Y (map): %f",map_min_y);
	ROS_INFO("   - Cell size: %f",cell_size);
	//ROS_INFO("   - Kernel size: %f",kernel_size);
	ROS_INFO("   - Sensor offset (x): %f",sensor_offset_x);
	ROS_INFO("   - Sensor offset (y): %f",sensor_offset_y);
	ROS_INFO("   - Max sensor value: %f",max_sensor_val);
	ROS_INFO("   - Min sensor value: %f",min_sensor_val);
	ROS_INFO("   - Colormap %s",colormap.c_str());
	ROS_INFO("   - Number of points %d",n_points_map);
	*/



	//------------------------------------------------------------
	//----------------------------------
	// Subscriptions
	//----------------------------------	
	//ros::Subscriber sub_nose = param_n.subscribe(sensor_topic, 1000, noseCallback);
	//ros::Subscriber sub_position = param_n.subscribe(position_topic, 1000, positionCallback);	
	//----------------------------------
	// Advertisements
	//----------------------------------
	ros::Publisher gas_advertise = param_n.advertise<sensor_msgs::PointCloud2>("visualization_map", 200);


	//ros::Publisher var_advertise = param_n.advertise<sensor_msgs::PointCloud2>("var_map", 20);
	//----------------------------------
	// Initializes the algorithm
	//----------------------------------

	//==============================================================================================
	// 1. read log file with readingLogFiles or readSingleLogFile
	// 2. get x,y,z and concentration (here we treat concentration as reading)
	// 3. publish x,y,z and concentration
	// 4. take them as input for "addDataPoint" function
	//===============================================================================================

	//ReadLogFiles();

	//std::vector<int>::size_type sz = vec_px.size();
	//std::cout<<"size of vector "<<sz/num_snapshots<<std::endl;


	/*
	float timeFlag=1;
	int count_cycles=0;
	int dataIndex=0;
	while (ros::ok() && dataIndex<sz){

		//ROS_INFO("the current position is. %f", vec_px[dataIndex]);

		curr_x = vec_px[dataIndex];
		curr_y = vec_py[dataIndex];
		curr_z = vec_pz[dataIndex];

		curr_reading = vec_concentration[dataIndex];

		GDM_map.addDataPoint(curr_x+sensor_offset_x,curr_y+sensor_offset_y, curr_z+sensor_offset_z, curr_reading, timeFlag);

        timeFlag = timeFlag+1;


		if (count_cycles >= 2000) {
			GDM_map.publishMap(mean_advertise);
			//ROS_INFO("working and timeflag------------------------------------): %f",timeFlag);
			count_cycles=0;
			timeFlag = 1;
		}

		count_cycles++;

		dataIndex++;

	}*/


	snapshot = 1;

	while (ros::ok() && snapshot<num_snapshots){

		gas_map		GDM_map(map_min_x,map_max_x, map_min_y, map_max_y, map_min_z, map_max_z, cell_size, min_sensor_val,max_sensor_val, colormap, frame_id, n_points_map);

		ReadSingleLogFile(snapshot,input_result_location);

		std::vector<int>::size_type single_sz = vec_px.size();
		//std::cout<<"size of vector within a snapshot: "<<single_sz<<std::endl;

		cellConcentration.concentraion_value=arr3dAlloc((int)floor(map_max_x/cell_size),(int)floor(map_max_y/cell_size),(int)floor(map_max_z/cell_size));

		for(int dataIndex=0; dataIndex<single_sz; dataIndex++)
		{
			//ROS_INFO("the current position is: (%f, %f, %f)", vec_px[dataIndex], vec_py[dataIndex], vec_pz[dataIndex]);

			curr_x = vec_px[dataIndex];
			curr_y = vec_py[dataIndex];
			curr_z = vec_pz[dataIndex];

			curr_reading = vec_concentration[dataIndex]/1000;
			if (curr_reading<0){ROS_INFO("minus: (%f)",curr_reading);}

			GDM_map.addDataPoint(curr_x+sensor_offset_x,curr_y+sensor_offset_y, curr_z+sensor_offset_z, curr_reading);

			int xx,yy,zz;

			xx=(int)floor(curr_x + cell_size/2);
			yy=(int)floor(curr_y + cell_size/2);
			zz=(int)floor(curr_z + cell_size/2);

			cellConcentration.concentraion_value[xx][yy][zz]=curr_reading;

			//if(curr_reading>5){
				//ROS_INFO(" At the position %d %d %d, concentration is %f",xx,yy,zz, cellConcentration.concentraion_value[xx][yy][zz]);
			//}
		}

		// In each loop we want to update positions and concentrations so refreshing the vectors is needed.
		vec_px.clear();
		vec_py.clear();
		vec_pz.clear();
		vec_concentration.clear();


		GDM_map.publishMap(gas_advertise);



		ros::spinOnce();
		loop_rate.sleep();

		//ROS_INFO("Now No. %d snapshot is being processed and showing", snapshot);
	}


	//ROS_INFO("Spinning");

	ROS_INFO(" The simulation files are run out");
}



void	loadNodeParameters(ros::NodeHandle private_nh)
{

	std::string parameter_name;
	std::string param_string;
	double param_double;
	int	param_int;
	bool param_bool;


	//max value of sensor
	parameter_name=std::string(NODE_NAME)+std::string("/MAX_CONCENTRATION");
	if(private_nh.getParam(parameter_name,param_double)) {
		max_sensor_val=param_double;
	}
	else {
		max_sensor_val=DEFAULT_MAX_SENSOR_VAL;
	}

	//min value of sensor
	parameter_name=std::string(NODE_NAME)+std::string("/MIN_CONCENTRATION");
	if(private_nh.getParam(parameter_name,param_double)) {
		min_sensor_val=param_double;
	}
	else {
		min_sensor_val=DEFAULT_MIN_SENSOR_VAL;
	}


	//color map
	parameter_name=std::string(NODE_NAME)+std::string("/COLORMAP");
	if(private_nh.getParam(parameter_name,param_string)) {
		colormap=param_string;
	}
	else {
		colormap=DEFAULT_COLORMAP;
	}


	//number of points
	parameter_name=std::string(NODE_NAME)+std::string("/NUM_N_POINTS_MAP");

	if(private_nh.getParam(parameter_name,param_int)) {
		n_points_map=param_int;
	}
	else {
		n_points_map=DEFAULT_N_POINTS_MAP;
	}

	//fixed frame
	parameter_name=std::string(NODE_NAME)+std::string("/FIXED_FRAME");

	if(private_nh.getParam(parameter_name,param_string)) {
		frame_id=param_string;
	}
	else {
		frame_id=DEFAULT_FRAME_ID;
	}

	//snapshots
	parameter_name=std::string(NODE_NAME)+std::string("/NUM_SNAPSHOTS");

	if(private_nh.getParam(parameter_name,param_int)) {
		num_snapshots = param_int;
	}
	else {
		num_snapshots = DEFAULT_SNAPSHOTS;
	}

	//Exploration area size x
	parameter_name=std::string(NODE_NAME)+std::string("/MIN_X");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_min_x=param_double;
	}
	else {
		map_min_x=DEFAULT_MAP_MIN_X;
	}


	//Exploration area size x
	parameter_name=std::string(NODE_NAME)+std::string("/MAX_X");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_max_x=param_double;
	}
	else {
		map_max_x=DEFAULT_MAP_MAX_X;
	}


	//Exploration area size x
	parameter_name=std::string(NODE_NAME)+std::string("/MIN_Y");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_min_y=param_double;
	}
	else {
		map_min_y=DEFAULT_MAP_MIN_Y;
	}


	//Exploration area size y
	parameter_name=std::string(NODE_NAME)+std::string("/MAX_Y");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_max_y=param_double;
	}
	else {
		map_max_y=DEFAULT_MAP_MAX_Y;
	}

	//Exploration area size z
	parameter_name=std::string(NODE_NAME)+std::string("/MIN_Z");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_min_z=param_double;
	}
	else {
		map_min_z=DEFAULT_MAP_MIN_Z;
	}


	//Exploration area size z
	parameter_name=std::string(NODE_NAME)+std::string("/MAX_Z");

	if(private_nh.getParam(parameter_name,param_double)) {
		map_max_z=param_double;
	}
	else {
		map_max_z=DEFAULT_MAP_MAX_Z;
	}



	//Cell size for the exploration area
	parameter_name=std::string(NODE_NAME)+std::string("/CELL_SIZE");

	if(private_nh.getParam(parameter_name,param_double)) {
		cell_size=param_double;
	}
	else {
		cell_size=DEFAULT_CELL_SIZE;
	}

	// The location of the simulation results.
	parameter_name=std::string(NODE_NAME)+std::string("/result_location");
	if(private_nh.getParam(parameter_name,param_string)) {
		input_result_location=param_string;
	}
	else {
		input_result_location=DEFAULT_RESULT_LOCATION;
	}


	// The parameter for setting the enviornment temperature.
	parameter_name=std::string(NODE_NAME)+std::string("/temperature");
	if(private_nh.getParam(parameter_name,param_double)) {
		input_temperature=param_double;
	}
	else {
		input_temperature=DEFAULT_TEMPERATURE;
	}

	// The parameter for setting the enviorment pressure.
	parameter_name=std::string(NODE_NAME)+std::string("/pressure");
	if(private_nh.getParam(parameter_name,param_double)) {
		input_pressure=param_double;
	}
	else {
		input_pressure=DEFAULT_PRESSURE;
	}

	// The parameter for setting the unit for concentration.
	parameter_name=std::string(NODE_NAME)+std::string("/concentration_unit_choice");
	if(private_nh.getParam(parameter_name,param_int)) {
		input_unit_choice=param_int;
	}
	else {
		input_unit_choice=DEFAULT_UNIT_CHOICE;
	}



	ROS_INFO("Visualization parameters: ");
	ROS_INFO("   - Fixed frame: %s",frame_id.c_str());
	ROS_INFO("   - Max X (map): %f",map_max_x);
	ROS_INFO("   - Min X (map): %f",map_min_x);
	ROS_INFO("   - Max Y (map): %f",map_max_y);
	ROS_INFO("   - Min Y (map): %f",map_min_y);
	ROS_INFO("   - Cell size: %f",cell_size);
	ROS_INFO("   - Sensor offset (x): %f",sensor_offset_x);
	ROS_INFO("   - Sensor offset (y): %f",sensor_offset_y);
	ROS_INFO("   - Max sensor value: %f",max_sensor_val);
	ROS_INFO("   - Min sensor value: %f",min_sensor_val);
	ROS_INFO("   - Colormap %s",colormap.c_str());
	ROS_INFO("   - Number of snapshots %d",num_snapshots);
	ROS_INFO("   - Number of points %d",n_points_map);
}
