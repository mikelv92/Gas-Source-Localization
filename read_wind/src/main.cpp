#include "read_wind/main.h"

void setOutput2(int snapshot)
{
	//Convert snapshot from int to char*
	char output[256];
	int i;
	for (i = 0; snapshot != 0; i++)
	{
		output[i] = '0' + snapshot % 10;
		snapshot /= 10;
	}
	output[i] = '\0';

	int size = strlen(output);
	for (int i = 0; i < size; i++)
		output2[i] = output[size - i - 1];
	output2[i] = '\0';
}


bool getWind(read_wind::WindData::Request &req, read_wind::WindData::Response &res)
{
	char temp_U [600];
	temp_U[0] = '\0';

	char temp_V [600];//3d
	temp_V[0] = '\0';

	char temp_W [600];
	temp_W[0] = '\0';

	char temp_out [600];
	temp_out[0] = '\0';

	int snapshot = req.snapshot;

	setOutput2(snapshot);

	if (snapshot < input_snapshots) {

		strcpy(temp_U, wind_files);
		strcpy(temp_W, wind_files);
		strcpy(temp_V, wind_files);//3d


		strncat(temp_U, output2, strlen(output2));
		strncat(temp_W, output2, strlen(output2));
		strncat(temp_V, output2, strlen(output2));//3d
		strcat(temp_U, ".csv_U");
		strcat(temp_W, ".csv_W");
		strcat(temp_V, ".csv_V");//3d

		//puts(temp_V);
		printf("The current wind file is No. %s \n", output2);

		flow_read(&flow, temp_U, temp_V, temp_W, 0); //sepid

		double x = req.x;
		double y = req.y;
		double z = req.z;

		flow_get(&flow, x, y, z);

		res.u = flow.result.u;
		res.w = flow.result.w;
		res.v = flow.result.v;
		return true;
	}
	else
	{
		printf("Snapshot not available\n");
		return false;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;

	loadNodeParameters(nh);

	flow_init(&flow, input_area_size_x, input_area_size_y, input_area_size_z, input_area_cell_size);
	char tmp_U[600];
	char tmp_W[600];
	char tmp_V[600]; //3d

	strcpy(output1, "0.0");
	strcpy(output2, "0.0");
	setNext2(output2);
	setNext1(output1);

	strcpy(wind_files, input_wind_data.c_str());

	strcpy(tmp_U, wind_files);
	strcpy(tmp_W, wind_files);
	strcpy(tmp_V, wind_files);//3d

	strcat(tmp_U, output2);
	strcat(tmp_W, output2);
	strcat(tmp_V, output2); //3d
	strcat(tmp_U, ".csv_U");
	strcat(tmp_W, ".csv_W");
	strcat(tmp_V, ".csv_V");//3d
	flow_read(&flow, tmp_U, tmp_V, tmp_W, 1); //sepid

	strcpy(output1, "0.0");
	strcpy(output2, "0.0"); //simulation will start at time 0 as defualt
	n = 1;

	ros::ServiceServer service = nh.advertiseService("read_wind", getWind);

	ros::spin();
	return 0;
}

void loadNodeParameters(ros::NodeHandle private_nh)
{

	std::string parameter_name;
	std::string param_string;
	double	param_double;
	int		param_int;
	bool	param_bool;


	//wind files
	parameter_name=std::string(NODE_NAME)+std::string("/wind_data");
	if(private_nh.getParam(parameter_name,param_string)) {
		input_wind_data=param_string;
	}
	else {
		input_wind_data=DEFAULT_WIND_DATA;
	}

	//snapshots
	parameter_name=std::string(NODE_NAME)+std::string("/snapshots");

	if(private_nh.getParam(parameter_name,param_int)) {
		input_snapshots=param_int;
	}
	else {
		input_snapshots=DEFAULT_SNAPSHOTS;
	}

	//Exploration area size x
	parameter_name=std::string(NODE_NAME)+std::string("/area_size_x");

	if(private_nh.getParam(parameter_name,param_int)) {
		input_area_size_x=param_int;
	}
	else {
		input_area_size_x=DEFAULT_AREA_X;
	}


	//Exploration area size y
	parameter_name=std::string(NODE_NAME)+std::string("/area_size_y");

	if(private_nh.getParam(parameter_name,param_int)) {
		input_area_size_y=param_int;
	}
	else {
		input_area_size_y=DEFAULT_AREA_Y;
	}


	//Exploration area size z
	parameter_name=std::string(NODE_NAME)+std::string("/area_size_z");

	if(private_nh.getParam(parameter_name,param_int)) {
		input_area_size_z=param_int;
	}
	else {
		input_area_size_z=DEFAULT_AREA_Z;
	}

	//Cell size for the exploration area
	parameter_name=std::string(NODE_NAME)+std::string("/cell_size");

	if(private_nh.getParam(parameter_name,param_double)) {
		input_area_cell_size=param_double;
	}
	else {
		input_area_cell_size=DEFAULT_AREA_CELL_SIZE;
	}


	ROS_INFO("The parameters are:");
	ROS_INFO("wind file location: %s",input_wind_data.c_str());
	ROS_INFO("Snapshots: %d",input_snapshots);
	ROS_INFO("Area size (%d,%d,%d) - with cell %f",input_area_size_x,input_area_size_y,input_area_size_z,input_area_cell_size);


}

