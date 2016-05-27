
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>



#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <boost/math/constants/constants.hpp>

const double PI = boost::math::constants::pi<double>();
#define NUM_CELL_TEMPLATES 200
#define MAX_POINTS_PER_CELL 1000

class gas_map{
public:
	gas_map(double, double, double, double, double, double, double, double, double, std::string, std::string, int);
	void	addDataPoint(double, double, double, double);
	void	publishMap(ros::Publisher &);

private:
	int		num_rows;
	int		num_cols;
	int		num_height;
	double map_min_x;
	double map_max_x;
	double map_min_y;

	double map_max_y;
	double map_max_z;
	double map_min_z;
	double cell_size;
	//double kernel_size;
	//double sigma_omega;
	double min_conc;
	double max_conc;

	//double global_mean;
	//double global_variance;
	//float num_samples;

	cv::Mat concentrationMap;


	std::string		colormap;
	std::string		frame_id;

	pcl::PointCloud<pcl::PointXYZRGB> template_cells[NUM_CELL_TEMPLATES];



};
