
#ifdef __cplusplus
 extern "C" {
 #endif

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "dispersion_simulation/foamToFilament.h"
#include "dispersion_simulation/utility.h"
#include "dispersion_simulation/flow.h"
#include "dispersion_simulation/flow_reader.h"


#include "dispersion_simulation/environment.h"
#include "dispersion_simulation/hit_rate.h"


//#include "display.h"
#include <stdio.h>
#include <stdlib.h>
//100
//#define NUM_FILAMENT 300
//#define NUM_SNAPSHOT 100

#define STORE_SIM_IN_FILE


#define WIND_DATA	"/home/victor/ros_catkin/odour_simulation_ws/src/dispersion_simulation/CSV/Velocity0."

//../CSV/Velocity0.



 struct sAlgorithm {



	 int max_area_size_z;
	 struct sFlow flow;
	 struct sFlowReader fr;
	 struct sHitRate hit_rate;
	 struct sFilament *filament;
	 //struct sFilament filament[NUM_FILAMENT*NUM_SNAPSHOT]; //sepid


	 struct sEnvironment environment;

	 struct {
		 double x;
		 double y;
		 double z;//3d
	 } source;

	 struct {
		 int steps_count;
	 } state;


	 struct
	 {
		 int counter;


		 double *x;
		 double *y;
		 double *z;
		 double *width;
		 double *colour;




		 /*double x[NUM_FILAMENT*NUM_FILAMENT];
double y[NUM_FILAMENT*NUM_FILAMENT];
double z[NUM_FILAMENT*NUM_FILAMENT]; //3d
double width[NUM_FILAMENT*NUM_FILAMENT];
double colour[NUM_FILAMENT*NUM_FILAMENT];*/

	 } marker;

 };

// BEGIN - GENERATED BY C-UPDATE-HEADER
// Initialize the algorithm structure


char wind_files[200];
char output1 [256];
char output2 [256];
int n;
void algorithm_init(char const *,int,int,int,int,int,double,double,double,double,double);
void algorithm_calculate_all();
// Calculates one hit_rate
int algorithm_hit_rate_calculate(int, int, double x1, double y1, double z1, double x2, double y2, double z2, char *filePath, double, long long int, double, double, int, double); //3d

extern double concentration(float, float,  float, float, double, double, int);

// END - GENERATED BY C-UPDATE-HEADER

#endif

#ifdef __cplusplus
 }
 #endif
