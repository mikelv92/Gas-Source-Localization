#include "dispersion_simulation/algorithm.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/select.h>

#include "dispersion_simulation/filament.h"
#include "dispersion_simulation/global.h"
#include "dispersion_simulation/mtwist.h"

// Initialize the algorithm structure
void algorithm_init(char const *wind_files_in, int num_snapshots, int num_filaments, int area_size_x, int area_size_y, int area_size_z, double area_cell_size, double source_x, double source_y, double source_z, double spreading_width_std) {
	
	
	algorithm.max_area_size_z=area_size_z;


	// Read the wind field
	flow_init(&algorithm.flow,area_size_x,area_size_y,area_size_z,area_cell_size);

	char tmp_U[600];
	char tmp_W[600];
	char tmp_V[600]; //3d

	strcpy(output1, "0.0");
	strcpy(output2, "0.0");
	setNext2(output2);
	setNext1(output1);
	
	strcpy(wind_files,wind_files_in);

	strcpy(tmp_U, wind_files);
	strcpy(tmp_W, wind_files);
	strcpy(tmp_V, wind_files);//3d

	strcat(tmp_U, output2);
	strcat(tmp_W, output2);
	strcat(tmp_V, output2); //3d
	strcat(tmp_U, ".csv_U");
	strcat(tmp_W, ".csv_W");
	strcat(tmp_V, ".csv_V");//3d

	flow_read( &algorithm.flow, tmp_U, tmp_V, tmp_W, 1); //sepid


	//flow_set_uniform(&algorithm.flow, 1.0, 0.4);
	//flow_set_fancy(&algorithm.flow);


	// Set up the environment
	environment_init(&algorithm.environment,area_size_x,area_size_y,area_size_z);

	

	// Initialize hit rate and filament
	
	hit_rate_init(&algorithm.hit_rate, area_size_x, area_size_y, area_size_z, area_cell_size);

	algorithm.filament=malloc(num_snapshots*num_filaments*sizeof(struct sFilament));
	algorithm.marker.x= malloc(num_snapshots*num_filaments*sizeof(double));
	algorithm.marker.y=malloc(num_snapshots*num_filaments*sizeof(double));
	algorithm.marker.z=malloc(num_snapshots*num_filaments*sizeof(double));
	algorithm.marker.width=malloc(num_snapshots*num_filaments*sizeof(double));
	algorithm.marker.colour=malloc(num_snapshots*num_filaments*sizeof(double));
	
	/*
	algorithm.filament=malloc(num_filaments*num_filaments*sizeof(struct sFilament));
	algorithm.marker.x= malloc(num_filaments*num_filaments*sizeof(double));
	algorithm.marker.y=malloc(num_filaments*num_filaments*sizeof(double));
	algorithm.marker.z=malloc(num_filaments*num_filaments*sizeof(double));
	algorithm.marker.width=malloc(num_filaments*num_filaments*sizeof(double));
	algorithm.marker.colour=malloc(num_filaments*num_filaments*sizeof(double));
	*/
		
	int i;
	for (i = 0; i < num_filaments*num_snapshots; i++)
		filament_init(&algorithm.filament[i], 0, 0, 0.05, area_size_z, spreading_width_std);


	//	filament_init(&algorithm.filament, 0, 0);
	// Source

	algorithm.source.x = source_x;
	algorithm.source.y = source_y;
	algorithm.source.z = source_z;
	
	//algorithm.source.z = in main, message received from environment 

	// State
	algorithm.state.steps_count = 0;
	
	strcpy(output1, "0.0");
	strcpy(output2, "0.0"); //simulation will start at time 0 as defualt
	
	n = 1; //it use for NUM_SNAPSHOT
	
}



/*void algorithm_calculate_all() {
	int x;
	for (x = 0; x < 90; x++) {
		int y;
		for (y = 0; y < 50; y++) {
			// File name
			char filename[128];
			sprintf(filename, "results/hit_rate_%d_%d", x, y);
			

			// Calculate and save the hit_rate
			double px = x * 0.2;
			double py = y * 0.2;
			algorithm_hit_rate_calculate(px, py, px + 0.2, py + 0.2,0,0);
			hit_rate_save(&algorithm.hit_rate, filename);
		}
	}
}*/



void algorithm_simulate_filaments(int num_filaments, double x1, double y1, double z1, double x2, double y2, double z2, double spreading_width_std){ //3d
	// Simulate filaments
	int i;
	
	for (i = 0; i < num_filaments; i++) {
		double x = mt_drand() * (x2 - x1) + x1;
		double y = mt_drand() * (y2 - y1) + y1;
		double z = mt_drand() * (z2 - z1) + z1;
		filament_init(&algorithm.filament[(n-1)*num_filaments + i], x, y, z, algorithm.max_area_size_z, spreading_width_std);
	}
}



// Calculates one hit_rate
int algorithm_hit_rate_calculate(int num_filaments, int num_snapshots, double x1, double y1, double z1, double x2, double y2, double z2, char *filePath, double spreading_width_std, long long int num_molecules, double temperature, double pressure, int unit_choice, double windflow_resolution) {//3d

	algorithm_simulate_filaments(num_filaments, x1, y1, z1, x2, y2, z2, spreading_width_std);

	char temp_U [600];
	temp_U[0] = '\0';

	char temp_V [600];//3d
	temp_V[0] = '\0';

	char temp_W [600];
	temp_W[0] = '\0';

	char temp_out [600];
	temp_out[0] = '\0';


	//print final output ------------------------------------------------------------------
#ifdef STORE_SIM_IN_FILE
	FILE *out;
	strcpy(temp_out, filePath);
	//strcpy(temp_out, "/home/han/gdm_simulator_catkin_ws/src/simulated_tdlas/recordedData/results/gasType1_simulation_output");
	setNext1(output1);
	strcat(temp_out, output1);

	out = fopen( temp_out, "w" );
	if( out == NULL )
		printf("Error in opening the final output files.\n");

	fprintf(out, "px\t py\t pz\t concentration\t u\t v\t w \n");
	//fprintf(out, "x\t y\t z\t px\t py\t pz\t value[x][y][z]\t u\t v\t w \n");
	//fprintf(out, "px\t py\t pz\t concentration \n");


	int x,y,z,jj,flow_x,flow_y,flow_z;
	double px,py,pz,ppm,contributedConcentration;

	/*for (i = 0; i < num_filaments*n; i++) {
		px = algorithm.marker.x[i];
		py = algorithm.marker.y[i];
		pz = algorithm.marker.z[i];

		ppm = concentration(px,py,pz,algorithm.hit_rate.cells_size.x);;
		fprintf(out,"%f   %f   %f   %f\n", px, py, pz, ppm);
	}*/


	// record the concentration of every cell and record the corresponding cell positions and winde data.
	for (z = 0; z < algorithm.hit_rate.cells_count.z-1; z++) {
		for (x = 0; x < algorithm.hit_rate.cells_count.x-1; x++) {
			for (y = 0; y < algorithm.hit_rate.cells_count.y-1; y++) {

				px= x * algorithm.hit_rate.cells_size.x + algorithm.hit_rate.cells_size.x/2;
				py= y * algorithm.hit_rate.cells_size.y + algorithm.hit_rate.cells_size.y/2;
				pz= z * algorithm.hit_rate.cells_size.z + algorithm.hit_rate.cells_size.z/2;

				for (jj = 0; jj < num_filaments*n; jj++){
					contributedConcentration = (1./ pow(algorithm.filament[jj].state.width, 3)*num_molecules)*exp(-sqrt(pow(px-algorithm.filament[jj].state.x,2)+pow(py-algorithm.filament[jj].state.y,2)+pow(pz-algorithm.filament[jj].state.z,2))/pow(algorithm.filament[jj].state.width, 2));
					//hit_rate_add(&algorithm.hit_rate, px, py, pz, contributedConcentration);
					hit_rate_add(&algorithm.hit_rate, x * algorithm.hit_rate.cells_size.x, y * algorithm.hit_rate.cells_size.y, z * algorithm.hit_rate.cells_size.z, contributedConcentration);
				}

				ppm = concentration(px,py,pz,algorithm.hit_rate.cells_size.x, temperature, pressure, unit_choice);

				flow_x=(int) floor(x*algorithm.hit_rate.cells_size.x);
				flow_y=(int) floor(y*algorithm.hit_rate.cells_size.y);
				flow_z=(int) floor(z*algorithm.hit_rate.cells_size.z);

				fprintf(out,"%f   %f   %f   %f   %f   %f   %f\n",px, py, pz, ppm, algorithm.flow.u[flow_x][flow_y][flow_z], algorithm.flow.v[flow_x][flow_y][flow_z], algorithm.flow.w[flow_x][flow_y][flow_z]);

			}
		}
	}

	fclose(out);
#endif


	int nn, i;
	int steps=20/windflow_resolution;
	float stepSize = 1./steps;

	if ( n < num_snapshots) {

		strcpy(temp_U, wind_files);
		strcpy(temp_W, wind_files);
		strcpy(temp_V, wind_files);//3d
		setNext2(output2);

		strncat(temp_U, output2, strlen(output2));
		strncat(temp_W, output2, strlen(output2));
		strncat(temp_V, output2, strlen(output2));//3d
		strcat(temp_U, ".csv_U");
		strcat(temp_W, ".csv_W");
		strcat(temp_V, ".csv_V");//3d

		//puts(temp_V);
		printf("The current wind file is No.%s \n", output2);

		flow_read( &algorithm.flow, temp_U, temp_V, temp_W, 0); //sepid

		for(nn = 0; nn < steps; nn++)
			for (i = 0; i < num_filaments*n; i++) {
				filament_step(&algorithm.filament[i], stepSize);
				if (! algorithm.filament[i].state.valid) {
					break;
				}

				//hit_rate_add(&algorithm.hit_rate, algorithm.filament[i].state.x, algorithm.filament[i].state.y, algorithm.filament[i].state.z, 1/pow(algorithm.filament[i].state.width, 3));
                // refer to Eq. 10 and 11 in the paper.

				algorithm.marker.x[i]=algorithm.filament[i].state.x;
				algorithm.marker.y[i]=algorithm.filament[i].state.y;
				algorithm.marker.z[i]=algorithm.filament[i].state.z;  //3d

				algorithm.marker.width[i]=algorithm.filament[i].state.width;
				algorithm.marker.colour[i]=(algorithm.filament[i].state.width-0.04)*2.0;

			}

		algorithm.marker.counter=n;
		n++;


		algorithm.state.steps_count ++;	
		//-------------------------------------------------------------------------
	}
	// Rescale the hit_rate
	hit_rate_rescale(&algorithm.hit_rate, 1. / 1000000.);
	//hit_rate_save(&algorithm.hit_rate, "/home/han/gdm_simulator_catkin_ws/src/simulated_tdlas/recordedData/results/test-hit_rate");
	return 1;
}



double concentration(float x_in, float y_in, float z_in, float cell_size, double temperature, double pressure, int if_ppm) {
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
	
	xx=(int)floor((x_in - cell_size/2)/cell_size); //+
	yy=(int)floor((y_in - cell_size/2)/cell_size);
	zz=(int)floor((z_in - cell_size/2)/cell_size);
	
	double conct;

	conct = (algorithm.hit_rate.value[xx][yy][zz])*scale;//
	//printf(" %f \n", conct);
	return conct;
	}
