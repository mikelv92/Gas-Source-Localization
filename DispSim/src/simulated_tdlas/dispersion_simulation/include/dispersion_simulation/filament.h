#ifndef FILAMENT_H
#define FILAMENT_H

#include "dispersion_simulation/Filamentproperties.h"
struct sFilament {
	struct {
		double stddev;			//!< The standard deviation of the superposed stochastic process.
		double growthgamma;		//!< The gamma parameter of the filament growth [m^2/s].
	} configuration;

	int max_z;
	struct {
		double x;
		double y;
		double z;
		double width;
		int valid;
	} state;

	struct {		//for buoyant force calculation
		float air;
		float gas;
		float g;
		float a;	//accleration
	} value;

};

// BEGIN - GENERATED BY C-UPDATE-HEADER
void filament_init(struct sFilament *f, double x, double y, double z, int, double);//3d
void filament_step(struct sFilament *f, double simulation_step);

// END - GENERATED BY C-UPDATE-HEADER


#endif
