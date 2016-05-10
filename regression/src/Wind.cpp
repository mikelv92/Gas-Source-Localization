/*
 * Wind.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Wind.h"

Wind::Wind(double u, double v, double w) {
	this->u = u;
	this->v = v;
	this->w = w;
	this->speed2D = sqrt(u * u + v * v);

}

Wind::Wind()
{
	this->u = 0;
	this->v = 0;
	this->w = 0;
	this->speed2D = 0;
}

Wind::~Wind() {
	// TODO Auto-generated destructor stub
}
