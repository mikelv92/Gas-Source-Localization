/*
 * GMap.h
 *
 *  Created on: Jun 28, 2016
 *      Author: mikel
 */

#ifndef GAS_SOURCE_LOCALIZATION_REGRESSION_SRC_GMAP_H_
#define GAS_SOURCE_LOCALIZATION_REGRESSION_SRC_GMAP_H_

#include <stdio.h>
#include <stdlib.h>

#include "regression/Position.h"

#define CELL_OCCUPATION_PROBABILITY_THRESHOLD 50

class GMap {
private:
	double resolution;
	unsigned int width;
	unsigned int height;

	Position origin;

	int** occupancyGrid;

	bool _isInit;

public:
	GMap() { _isInit = false; };
	void init(unsigned int width, unsigned int height, double resolution, int * data);
	bool isOccupied(Position position);
	bool isInitialized();
	virtual ~GMap();

	unsigned int getHeight() const {
		return height;
	}

	void setHeight(unsigned int height) {
		this->height = height;
	}

	double getResolution() const {
		return resolution;
	}

	void setResolution(double resolution) {
		this->resolution = resolution;
	}

	unsigned int getWidth() const {
		return width;
	}

	void setWidth(unsigned int width) {
		this->width = width;
	}
};

#endif /* GAS_SOURCE_LOCALIZATION_REGRESSION_SRC_GMAP_H_ */
