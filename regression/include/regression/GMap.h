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
#include <list>
#include <map>

#include "regression/Position.h"

#define CELL_OCCUPATION_PROBABILITY_THRESHOLD 35

using namespace std;
class GMap {
private:
	double resolution;
	unsigned int width;
	unsigned int height;

	int** occupancyGrid;

	bool _isInit;
	Position origin;
	std::list<Position> triedGoals;

	bool isTriedPos(Position position);


public:
	GMap();
	void init(	unsigned int width,
				unsigned int height,
				double resolution,
				double origin_x,
				double origin_y,
				int * data
			);
	bool isOccupied(Position position);
	int getOccupancyValue(Position position);
	void updateGrid(int x, int y, int value);
	bool isWithinBounds(Position position);
	bool isWithinBoundsX(int x);
	bool isWithinBoundsY(int y);
	bool isInitialized();
	void printMap();
	void printMeanMap(map<Position, double> globalMeanMap, list<Position> sensingPositions);
	void printVarianceMap(map<Position, double> globalVarianceMap, list<Position> sensingPositions);
	void addTriedPosition(Position position);
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

	const Position& getOrigin() const {
		return origin;
	}

	void setOrigin(const Position& origin) {
		this->origin = origin;
	}
};

#endif /* GAS_SOURCE_LOCALIZATION_REGRESSION_SRC_GMAP_H_ */
