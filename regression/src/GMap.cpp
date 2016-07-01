/*
 * GMap.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: mikel
 */

#include "regression/GMap.h"
GMap::GMap()
{
	this->_isInit = false;
	this->width = 0;
	this->height = 0;
}

void GMap::init(unsigned int width, unsigned int height, double resolution, int * data) {
	this->width = width;
	this->height = height;
	this->resolution = resolution;

	occupancyGrid = (int **)malloc(width * sizeof(int *));
	for (int i = 0; i < width; i++)
		occupancyGrid[i] = (int *)malloc(height * sizeof(int));

	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			occupancyGrid[i][j] = data[width * i + j];

	this->_isInit = true;
}

bool GMap::isOccupied(Position position)
{
	int x = position.getX();
	int y = position.getY();
	return occupancyGrid[x][y] > CELL_OCCUPATION_PROBABILITY_THRESHOLD;
}

bool GMap::isInitialized()
{
	return this->_isInit;
}

GMap::~GMap() {
	for (int i = 0; i < this->width; i++)
		if (occupancyGrid[i])
			free(occupancyGrid[i]);

	if (occupancyGrid)
		free(occupancyGrid);
}

