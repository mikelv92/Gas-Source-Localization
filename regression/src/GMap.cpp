/*
 * GMap.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: mikel
 */

#include "regression/GMap.h"
GMap::GMap()
{
	this->_isInit 		= false;
	this->width 		= 0;
	this->height 		= 0;
	this->resolution 	= 0;
}

void GMap::init(unsigned int width,
				unsigned int height,
				double resolution,
				double origin_x,
				double origin_y,
				int * data
				)
{
	this->width 		= width;
	this->height 		= height;
	this->resolution 	= resolution;
	this->origin 		= Position(origin_x, origin_y);

	occupancyGrid 		= (int **)malloc(width * sizeof(int *));

	for (int i = 0; i < width; i++)
		occupancyGrid[i] = (int *)malloc(height * sizeof(int));

	for (int j = 0; j < height; j++)
		for (int i = 0; i < width; i++)
			occupancyGrid[i][j] = data[width * j + i];

	this->_isInit = true;
}

bool GMap::isOccupied(Position position)
{
    int offset_x 	= -origin.getX() / resolution;
    int offset_y 	= -origin.getY() / resolution;
    int x 			= position.getX() / resolution + offset_x;
    int y 			= position.getY() / resolution + offset_y;

	return occupancyGrid[x][y] > CELL_OCCUPATION_PROBABILITY_THRESHOLD || occupancyGrid[x][y] == -1;
}

bool GMap::isInitialized()
{
	return this->_isInit;
}

bool GMap::isWithinBounds(Position position)
{
	return position.getX() < origin.getX() + width * resolution
			&& position.getX() >= -origin.getX() - width * resolution
			&& position.getY() < origin.getY() + height * resolution
			&& position.getY() >= -origin.getY() - height * resolution;
}

bool GMap::isWithinBoundsX(int x)
{
	return x < origin.getX() + width * resolution && x >= -origin.getX() - width * resolution;
}

bool GMap::isWithinBoundsY(int y)
{
	return y < origin.getY() + height * resolution && y >= -origin.getY() - height * resolution;
}

int GMap::getOccupancyValue(Position position)
{
    int offset_x 	= -origin.getX() / resolution;
    int offset_y 	= -origin.getY() / resolution;
    int x 			= position.getX() / resolution + offset_x;
    int y 			= position.getX() / resolution + offset_y;

	return occupancyGrid[x][y];
}

void GMap::updateGrid(int x, int y, int value)
{
	occupancyGrid[x][y] = value;
}

GMap::~GMap() {
	for (int i = 0; i < this->width; i++)
		if (occupancyGrid[i])
			free(occupancyGrid[i]);

	if (occupancyGrid)
		free(occupancyGrid);
}

