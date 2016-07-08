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

void GMap::init(unsigned int width,
				unsigned int height,
				double resolution,
				double origin_x,
				double origin_y,
				int * data
				)
{
	this->width = width;
	this->height = height;
	this->resolution = resolution;
	this->origin = Position(origin_x, origin_y);

	occupancyGrid = (int **)malloc(width * sizeof(int *));
	for (int i = 0; i < width; i++)
		occupancyGrid[i] = (int *)malloc(height * sizeof(int));

	for (int j = 0; j < height; j++)
		for (int i = 0; i < width; i++)
			occupancyGrid[i][j] = data[width * j + i];

	this->_isInit = true;
}

bool GMap::isOccupied(Position position)
{
    int offset_x = -origin.getX();
    int offset_y = -origin.getY();

    int tr = 1 / resolution;

    int x = position.getX() * tr + offset_x;
    int y = position.getY() * tr + offset_y;

	return occupancyGrid[x][y] > CELL_OCCUPATION_PROBABILITY_THRESHOLD || occupancyGrid[x][y] == -1;
}

bool GMap::isInitialized()
{
	return this->_isInit;
}

bool GMap::isWithinBounds(Position position)
{
	return position.getX() < origin.getX() + width && position.getX() >= origin.getX()
			&& position.getY() < origin.getY() + height && position.getY() >= origin.getY();
}

bool GMap::isWithinBoundsX(int x)
{
	return x < origin.getX() + width && x >= origin.getX();
}

bool GMap::isWithinBoundsY(int y)
{
	return y < origin.getY() + height && y >= origin.getY();
}

int GMap::getOccupancyValue(int x, int y)
{
    int offset_x = -origin.getX();
    int offset_y = -origin.getY();

    int tr = 1 / resolution;

	return occupancyGrid[x * tr  + offset_x][y * tr + offset_y];
}

GMap::~GMap() {
	for (int i = 0; i < this->width; i++)
		if (occupancyGrid[i])
			free(occupancyGrid[i]);

	if (occupancyGrid)
		free(occupancyGrid);
}

