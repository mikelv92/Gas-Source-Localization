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
	int occupancyValue = getOccupancyValue(position);
	return isTriedPos(position) || occupancyValue > CELL_OCCUPATION_PROBABILITY_THRESHOLD
			|| occupancyValue == -1;
}

bool GMap::isTriedPos(Position position)
{
	for (std::list<Position>::iterator it = triedGoals.begin(); it != triedGoals.end(); it++)
		if (position.equals(*it))
			return true;

	return false;
}

void GMap::addTriedPosition(Position position)
{
	triedGoals.push_back(position);
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
	int y 			= position.getY() / resolution + offset_y;

	return occupancyGrid[x][y];
}

void GMap::updateGrid(int x, int y, int value)
{
	occupancyGrid[x][y] = value;
}

void GMap::printMeanMap(map<Position, double> globalMeanMap, list<Position> sensingPositions)
{
	printf("Printing mean map...\n");

	FILE * file = fopen("meanMap.ppm", "wb"); /* b - binary mode */
	fprintf(file, "P6\n%d %d\n255\n", width, height);

	for (int j = width - 1; j >= 0; j--)
	{
		for (int i = 0; i < height; i++)
		{
			static unsigned char color[3];

			bool existsInMap = false;

			for (map<Position, double>::iterator it = globalMeanMap.begin(); it != globalMeanMap.end(); it++)
			{
				int offset_x 	= -origin.getX() / resolution;
				int offset_y 	= -origin.getY() / resolution;
				int low_x		= (it->first.getX() - 0.5) / resolution + offset_x;
				int low_y		= (it->first.getY() - 0.5) / resolution + offset_y;
				int high_x 		= (it->first.getX() + 0.5) / resolution + offset_x;
				int high_y 		= (it->first.getY() + 0.5) / resolution + offset_y;

				if (i > low_x && i < high_x
						&& j > low_y && j < high_y)
				{
					existsInMap = true;

					bool isSensingPos = false;

					for (list<Position>::iterator s = sensingPositions.begin(); s != sensingPositions.end(); s++)
						if ((it->first).equals(*s))
							isSensingPos = true;

					if (isSensingPos)
					{
						color[0] = (it->second / 0.0008) > 255 ? 255 : floor(it->second / 0.0008);  /* red */
						color[1] = 0;  /* green */
						color[2] = 0;  /* blue */
					}
					else
					{
						color[0] = (it->second / 0.0008) > 255 ? 255 : floor(it->second / 0.0008);  /* red */
						color[1] = 255;  /* green */
						color[2] = 255;  /* blue */
					}

					fwrite(color, 1, 3, file);

				}
			}
			if (!existsInMap)
			{
				int occupancyVal = occupancyGrid[i][j];
				color[0] = occupancyVal / 100 * 256;  /* red */
				color[1] = occupancyVal / 100 * 256;  /* green */
				color[2] = occupancyVal / 100 * 256;  /* blue */

				fwrite(color, 1, 3, file);

			}


		}
	}

	fclose(file);
	printf("Done.\n");


}

void GMap::printVarianceMap(map<Position, double> globalVarianceMap, list<Position> sensingPositions)
{
	printf("Printing variance map...\n");

	FILE * file = fopen("varianceMap.ppm", "wb"); /* b - binary mode */
	fprintf(file, "P6\n%d %d\n255\n", width, height);

	for (int j = height - 1; j >= 0; j--)
	{
		for (int i = 0; i < width; i++)
		{
			static unsigned char color[3];

			bool existsInMap = false;

			for (map<Position, double>::iterator it = globalVarianceMap.begin(); it != globalVarianceMap.end(); it++)
			{
				int offset_x 	= -origin.getX() / resolution;
				int offset_y 	= -origin.getY() / resolution;
				int low_x		= (it->first.getX() - 0.5) / resolution + offset_x;
				int low_y		= (it->first.getY() - 0.5) / resolution + offset_y;
				int high_x 		= (it->first.getX() + 0.5) / resolution + offset_x;
				int high_y 		= (it->first.getY() + 0.5) / resolution + offset_y;

				if (i > low_x && i < high_x
						&& j > low_y && j < high_y)
				{
					existsInMap = true;

					bool isSensingPos = false;

					for (list<Position>::iterator s = sensingPositions.begin(); s != sensingPositions.end(); s++)
						if ((it->first).equals(*s))
							isSensingPos = true;

					if (isSensingPos)
					{
						color[0] = (it->second / 0.0008) > 255 ? 255 : floor(it->second / 0.0008);  /* red */
						color[1] = 0;  /* green */
						color[2] = 0;  /* blue */
					}
					else
					{
						color[0] = (it->second / 0.0008) > 255 ? 255 : floor(it->second / 0.0008);  /* red */
						color[1] = 255;  /* green */
						color[2] = 255;  /* blue */
					}

					fwrite(color, 1, 3, file);

				}
			}
			if (!existsInMap)
			{
				int occupancyVal = occupancyGrid[i][j];
				color[0] = occupancyVal / 100 * 256;  /* red */
				color[1] = occupancyVal / 100 * 256;  /* green */
				color[2] = occupancyVal / 100 * 256;  /* blue */

				fwrite(color, 1, 3, file);
			}
		}
	}

	fclose(file);
	printf("Done.\n");

}

void GMap::printMap()
{
	printf("Printing costmap...\n");
	FILE * file = fopen("costmap.pgm", "w");

	fprintf(file, "P2\n4000\n4000\n100\n");

	for (int j = height - 1; j >= 0; j--)
	{
		for (int i = 0; i < height; i++)
			fprintf(file, "%d ", occupancyGrid[i][j]);
		fprintf(file, "\n");
	}

	fclose(file);
	printf("Finished printing costmap\n");
}

GMap::~GMap() {
	for (int i = 0; i < this->width; i++)
		if (occupancyGrid[i])
			free(occupancyGrid[i]);

	if (occupancyGrid)
		free(occupancyGrid);
}

