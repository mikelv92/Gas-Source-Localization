/*
 * GaussianRegression.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/GaussianRegression.h"

#define DEBUG

GaussianRegression::GaussianRegression()
{
	K = MatrixXd(0, 0);
	alpha = INIT_ALPHA;
	currentPosition = Position(0, 0);
	meanAngle = 0;
	varAngle = 0;
}

void GaussianRegression::addMeasurement(Position x_prime, double value)
{
	X.push_back(x_prime);

	Utilities::addElementToVector(&y, value);

	VectorXd cv;
	RowVectorXd rv;
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
	{
		Utilities::addElementToVector(&cv, kernel->getK(*x, x_prime));
		Utilities::addElementToVector(&rv, kernel->getK(x_prime, *x));
	}

	K.conservativeResize(K.rows() + 1, K.cols() + 1);
	K.row(K.rows() - 1) = rv;
	K.col(K.cols() - 1) = cv;

	globalMeanMap[x_prime] = value;

}

double GaussianRegression::mean(Position x_star)
{
	if (gmap->isOccupied(x_star))
		return 0;

	int datasetSize = X.size();

	RowVectorXd k_x_xstar;

	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		Utilities::addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	MatrixXd I_n 	= MatrixXd::Identity(datasetSize, datasetSize);
	I_n 			*= SIGMA_N;
	MatrixXd v 		= K + I_n;

	return (k_x_xstar * v.inverse() * y)(0);
}

double GaussianRegression::variance(Position x_star)
{
	if (gmap->isOccupied(x_star))
		return 0;

	int datasetSize = X.size();

	double k_star 	= kernel->getK(x_star, x_star);

	MatrixXd I_n 	= MatrixXd::Identity(datasetSize, datasetSize);
	I_n 			*= SIGMA_N;
	MatrixXd v 		= K + I_n;

	RowVectorXd k_x_xstar;
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		Utilities::addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	return k_star - (k_x_xstar * v.inverse() * k_x_xstar.transpose())(0);
}

Position GaussianRegression::nextBestPosition()
{
	Position maxMeanPos = getMaxMeanPos();
	Position maxVariancePos = getMaxVariancePos();

	return updateCurrentPosition(maxMeanPos, maxVariancePos);
}

Position GaussianRegression::getMaxMeanPos()
{
	map<Position, double> meanMap;

	for (float i = currentPosition.getX() - EXPLORE_X; i < currentPosition.getX() + EXPLORE_X; i += STEP_SIZE)
		for (float j = currentPosition.getY() - EXPLORE_Y; j < currentPosition.getY() + EXPLORE_Y; j += STEP_SIZE)
		{
			Position x(i, j);
			if (gmap->isWithinBounds(x))
				if (!isExplored(x))
				{
					double m = mean(x);
					meanMap[x] = m;
					globalMeanMap[x] = m;
				}
		}

	Position maxMeanPos = meanMap.begin()->first;
	double maxMean = 0;

	for (map<Position, double>::iterator it = meanMap.begin(); it != meanMap.end(); it++)
		if (it->second > maxMean)
		{
			maxMeanPos 	= it->first;
			maxMean 	= it->second;
		}

	return maxMeanPos;

}

Position GaussianRegression::getMaxVariancePos()
{
	map<Position, double> varianceMap;

	for (float i = currentPosition.getX() - EXPLORE_X; i < currentPosition.getX() + EXPLORE_X; i += STEP_SIZE)
		for (float j = currentPosition.getY() - EXPLORE_Y; j < currentPosition.getY() + EXPLORE_Y; j += STEP_SIZE)
		{
			Position x(i, j);
			if (gmap->isWithinBounds(x))
			{
				double v = variance(x);
				varianceMap[x] = v;
				globalVarianceMap[x] = v;
			}
		}

	Position maxVariancePos = varianceMap.begin()->first;
	double maxVariance = 0;

	for (map<Position, double>::iterator it = varianceMap.begin(); it != varianceMap.end(); it++)
		if (it->second > maxVariance)
		{
			maxVariancePos 	= it->first;
			maxVariance 	= it->second;
		}

	return maxVariancePos;
}

Position GaussianRegression::updateCurrentPosition(Position meanPos, Position varPos)
{
	meanAngle 	= atan2(meanPos.getY(), meanPos.getX());
	varAngle 	= atan2(varPos.getY(), varPos.getX());

	double angle;

	if (((double)rand() / (double)RAND_MAX) > alpha)
	{
		mt19937 gen;
		normal_distribution<double> distribution(meanAngle, MEAN_GAUSS_VARIANCE);
		variate_generator<mt19937&, normal_distribution<double> > var_nor(gen, distribution);
		angle = var_nor();
	}
	else
	{
		mt19937 gen;
		normal_distribution<double> distribution(varAngle, VAR_GAUSS_VARIANCE);
		variate_generator<mt19937&, normal_distribution<double> > var_nor(gen, distribution);
		angle = var_nor();
	}


	float old_x = currentPosition.getX();
	float old_y = currentPosition.getY();

	float new_pos_x = old_x + RHO * cos(angle);
	float new_pos_y = old_y + RHO * sin(angle);

	alpha *= 0.99;

	if (alpha < ALPHA_THRESHOLD)
	{
		printMeanMap();
		printVarianceMap();
		return finalPosition();
	}
	else
	{

		Position candidatePosition(new_pos_x, new_pos_y);
		Position newPosition(0, 0);

		//Check if there are any obstacles and get the nearest free cell to the candidate
		if (!gmap->isOccupied(candidatePosition) && !isExplored(candidatePosition))
			return candidatePosition;
		else
			newPosition = updatePosToNearestFreeCell(candidatePosition);

		newPosition.setOrientation(atan2(newPosition.getY() - old_y, newPosition.getX() - old_x));

		if (newPosition.isNullPos())
			printf("Error: Couldn't find a free cell to go to. Going to starting position\n");

		return newPosition;
	}

}

Position GaussianRegression::updatePosToNearestFreeCell(Position position)
{
	int x = position.getX();
	int y = position.getY();

	printf("Chosen position occupancy value: %d %d %d\n", x, y, gmap->getOccupancyValue(position));

	for (int k = 1; gmap->isWithinBoundsX(x + k) || gmap->isWithinBoundsY(y + k); k++)
		for (int i = -k; i <= k; i += k)
		{
			if (gmap->isWithinBoundsX(x + i))
				position.setX(x + i);

			for (int j = -k; j <= k; j += k)
			{
				if (gmap->isWithinBoundsY(y + j))
					position.setY(y + j);

				printf("x,y,k, occupancy val, is explored, is occupied:%lf %lf %d: %d %c, %c\n",
						position.getX(),
						position.getY(),
						k,
						gmap->getOccupancyValue(position),
						isExplored(position) ? 'e' : 'f',
								gmap->isOccupied(position) ? 'o' : 'f');

				if (!gmap->isOccupied(position) && !isExplored(position))
					return position;
			}
		}
	return Position(true);
}

bool GaussianRegression::isExplored(Position x_new)
{
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		if (x_new.equals(*x))
			return true;
	return false;
}

void GaussianRegression::setKernel(KernelFunction * kernelFunction)
{
	this->kernel = kernelFunction;
}

void GaussianRegression::setGMap(GMap * gmap)
{
	this->gmap = gmap;
}

void GaussianRegression::printMeanMap()
{
	printf("Printing mean map...\n");
	FILE * file = fopen("meanMap.pgm", "w");

	fprintf(file, "P2\n%d\n%d\n100\n", gmap->getWidth(), gmap->getHeight());

	bool existsInMap = false;

	for (int j = gmap->getHeight() - 1; j >= 0; j--)
	{
		for (int i = 0; i < gmap->getWidth(); i++)
		{
			for (map<Position, double>::iterator it = globalMeanMap.begin(); it != globalMeanMap.end(); it++)
				if (it->first.getX() == i && it->first.getY() == j)
				{
					existsInMap = true;
					fprintf(file, "%lf ", it->second);
				}
			if (!existsInMap)
				fprintf(file, "-1 ");
		}
		fprintf(file, "\n");
	}

	fclose(file);
	printf("Done.\n");

}

void GaussianRegression::printVarianceMap()
{
	printf("Printing variance map...\n");
	FILE * file = fopen("varianceMap.pgm", "w");

	fprintf(file, "P2\n%d\n%d\n100\n", gmap->getWidth(), gmap->getHeight());

	bool existsInMap = false;

	for (int j = gmap->getHeight() - 1; j >= 0; j--)
	{
		for (int i = 0; i < gmap->getWidth(); i++)
		{
			for (map<Position, double>::iterator it = globalVarianceMap.begin(); it != globalVarianceMap.end(); it++)
				if (it->first.getX() == i && it->first.getY() == j)
				{
					existsInMap = true;
					fprintf(file, "%lf ", it->second);
				}
			if (!existsInMap)
				fprintf(file, "-1 ");
		}
		fprintf(file, "\n");
	}

	fclose(file);
	printf("Done.\n");
}

Position GaussianRegression::finalPosition()
{
	double max = 0;
	Position maxPos = globalMeanMap.begin()->first;
	for (map<Position, double>::iterator it = globalMeanMap.begin(); it != globalMeanMap.end(); it++)
		if (it->second > max)
		{
			maxPos 	= it->first;
			max 	= it->second;
		}
	return maxPos;
}

double GaussianRegression::meanDirGaussF(double theta)
{
	return exp((-1 * (theta - meanAngle) * (theta - meanAngle)) / (MEAN_GAUSS_VARIANCE * MEAN_GAUSS_VARIANCE));
}

double GaussianRegression::varDirGaussF(double theta)
{
	return exp((-1 * (theta - varAngle) * (theta - varAngle)) / (VAR_GAUSS_VARIANCE * VAR_GAUSS_VARIANCE));
}

GaussianRegression::~GaussianRegression()
{

}

