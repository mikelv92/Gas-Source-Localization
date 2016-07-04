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
	alpha = 1;
	currentPosition = Position(0, 0, 0);
	meanAngle = 0;
	varAngle = 0;

	logFile = fopen("regressionLogs", "w");
}

void GaussianRegression::addMeasurement(Position x_prime, int boutCount)
{
	X.push_back(x_prime);

	Utilities::addElementToVector(&y, boutCount);

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
}

double GaussianRegression::mean(Position x_star)
{
	if (gmap->isOccupied(x_star))
		return 0;

	int datasetSize = X.size();

	RowVectorXd k_x_xstar;

	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		Utilities::addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	return (k_x_xstar * v.inverse() * y)(0);
}

double GaussianRegression::variance(Position x_star)
{
	if (gmap->isOccupied(x_star))
		return 0;

	int datasetSize = X.size();

	double k_star = kernel->getK(x_star, x_star);

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	RowVectorXd k_x_xstar;
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		Utilities::addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	return k_star - (k_x_xstar * v.inverse() * k_x_xstar.transpose())(0);
}

Position GaussianRegression::nextBestPosition()
{
	map<Position, double> meanMap;
	map<Position, double> varianceMap;

	for (float i = gmap->getOrigin().getX(); i < gmap->getOrigin().getX() + gmap->getWidth(); i += STEP_SIZE)
		for (float j = gmap->getOrigin().getY(); j < gmap->getOrigin().getY() + gmap->getWidth(); j += STEP_SIZE)
		{
			Position x(i, j, 3);

			if (!isExplored(x))
			{
				meanMap[x] = mean(x);
				varianceMap[x] = variance(x);
			}
		}

	Position maxMeanPos = meanMap.begin()->first;
	double maxMean = 0;

	for (map<Position, double>::iterator it = meanMap.begin(); it != meanMap.end(); it++)
		if (it->second > maxMean)
		{
			maxMeanPos = it->first;
			maxMean = it->second;
		}

	Position maxVariancePos = varianceMap.begin()->first;
	double maxVariance = 0;

	for (map<Position, double>::iterator it = varianceMap.begin(); it != varianceMap.end(); it++)
		if (it->second > maxVariance)
		{
			maxVariancePos = it->first;
			maxVariance = it->second;
		}

	return updateCurrentPosition(maxMeanPos, maxVariancePos);
}

Position GaussianRegression::updateCurrentPosition(Position meanPos, Position varPos)
{
	double dot = 0, det = 0;

	//angle of meanPos wrt x axis [1 0]
	dot 				= meanPos.getX() * 1 + meanPos.getY() * 0;
	det 				= meanPos.getX() * 0 - meanPos.getY() * 1;
	meanAngle 	= atan2(det, dot);

	//angle of varPos wrt x axis [1 0]
	dot 				= varPos.getX() * 1 + varPos.getY() * 0;
	det 				= varPos.getX() * 0 - varPos.getY() * 1;
	varAngle 	= atan2(det, dot);

	double angle;

	if ((double)(rand() / RAND_MAX) > alpha)
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

	int new_pos_x = floor(currentPosition.getX() + RHO * cos((double)angle * M_PI / 180));
	int new_pos_y = floor(currentPosition.getY() + RHO * sin((double)angle * M_PI / 180));

	alpha *= 0.99;

	if (alpha < ALPHA_THRESHOLD)
		return meanPos;
	else
	{
		int old_x = currentPosition.getX();
		int old_y = currentPosition.getY();

		currentPosition.setX(new_pos_x);
		currentPosition.setY(new_pos_y);

		updatePosToNearestFreeCell(&currentPosition);
		currentPosition.setOrientation(atan2(currentPosition.getY() - old_y, currentPosition.getX() - old_x));

#ifdef DEBUG
		printf("Wind direction: %f\n", kernel->getWind().getDirection());
		printf("Wind speed: %f\n", kernel->getWind().getSpeed());

		fprintf(logFile, "Wind direction: %f\n", kernel->getWind().getDirection());
		fprintf(logFile, "Wind speed: %f\n", kernel->getWind().getSpeed());

		printMeanMap();
		printVarianceMap();

		fprintf(logFile, "Initial new pos: %f %f\n", new_pos_x, new_pos_y);
		fprintf(logFile, "After checking obstacles: %f %f\n", currentPosition.getX(), currentPosition.getY());

		printf("Initial new pos: %f %f\n", new_pos_x, new_pos_y);
		printf("After checking obstacles: %f %f\n", currentPosition.getX(), currentPosition.getY());
#endif


		return currentPosition;
	}

}

void GaussianRegression::updatePosToNearestFreeCell(Position * position)
{
	int x = position->getX();
	int y = position->getY();
	Position origin = gmap->getOrigin();

	for (int k = 0; x + k < gmap->getWidth() || y + k < gmap->getHeight(); k++)
		for (int i = -k; i <= k; i += k)
		{
			if (x + k < origin.getX() + gmap->getWidth() && x + k >= origin.getX())
				position->setX(x + k);

			for (int j = -k; j <= k; j += k)
			{
				if (y + k < origin.getY() + gmap->getHeight() && y + k >= origin.getY())
					position->setY(y + k);
				if (!gmap->isOccupied(*position) && !isExplored(*position))
					return;
			}
		}
}

bool GaussianRegression::isExplored(Position x_new)
{
	printf("x_new: %f %f\n", x_new.getX(), x_new.getY());
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
	{
		printf("x: %f %f\n", x->getX(), x->getY());
		if (x_new.equals(*x))
			return true;
		printf("After if\n");
	}
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

double GaussianRegression::meanDirGaussF(double theta)
{
	return exp((-1 * (theta - meanAngle) * (theta - meanAngle)) / (MEAN_GAUSS_VARIANCE * MEAN_GAUSS_VARIANCE));
}

double GaussianRegression::varDirGaussF(double theta)
{
	return exp((-1 * (theta - varAngle) * (theta - varAngle)) / (VAR_GAUSS_VARIANCE * VAR_GAUSS_VARIANCE));
}



void GaussianRegression::printMeanMap()
{
	fprintf(logFile, "Mean map\n");

	for (float i = gmap->getOrigin().getX(); i < gmap->getOrigin().getX() + gmap->getWidth(); i += STEP_SIZE)
	{
		for (float j = gmap->getOrigin().getY(); j < gmap->getOrigin().getY() + gmap->getHeight(); j += STEP_SIZE)
		{
			Position x(i, j);
			if (!isExplored(x))
				fprintf(logFile, "%lf,", mean(x));
			else
			{
				int index = 0;
				for (list<Position>::iterator x_it = X.begin(); x_it != X.end(); x_it++)
				{
					if (x.equals(*x_it))
						fprintf(logFile, "%lf,", y.row(index)(0));
					index++;
				}
			}
		}
		fprintf(logFile, "\n");
	}
	fprintf(logFile, "\n\n");

}

void GaussianRegression::printVarianceMap()
{
	fprintf(logFile, "Variance map\n");

	for (float i = gmap->getOrigin().getX(); i < gmap->getOrigin().getX() + gmap->getWidth(); i += STEP_SIZE)
	{
		for (float j = gmap->getOrigin().getY(); j < gmap->getOrigin().getY() + gmap->getHeight(); j += STEP_SIZE)
		{
			Position x(i, j);
			fprintf(logFile, "%lf,", variance(x));
		}
		fprintf(logFile, "\n");
	}
	fprintf(logFile, "\n\n");
}

GaussianRegression::~GaussianRegression()
{
	if (logFile)
		fclose(logFile);
}

