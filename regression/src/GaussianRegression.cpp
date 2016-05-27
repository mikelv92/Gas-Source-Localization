/*
 * GaussianRegression.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/GaussianRegression.h"

GaussianRegression::GaussianRegression()
{
	K = MatrixXd(0, 0);
}

void GaussianRegression::addMeasurement(Position x_prime, int boutCount)
{
	X.push_back(x_prime);

	addElementToVector(&y, boutCount);

	VectorXd cv;
	RowVectorXd rv;
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
	{
		addElementToVector(&cv, kernel->getK(*x, x_prime));
		addElementToVector(&rv, kernel->getK(x_prime, *x));
	}

	K.conservativeResize(K.rows() + 1, K.cols() + 1);
	K.row(K.rows() - 1) = rv;
	K.col(K.cols() - 1) = cv;
}

double GaussianRegression::mean(Position x_star)
{
	int datasetSize = X.size();

	RowVectorXd k_x_xstar;

	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	return (k_x_xstar * v.inverse() * y)(0);
}

double GaussianRegression::variance(Position x_star)
{
	int datasetSize = X.size();

	double k_star = kernel->getK(x_star, x_star);

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	RowVectorXd k_x_xstar;
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		addElementToVector(&k_x_xstar, kernel->getK(*x, x_star));

	return k_star - (k_x_xstar * v.inverse() * k_x_xstar.transpose())(0);
}

Position GaussianRegression::nextBestPosition()
{
	map<Position, double> meanMap;
	map<Position, double> varianceMap;

	for (float i = 0; i < ENV_X; i += STEP_SIZE)
		for (float j = 0; j < ENV_Y; j += STEP_SIZE)
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

	printf("Max mean: %f, %f: %lf\n", maxMeanPos.getX(), maxMeanPos.getY(), maxMean);

	Position maxVariancePos = varianceMap.begin()->first;
	double maxVariance = 0;

	for (map<Position, double>::iterator it = varianceMap.begin(); it != varianceMap.end(); it++)
		if (it->second > maxVariance)
		{
			maxVariancePos = it->first;
			maxVariance = it->second;
		}

	//TODO For now don't consider the variance. Maybe can use MCDM in the future?
	return maxMeanPos;
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

void GaussianRegression::addElementToVector(VectorXd * vector, double element)
{
	vector->conservativeResize(vector->rows() + 1);
	RowVectorXd vec(1);
	vec << element;
	vector->row(vector->rows() - 1) = vec;
}

void GaussianRegression::addElementToVector(RowVectorXd * vector, double element)
{
	vector->conservativeResize(vector->cols() + 1);
	VectorXd vec(1);
	vec << element;
	vector->col(vector->cols() - 1) = vec;
}

void GaussianRegression::writeMeanMap(FILE * logFile)
{
	printf("Mean map\n");

	for (float i = 0; i < ENV_X; i += STEP_SIZE)
	{
		for (float j = 0; j < ENV_Y; j += STEP_SIZE)
		{
			Position x(i, j, 3);
			if (!isExplored(x))
				printf("%lf ", mean(x));
			else
			{
				int index = 0;
				for (list<Position>::iterator x_it = X.begin(); x_it != X.end(); x_it++)
				{
					if (x.equals(*x_it))
						printf("%lf ", y.row(index)(0));
					index++;
				}
			}
		}
		printf("\n");
	}
}
