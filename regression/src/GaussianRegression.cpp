/*
 * GaussianRegression.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/GaussianRegression.h"

GaussianRegression::GaussianRegression(KernelFunction kernelFunction)
{
	kernel = kernelFunction;
	K = MatrixXd(0, 0);
}

void GaussianRegression::addMeasurement(Position x_prime, int boutCount)
{
	X.push_back(x_prime);
	y.conservativeResize(y.rows() + 1);
	y << boutCount;

	int datasetSize = X.size();
	VectorXd cv(datasetSize);
	RowVectorXd rv(datasetSize);
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
	{
		cv << kernel.getK(*x, x_prime);
		rv << kernel.getK(x_prime, *x);
	}

	K.conservativeResize(K.rows() + 1, K.cols() + 1);
	K.row(K.rows() - 1) = rv;
	K.col(K.cols() - 1) = cv;

}

double GaussianRegression::mean(Position x_star)
{
	int datasetSize = X.size();
	RowVectorXd k_x_xstar(datasetSize);
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		k_x_xstar << kernel.getK(*x, x_star);

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	return (k_x_xstar * v.inverse() * y)(0);
}

double GaussianRegression::variance(Position x_star)
{
	int datasetSize = X.size();

	double k_star = kernel.getK(x_star, x_star);

	MatrixXd I_n = MatrixXd::Identity(datasetSize, datasetSize);
	I_n *= SIGMA_N;
	MatrixXd v = K + I_n;

	RowVectorXd k_x_xstar(datasetSize);
	for (list<Position>::iterator x = X.begin(); x != X.end(); x++)
		k_x_xstar << kernel.getK(*x, x_star);

	return k_star - (k_x_xstar * v.inverse() * k_x_xstar.transpose())(0);
}

GaussianRegression::~GaussianRegression()
{
}
