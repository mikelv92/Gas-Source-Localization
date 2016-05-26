/*
 * Wind.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_WIND_H_
#define REGRESSION_SRC_WIND_H_

#include <cmath>

class Wind {
private:
	double speed2D;
	double u;
	double w;
	double v;

public:
	Wind();
	Wind(double u, double v, double w);

	double get2DSpeed() const {
		return speed2D;
	}

	void set2DSpeed(double speed)
	{
		this->speed2D = speed;
	}

	double getU() const {
		return u;
	}

	void setU(double u) {
		this->u = u;
	}

	double getV() const {
		return v;
	}

	void setV(double v) {
		this->v = v;
	}

	double getW() const {
		return w;
	}

	void setW(double w) {
		this->w = w;
	}
};

#endif /* REGRESSION_SRC_WIND_H_ */
