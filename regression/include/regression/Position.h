/*
 * Position.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_POSITION_H_
#define REGRESSION_SRC_POSITION_H_

class Position {
private:
	float x;
	float y;
	float z;
public:
	Position(float x, float y, float z = 1) : x(x), y(y), z(z) {};

	float getX() const {
		return x;
	}

	void setX(float x) {
		this->x = x;
	}

	float getY() const {
		return y;
	}

	void setY(float y) {
		this->y = y;
	}

	float getZ() const {
		return z;
	}

	void setZ(float z) {
		this->z = z;
	}

	Position diff(Position pos)
	{
		return Position(this->getX() - pos.getX(), this->getY() - pos.getY());
	}
};

#endif /* REGRESSION_SRC_POSITION_H_ */
