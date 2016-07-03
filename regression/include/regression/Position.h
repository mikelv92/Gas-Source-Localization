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
	float orientation; //radians
public:
	Position() {};
	Position(float x, float y, float z = 0) : x(x), y(y), z(z) {};

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

	bool equals(Position pos)
	{
		return this->x == pos.getX() && this->y == pos.getY() && this->z == pos.getZ();
	}

	inline bool operator<(const Position & pos) const
	{
		return this->getX() < pos.getX() ||
				(this->getX() == pos.getX() && this->getY() < pos.getY()) ||
				(this->getX() == pos.getX() && this->getY() == pos.getY() && this->getZ() < pos.getZ());
	}

	float getOrientation() const {
		return orientation;
	}

	void setOrientation(float orientation) {
		this->orientation = orientation;
	}
};

#endif /* REGRESSION_SRC_POSITION_H_ */
