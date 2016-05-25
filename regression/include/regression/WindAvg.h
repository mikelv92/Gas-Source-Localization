/*
 * WindAvg.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_WINDAVG_H_
#define REGRESSION_SRC_WINDAVG_H_
#include "regression/Wind.h"
#include "regression/Bout.h"

#include <stdio.h>

class WindAvg {
private:
	Wind windSamples[Bout::SIGNAL_LEN];
	int windIndex;
public:
	WindAvg();
	void resetSamples();
	Wind getWindAverage();
	void addSample(Wind sample);
	~WindAvg();
};

#endif /* REGRESSION_SRC_WINDAVG_H_ */
