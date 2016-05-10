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
	Wind windSamples[Bout::MAX_NUM_SAMPLES];
	int windIndex;
public:
	WindAvg();
	void resetSamples();
	Wind getWindAverage();
	void addSample(Wind sample);
	virtual ~WindAvg();
};

#endif /* REGRESSION_SRC_WINDAVG_H_ */
