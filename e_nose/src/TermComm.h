/*
 * ENoseTermComm.h
 *
 *  Created on: Apr 4, 2016
 *      Author: mikel
 */

#ifndef ENOSETERMCOMM_H_
#define ENOSETERMCOMM_H_

#include <termios.h>
#include <poll.h>
#include <exception>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <strings.h>
#include <cstdio>

#include "CharCircularBuffer.h"


class SerialDeviceException: public std::exception
{
public:
	virtual const char* what() const throw();
};

class TermComm {
private:
	int fd;
	termios oldterm, newterm;
	struct pollfd ufd[1];
	char* tmp_buf;
	CharCircularBuffer* buffer;
	static const int TMP_BUF_SIZE = 512;

	static const int wait_ok=1;
	static const int wait_tout=0;
	static const int wait_err=-1;


	int waitData(int);


public:
	TermComm(const char*) throw (SerialDeviceException);

	bool isReady();
	int readData();
	std::string getLine();
	int sendCommand(const char *cmd,int len);
	unsigned int getLineCount();

	virtual ~TermComm();
};

#endif /* ENOSETERMCOMM_H_ */
