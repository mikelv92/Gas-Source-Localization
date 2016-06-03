/*
 * ENoseTermComm.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: mikel
 */

#include "TermComm.h"
#include <errno.h>
#include <iostream>
#include <sstream>


#define BAUDRATE 		B9600
#define TOUT			300 //msec
#define MAX_C_RECV 		40
#define MAX_BUF_SIZE 	30000
#define CHAR_PAUSE		3000 //usec


using namespace std;

const char* SerialDeviceException::what() const throw()
{
  return "Serial port open error!";
}

TermComm::TermComm(const char* serialFilename) throw (SerialDeviceException)
{
	fd = open(serialFilename, O_RDWR | O_NOCTTY);
	if (fd >= 0)
	{
		tcgetattr(fd, &oldterm);
		bzero(&newterm, sizeof(termios));
		newterm.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD | CLOCAL;
		newterm.c_iflag = 0;
		newterm.c_oflag = 0;
		newterm.c_lflag &= ~ICANON;

		//set fd
		ufd[0].fd = fd;
		ufd[0].events = POLLIN;

		newterm.c_cc[VTIME] = 0;
		newterm.c_cc[VMIN] 	= 1;

		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newterm);

	}
	else
		throw SerialDeviceException();
//		cout << "Bad things happened." << endl;

	tmp_buf = new char[TMP_BUF_SIZE];
	buffer = new CharCircularBuffer(MAX_BUF_SIZE, '\r');
}

TermComm::~TermComm() {
	if (fd >= 0)
	{
		close(fd);
	}
	if (tmp_buf)
	{
		delete[] tmp_buf;
	}
	if (buffer)
	{
		delete buffer;
	}
}

bool TermComm::isReady()
{
	return fd >= 0;
}

int TermComm::waitData(int msec_tout)
{
	bool rexec;
	int v;
	int fd = ufd[0].fd;

	do
	{
		rexec = false;
		v = poll(ufd, 1, msec_tout);
		if (v < 0)
		{
			if (errno == EINTR)
			{
				fprintf(stderr, "poll EINTR on file %d\n", fd);
				rexec = true;
			}
			else
			{
				fprintf(stderr, "poll error on file %d,errno=%d\n", fd, errno);
				return wait_err;
			}
		}
		else if (v == 0)
		{
			return wait_tout;
		}
	} while (rexec);
	return wait_ok;
}

int TermComm::readData()
{
	if(fd<0)
	{
		return -1;
	}

	int res = 0;
	unsigned int count_c = 0;

	do {
		switch(waitData(TOUT))
		{
			case wait_ok:
				res = read(fd,tmp_buf, TMP_BUF_SIZE);
				if(res<=0)
				{
					cerr << "(!) READ Error on fd " << fd << endl;
					res=-1;
				}

				if(buffer->addNChar(tmp_buf,res)!=res)
				{
					count_c	= count_c+res;
					cerr << "(!) BUF_FULL on fd " << fd << endl;
					res=-1;
				}
				break;
			case wait_err:
				cerr << "(!) Error on fd " << fd << endl;
				res=-1;
				break;
			case wait_tout:
				cerr << "(!) TIME_OUT on fd " << fd << endl;
				res=-1;
				break;
		}

		if(res==-1)break;

	}while(count_c<MAX_C_RECV && buffer->getLineCount()<=0);

	if(res<=0)return -1;
	return 0;
}


int TermComm::sendCommand(const char *cmd,int len)
{
//	std::cerr << cmd << std::endl;
	if(fd < 0)
		return -1;
	for(int i = 0; i < len; i++)
	{
		if(write(fd, &cmd[i], 1) == -1)
		{
			return -1;
			perror("Error writing to the serial port!");
		}
		usleep(CHAR_PAUSE);
	}
	usleep(500000);
	return 0;
}

std::string TermComm::getLine()
{
	if(buffer->getLineCount()<=0) return "Error";
	int len=buffer->removeLine(tmp_buf,MAX_BUF_SIZE);
	if(len<=0)return "Error";
	if(tmp_buf[len-1]=='\n')tmp_buf[len-1]='\0';
	return tmp_buf;
}

unsigned int TermComm::getLineCount()
{
	return buffer->getLineCount();
}
