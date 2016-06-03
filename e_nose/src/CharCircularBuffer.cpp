#include "CharCircularBuffer.h"

#include <sstream>
#include <iostream>
#include <string>

CharCircularBuffer::CharCircularBuffer(unsigned int n, char end_line_char)
{
	buf = new char[n + 1];
	this->n = n + 1;
	start = 0;
	end = 0;
	lineCount = 0;
	this->end_line_chars = new char[1];
	this->end_line_chars[0] = end_line_char;
	this->end_line_chars_num = 1;
}

CharCircularBuffer::CharCircularBuffer(unsigned int n, char * end_line_chars)
{
	buf = new char[n + 1];
	this->n = n + 1;
	start = 0;
	end = 0;
	lineCount = 0;
	this->end_line_chars_num = strlen(end_line_chars);
	this->end_line_chars = new char[this->end_line_chars_num + 1];
	strcpy(this->end_line_chars, end_line_chars);
}

CharCircularBuffer::~CharCircularBuffer()
{
	delete[] buf;
	delete[] end_line_chars;
}

unsigned int CharCircularBuffer::getLineCount()
{
	return lineCount;
}

int CharCircularBuffer::addChar(char src)
{
	if (isFull())
		return 0;
	buf[end] = src;
	end = inc(end);
	if (isEndLine(src))
		lineCount++;
	return 1;
}

int CharCircularBuffer::isEndLine(char c)
{
	for (int i = 0; i < end_line_chars_num; i++)
	{
		if (c == end_line_chars[i])
			return 1;
	}
	return 0;
}

int CharCircularBuffer::removeChar(char *dest)
{
	if (isEmpty())
		return 0;
	*dest = buf[start];
	start = inc(start);
	if (isEndLine(*dest))
		lineCount--;
	return 1;
}

int CharCircularBuffer::addNChar(char *src, unsigned int nx)
{
	unsigned int disp = n - getCount() - 1;
	nx = (disp > nx ? nx : disp);
	for (unsigned int i = 0; i < nx; i++)
	{
		if (isEndLine(src[i]))
			lineCount++;
		buf[end] = src[i];
		end = inc(end);
	}
	return nx;
}

int CharCircularBuffer::removeNChar(char *dest, unsigned int nx)
{
	unsigned int disp = getCount();
	nx = (disp > nx ? nx : disp);
	for (unsigned int i = 0; i < nx; i++)
	{
		if (isEndLine(buf[start]))
			lineCount--;
		dest[i] = buf[start];
		start = inc(start);
	}
	return nx;
}

int CharCircularBuffer::removeLine(char *dest, unsigned int maxn)
{
	unsigned int c = 0;
	if (lineCount == 0)
		return 0;
	while ((!isEmpty()) && (!isEndLine(buf[start])) && (c < maxn - 1))
	{
		dest[c] = buf[start];
		start = inc(start);
		c++;
	}
	dest[c] = '\0';
	if (!isEmpty() && isEndLine(buf[start]))
	{
		lineCount--;
		start = inc(start);
	}
	return c;
}
