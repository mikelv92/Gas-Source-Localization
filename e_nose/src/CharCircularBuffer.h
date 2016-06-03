#ifndef CharCircularBuffer_h_
#define CharCircularBuffer_h_

#include <cstdio>
#include <string.h>
#include <string>


class CharCircularBuffer
{
	char * buf;
	unsigned int start,end;
	
	char * end_line_chars;
	int end_line_chars_num;
	
	unsigned int n;
	unsigned int lineCount;
	
	inline unsigned int inc(unsigned int v)
	{
		return (v+1==n?0:v+1);
	};
	inline unsigned int dec(unsigned int v)
	{
		return (v==0?n-1:v-1);
	};
	
	int isEndLine(char c);
	
	public:
		CharCircularBuffer(unsigned int n,char end_line_char='\n');
		CharCircularBuffer(unsigned int n,char * end_line_chars);
		~CharCircularBuffer();

		inline unsigned int getCount()
		{
				return (end>=start?end-start:n-start+end);
		}
		
		unsigned int getLineCount();
		
		int addChar(char src);
		int removeChar(char *dest);

		inline void reset()
		{
			start=end=0;
		};

		int addNChar(char *src,unsigned int n);
		int removeNChar(char *dest,unsigned int n);

		int removeLine(char *dest,unsigned int maxn);
		
		inline bool isFull()
		{
			return inc(end)==start;
		};
		inline bool isEmpty()
		{
			return start==end;
		};

};

#endif
