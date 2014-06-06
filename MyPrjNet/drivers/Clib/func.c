/*
 * func.c
 *
 *  Created on: 2014年2月5日
 *      Author: Administrator
 */
#include <includes.h>
#define in_range(c, lo, up)  ((unsigned char)c >= lo && (unsigned char)c <= up)
#define isdigit(c)           in_range(c, '0', '9')

int str2num(const char* s)
{
	char c;
	float num = 0;

	c = *s;

	for (;;)
	{
		if (isdigit(c))
		{
			num = (num * 10) + (int) (c - '0');
		}
		else if (c == '\0' || c == '\n' || c == '.')
			break;
		c = *++s;
	}
		return num;
}

/*
 * 若返回值为0 	表示文件已读完
 * 若返回值为>0	表示读取的字符串的长度
 */

int16_t f_gets (int16_t fd,	uint8_t* buff)
{
	int n =0;
	uint8_t c, *p = buff;
	uint8_t s[2];
	uint8_t rc;

	while (1)
	{ /* Read bytes until buffer gets filled */
		rc = read(fd, s, 1);
		if (rc != 1) break; /* Break on EOF or error */
		c = s[0];
		*p++ = c;
		n++;
		if (c == '\n') break; /* Break on EOL */
	}
	*p = 0;
	return n; /* When no data read (eof or error), return with error. */
}
