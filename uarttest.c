#include <stdio.h>

int main()
{
	char buf;
	FILE * uart;
	uart = fopen("/dev/serial0", "r+");

	while(1)
	{
		if(fgets(buf, 1, uart) != NULL)
		{
			printf("%c", buf);
		}
	}

	return 0;
}
