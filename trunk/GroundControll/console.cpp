#include <stdio.h>
#include <conio.h>
#include "comm.h"


int console_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
	{
		printf("---pilot connected---\n\n");
	}
	if (code == WM_DISCONNECT)
	{
		printf("---pilot disconnected---\n\n");
	}

	return 0;
}

int main()
{
	Comm con;
	con.add_callback(console_OnEvent);

	int p = 0;
	char input[1024];
	char output[20480];

	while(true)
	{
		fgets(input, 1024, stdin);
		p = strlen(input);
		int count = con.command(input, p, output);
		p = 0;

		if (count > 0)
		{
			for(int i=0; i<count; i++)
				fputc(output[i], stdout);
			fputc('\n', stdout);
		}
		else
		{
			printf("error:%d\n", count);
		}
	}
}