#include <stdio.h>
#include <conio.h>
#include "comm.h"
#include "../common/space.h"

static unsigned long pnan[2]={0xffffffff, 0x7fffffff};
static double NAN = *( double* )pnan;

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

int spaceiotest()
{
	space_init(true);
	for(int i=0; i<102400; i++)
	{
		float test[30] = {4.8f};
		float read[30] = {0};
	space_write("hi", 2, test, sizeof(test), NULL);
	space_read("hi", 2, read, sizeof(read), NULL);

	if (memcmp(test, read, sizeof(test)))
		break;
	}

	return 0;
}

int main()
{
	spaceiotest();

	Comm con;
	con.add_callback(console_OnEvent);

	int p = 0;
	char input[1024];
	char output[20480];

	Sleep(2000);

	for(int i=1; i<10000; i++)
	{
		float vt = -float(i)/1000;
		while(con.write_float("altP", vt)<0)
			;

		float r = 0;
		while(con.read_float("altP", &r)<0)
			;

		if (abs(r-vt)>0.0001f)
		{
			printf("(ERROR)");
		}
		Sleep(10);

		printf("%d=%f\n", i, r);
	}

	char test[5];
	float v;
	con.enum_float(2, test, &v);

	while(true)
	{
		memset(input, 0, sizeof(input));
		memset(output, 0, sizeof(output));
		fgets(input, 1024, stdin);
		p = strlen(input);
		int count = con.command(input, p, output, sizeof(output));
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