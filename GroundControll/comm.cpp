#include "comm.h"
#include <stdio.h>

#define HANDSHAKE_TIMEOUT 200
#define safe_close_handle(x) {if(x!=INVALID_HANDLE_VALUE)CloseHandle(x);x=INVALID_HANDLE_VALUE;}

Comm::Comm()
:m_states(-1)
,last_success_port(-1)
,port(INVALID_HANDLE_VALUE)
{
	InitializeCriticalSection(&cs);
	InitializeCriticalSection(&cs_cb);
	CreateThread(NULL, NULL, find_device_thread, this, NULL, NULL);
}

int Comm::find_device()
{
	EnterCriticalSection(&cs);
	if (m_states == 0)
	{
		LeaveCriticalSection(&cs);
		return 0;
	}

	for(int i=-1; i<48; i++)
	{

		if (i == 31)
			goto fail;

		wchar_t port_name[260];
		wsprintfW(port_name, L"\\\\.\\COM%d", i==-1?last_success_port:i);
		port = CreateFileW(port_name,
			GENERIC_READ | GENERIC_WRITE,       // Specify mode that open device.
			0,                                  // the devide isn't shared.
			NULL,                               // the object gets a default security.
			OPEN_EXISTING,                      // Specify which action to take on file. 
			FILE_FLAG_NO_BUFFERING,				// default.
			NULL);                              // default.

		if (port == INVALID_HANDLE_VALUE)
			continue;

		COMMTIMEOUTS comTimeOut;
		comTimeOut.ReadIntervalTimeout = 300;
		comTimeOut.ReadTotalTimeoutMultiplier = 3;
		comTimeOut.ReadTotalTimeoutConstant = 2;
		comTimeOut.WriteTotalTimeoutMultiplier = 3;
		comTimeOut.WriteTotalTimeoutConstant = 200;
		SetCommTimeouts(port,&comTimeOut);

		m_states = 0;
		if (hand_shake() == 0)
		{
			last_success_port = i;
			break;
		}
		else
			disconnect();
	}


	m_states = 0;
	LeaveCriticalSection(&cs);

	// send callbacks
// 	EnterCriticalSection(&cs_cb);
	{
		for(std::vector<ICommEvent*>::iterator i = callbacks1.begin(); i!= callbacks1.end(); ++i)
			(*i)->OnEvent(WM_CONNECT, NULL);
		for(std::vector<CommEventProc>::iterator i = callbacks2.begin(); i!= callbacks2.end(); ++i)
			(*i)(WM_CONNECT, NULL);
		for(std::vector<HWND>::iterator i = callbacks3.begin(); i!= callbacks3.end(); ++i)
			PostMessage(*i, WM_CONNECT, NULL, NULL);
	}
// 	LeaveCriticalSection(&cs_cb);

	return m_states;
fail:
	m_states = -1;
	LeaveCriticalSection(&cs);
	return m_states;
}

Comm::~Comm()
{
	safe_close_handle(port);
	DeleteCriticalSection(&cs);
	DeleteCriticalSection(&cs_cb);
}

int Comm::hand_shake()
{
	char cmd[] = "hello\n";
	char out[500] = {0};
	if (command(cmd, strlen(cmd), out) >= 3  && strstr(out, "yap") == out)
		return 0;

	return -1;
}

int Comm::states()
{
	return m_states;
}

DWORD CALLBACK Comm::find_device_thread(LPVOID p)
{
	Sleep(1000);
	Comm *c = (Comm*)p;
	while(true)
	{
		c->find_device();

		Sleep(400);
	}
	return 0;
}

int Comm::command(const char *input, int input_count, char*output)
{
	DWORD got=0;
	EnterCriticalSection(&cs);

	if (m_states<0)
	{
		LeaveCriticalSection(&cs);
		return m_states;
	}

	if (!WriteFile(port, input, input_count, &got, NULL) || !FlushFileBuffers(port))
	{
// 		printf("write failed\n");

		disconnect();
		LeaveCriticalSection(&cs);
		return -1;
	}

	int count = 0;
	int start = GetTickCount();
	while(true)
	{
		char p;
		if (ReadFile(port, &p, 1, &got, NULL) && got > 0)
		{
			if (p == '\r')
				continue;
			else if (p == '\n')
			{
				break;;
			}
			else
			{
				output[count++] = p;
			}

			start = GetTickCount();
		}
		else if (GetTickCount() - start > HANDSHAKE_TIMEOUT)
		{
			count = -2;
			disconnect();
			break;
		}
	}

	LeaveCriticalSection(&cs);
// 	printf("input: %s (%d bytes). output: %s (%d bytes)\n", input, input_count, output, count);
	return count;
}

int Comm::read_float(const char* id, float *out)
{
	if (!out)
		return -3;

	char cmd[200];
	char output[20480] = {0};

	memset(output, 0, sizeof(output));
	sprintf(cmd, "?%s\n", id);

	if (command(cmd, strlen(cmd), output) <= 0)
		return -1;

	if (sscanf(output, "%f", out) != 1)
		return -2;

	return 0;
}

int Comm::enum_float(int pos, char *id, float *out)
{
	if (!out || !id)
		return -3;

	char cmd[200];
	char output[20480] = {0};

	memset(output, 0, sizeof(output));
	sprintf(cmd, "!%d\n", pos);

	if (command(cmd, strlen(cmd), output) <= 0)
		return -1;

	if (strstr(output, "null") == output)
		return 1;

	const char *p = strstr(output, "=");
	if (!p)
		return -2;
	*(char*)p = NULL;
	strncpy(id, output, 4);
	id[4] = NULL;
	*out = atof(p+1);

	return 0;
}

int Comm::write_float(const char* id, float newdata)
{
	char cmd[200];
	char output[20480] = {0};
	memset(output, 0, sizeof(output));
	sprintf(cmd, "%s=%f\n", id, newdata);

	if (command(cmd, strlen(cmd), output) <= 0)
		return -1;

	if (strstr(output, "ok") != output)
		return -2;

	return 0;
}

int Comm::disconnect()
{
	if (m_states < 0)
		return 0;

	safe_close_handle(port);
	m_states = -1;

	// send callbacks
// 	EnterCriticalSection(&cs_cb);
	for(std::vector<ICommEvent*>::iterator i = callbacks1.begin(); i!= callbacks1.end(); ++i)
		(*i)->OnEvent(WM_DISCONNECT, NULL);
	for(std::vector<CommEventProc>::iterator i = callbacks2.begin(); i!= callbacks2.end(); ++i)
		(*i)(WM_DISCONNECT, NULL);
	for(std::vector<HWND>::iterator i = callbacks3.begin(); i!= callbacks3.end(); ++i)
		PostMessage(*i, WM_DISCONNECT, NULL, NULL);
// 	LeaveCriticalSection(&cs_cb);

	return 0;
}

int Comm::add_callback(ICommEvent * cb)
{
	EnterCriticalSection(&cs_cb);
	callbacks1.push_back(cb);
	LeaveCriticalSection(&cs_cb);
	return 0;
}
int Comm::add_callback(CommEventProc cb)
{
	EnterCriticalSection(&cs_cb);
	callbacks2.push_back(cb);
	LeaveCriticalSection(&cs_cb);
	return 0;
}
int Comm::add_callback(HWND cb)
{
	EnterCriticalSection(&cs_cb);
	callbacks3.push_back(cb);
	LeaveCriticalSection(&cs_cb);
	return 0;
}

