#include "wnd_remote.h"
#include "comm.h"
#include "resource.h"
#include <CommCtrl.h>
#include <stdio.h>
#include <float.h>
#include "../../common/vector.h"
#include "OwnerDraw.h"
#include "common.h"

extern Comm test;

static HWND hWnd;
#define PI 3.1415926
#define PI_RATIO 1.1

static float default_pid_factor[3][4] = 			// pid_factor[roll,pitch,yaw][p,i,d,i_limit]
{
	{0.50, 0.375, 0.05, PI,},
	{0.50, 0.375, 0.05, PI,},
	{1.75, 0.25, 0.01, PI,},
};

static float default_pid_factor2[3][4] = 			// pid_factor2[roll,pitch,yaw][p,i,d,i_limit]
{
	{6, 0, 0.12, PI/45,},
	{6, 0, 0.12, PI/45,},
	{8, 0, 0.23, PI/45,},
};

static float pid_factor[3][4];
static float pid_factor2[3][4];
static float p_rate_climb;
static float default_p_rate_climb = 4.5f;

#ifdef WIN32
#define isnan _isnan
#endif

static char name_table[25][5] =
{
	"rP1", "rI1", "rD1", "rM1",
	"rP2", "rI2", "rD2", "rM2",
	"rP3", "rI3", "rD3", "rM3",

	"sP1", "sI1", "sD1", "sM1",
	"sP2", "sI2", "sD2", "sM2",
	"sP3", "sI3", "sD3", "sM3",

	"cliP",
};

static float *variable_table[] = 
{
	&pid_factor[0][0], &pid_factor[0][1], &pid_factor[0][2], &pid_factor[0][3],
	&pid_factor[1][0], &pid_factor[1][1], &pid_factor[1][2], &pid_factor[1][3],
	&pid_factor[2][0], &pid_factor[2][1], &pid_factor[2][2], &pid_factor[2][3],

	&pid_factor2[0][0], &pid_factor2[0][1], &pid_factor2[0][2], &pid_factor2[0][3],
	&pid_factor2[1][0], &pid_factor2[1][1], &pid_factor2[1][2], &pid_factor2[1][3],
	&pid_factor2[2][0], &pid_factor2[2][1], &pid_factor2[2][2], &pid_factor2[2][3],

	&p_rate_climb,
};

static float factor_display[10];		// rate all roll, pitch, yaw, stabilize all roll, pitch, yaw, rate D roll, pitch, yaw,  climb rate P
DWORD factor_editbox_id[] = 
{
	IDC_RATE_0, IDC_RATE_1, IDC_RATE_2,
	IDC_STABLIZE_0, IDC_STABLIZE_1, IDC_STABLIZE_2,
	IDC_RATE_D0, IDC_RATE_D1, IDC_RATE_D2,
	IDC_VERTICAL,
};

int read_editbox()
{
	for(int i=0; i<sizeof(factor_display)/sizeof(factor_display[0]); i++)
	{
		wchar_t tmp[200] = {0};
		GetDlgItemTextW(hWnd, factor_editbox_id[i], tmp, 199);

		factor_display[i] = _wtoi(tmp)/100.0f;
	}
	return 0;
}

bool app_writing_editbox = false;
int write_editbox()
{
	app_writing_editbox = true;
	for(int i=0; i<sizeof(factor_display)/sizeof(factor_display[0]); i++)
	{
		wchar_t tmp[200] = {0};

		swprintf(tmp, L"%d", int(factor_display[i]*100+0.5));

		SetDlgItemTextW(hWnd, factor_editbox_id[i], tmp);
	}
	app_writing_editbox = false;
	return 0;
}

int read_pid()
{
	char cmd[200];
	char output[20480] = {0};
	for(int i=0; i<25; i++)
	{
		memset(output, 0, sizeof(output));
		sprintf(cmd, "?%s\n", name_table[i]);
		if (test.command(cmd, strlen(cmd), output, sizeof(output)) <= 0)
			return -1;

		if (sscanf(output, "%f", variable_table[i]) != 1)
			return -2;
	}

	// update display values
	for(int i=0; i<3; i++)
	{
		factor_display[i] = pid_factor[i][0] / default_pid_factor[i][0];
		factor_display[i+3] = pid_factor2[i][0] / default_pid_factor2[i][0];
		factor_display[i+6] = pid_factor[i][2] / default_pid_factor[i][2] / factor_display[i];
	}
	factor_display[9] = p_rate_climb / default_p_rate_climb;

	write_editbox();

	return 0;
}

int write_pid()
{
	read_editbox();

	// update display values into pid
	for(int i=0; i<3; i++)
	{
		pid_factor[i][0] = factor_display[i] * default_pid_factor[i][0];
		pid_factor[i][1] = pid_factor[i][0] * PI_RATIO;
		pid_factor[i][2] = factor_display[i+6] * factor_display[i] * default_pid_factor[i][2];
		pid_factor2[i][0] = factor_display[i+3] * default_pid_factor2[i][0];
		pid_factor2[i][1] = factor_display[i+3] * default_pid_factor2[i][1];
		pid_factor2[i][2] = factor_display[i+3] * default_pid_factor2[i][2];
	}
	p_rate_climb = factor_display[9] * default_p_rate_climb;

	char cmd[200];
	char output[20480] = {0};
	for(int i=0; i<25; i++)
	{
		memset(output, 0, sizeof(output));
		sprintf(cmd, "%s=%f\n", name_table[i], *variable_table[i]);
		if (test.command(cmd, strlen(cmd), output, sizeof(output)) <= 0)
			return -1;

		if (strstr(output, "ok") != output)
			return -2;
	}

	return 0;
}
int pid_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
		read_pid();

	return 0;
}

INT_PTR CALLBACK WndProcPid(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	HANDLE_CTLCOLORSTATIC;
	case WM_LBUTTONDOWN:
		SendMessage(GetParent(GetParent(hWnd)), WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_INITDIALOG:
		::hWnd = hWnd;
		test.add_callback(pid_OnEvent);
		read_pid();
		break;

	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;

	case WM_COMMAND:
		if (HIWORD(wParam) == EN_CHANGE && !app_writing_editbox)
			write_pid();
		break;

	case WM_PAINT:
		return paint_white(hWnd, wParam, lParam);
		break;


	default:
		return FALSE;
	}
	return TRUE;
}