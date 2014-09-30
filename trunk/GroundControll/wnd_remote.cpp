#include "wnd_remote.h"
#include "comm.h"
#include "resource.h"
#include <CommCtrl.h>
#include <WindowsX.h>
#include <stdio.h>
#include "OwnerDraw.h"
#include "common.h"

extern Comm test;
static HWND wnd;
HWND sliders[6];
int ppm_states[8][3];
int ppm_center[8] = {0};
float rc[8];
float rc_setting[8][4];

static bool remote_calibrating = false;
static bool need_update_revert_button = false;

static float limit(float v, float low, float high)
{
	if (v<low)
		return low;
	if (v>high)
		return high;
	return v;
}

static float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = limit(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}

int update_ppm()
{
	char cmd[] = "rcstates\n";
	char output[20480] = {0};
	char *p = output;

	int len = test.command(cmd, strlen(cmd), output, sizeof(output));

	// parse result
	p = strstr(p, "rc:");
	if (p)
	{
		p+=3;
		for(int i=0; i<6; i++)
		{
			if (sscanf(p, "%d,%d,%d,", &ppm_states[i][0], &ppm_states[i][1], &ppm_states[i][2]) != 3)
				return -1;

			// skip comma
			p = strstr(p, ",")+1;
			p = strstr(p, ",")+1;
			p = strstr(p, ",")+1;
		}
	}

	// update scaled rc position
	for(int i=0; i<8; i++)
		rc[i] = ppm2rc(ppm_states[i][1], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);

	rc[2] = (rc[2]+1)/2;

	if (remote_calibrating)
	{
		for(int i=0; i<8; i++)
		{
			rc_setting[i][0] = ppm_states[i][0];
			rc_setting[i][2] = ppm_states[i][2];
		}
		for(int i=5; i<8; i++)
			rc_setting[i][1] = (ppm_states[i][0] + ppm_states[i][2])/2;
	}

	return 0;
}

int read_rc_settings()
{
	for(int i=0; i<8; i++)
	for(int j=0;j<4; j++)
	{
		char tmp[200];
		sprintf(tmp, "rc%d%d", i,j);
		if (test.read_float(tmp, &rc_setting[i][j]) < 0)
			return -1;
	}

	return 0;
}

int write_rc_settings()
{
	for(int i=0; i<8; i++)
	for(int j=0;j<4; j++)
	{
		char tmp[200];
		sprintf(tmp, "rc%d%d", i,j);
		if (test.write_float(tmp, rc_setting[i][j]) < 0)
			return -1;
	}

	return 0;
}

DWORD CALLBACK remote_update_thread(LPVOID p)
{
	while(true)
	{
		update_ppm();

		// update UI
		for(int i=0; i<6;i ++)
		{
			SendMessage(sliders[i], TBM_SETRANGEMIN, TRUE, 0);
			SendMessage(sliders[i], TBM_SETPOS, TRUE, i==2 ? (rc[i]*65535) : ((rc[i]+1)*32767));
			SendMessage(sliders[i], TBM_SETRANGEMAX, TRUE, 65535);
		}

//  		if (need_update_revert_button)
 		{
			DWORD revert_button_ids[] = {IDC_REVERT0, IDC_REVERT1, IDC_REVERT2, IDC_REVERT3, IDC_REVERT4, IDC_REVERT5};
 			for(int i=0; i<6;i ++)
				Button_SetCheck(GetDlgItem(wnd, revert_button_ids[i]), rc_setting[i][3] > 0 ? BST_CHECKED : BST_UNCHECKED);
 			need_update_revert_button = false;
 		}

		Sleep(17);
	}

	return 0;
}

int remote_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
		read_rc_settings();

	return 0;
}

INT_PTR CALLBACK WndProcRemote(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	HANDLE_CTLCOLORSTATIC;
	case WM_LBUTTONDOWN:
		SendMessage(GetParent(GetParent(hWnd)), WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_INITDIALOG:
		{
			wnd = hWnd;
			test.add_callback(remote_OnEvent);
			CreateThread(NULL, NULL, remote_update_thread, NULL, NULL, NULL);
			DWORD slider_ids[] = {IDC_SLIDER1, IDC_SLIDER2, IDC_SLIDER3, IDC_SLIDER4, IDC_SLIDER5, IDC_SLIDER6};
			for(int i=0; i<6;i ++)
				sliders[i] = GetDlgItem(hWnd, slider_ids[i]);
		}
		break;
	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			if (id == IDC_RC)
			{
				if (remote_calibrating)
				{
					remote_calibrating = false;

					// save
					// 
					wchar_t test[2048];
					swprintf(test, L"%f,%f,%f", 0, 1, 2);
					MessageBoxW(hWnd, test, L"info", MB_OK);
					write_rc_settings();
					SetDlgItemTextW(hWnd, IDC_RC, L"开始校准");
				}
				else
				{
					if (MessageBoxW(hWnd, L"请先将所有通道至于居中位置（包括油门），然后点击确定开始校准遥控器。"
						L"开始校准后请将所有通道分别拨动到极限位置来回几次（包括停机开关和控制模式通道）。" 
						L"或点击取消继续使用当前设定。", L"::", MB_OKCANCEL | MB_ICONINFORMATION) == IDCANCEL)
						break;

					remote_calibrating = true;
					SetDlgItemTextW(hWnd, IDC_RC, L"完成校准");

					char cmd[] = "rcreset\n";
					char output[20480] = {0};

					int len = test.command(cmd, strlen(cmd), output, sizeof(output));

					Sleep(100);
					update_ppm();
					for(int i=0; i<4; i++)
						rc_setting[i][1] = ppm_states[i][1];

				}
			}
			else if (id == IDC_REVERT0 || id == IDC_REVERT1 || id == IDC_REVERT2 || id == IDC_REVERT3 || id == IDC_REVERT4 || id == IDC_REVERT5)
			{
				DWORD revert_button_ids[] = {IDC_REVERT0, IDC_REVERT1, IDC_REVERT2, IDC_REVERT3, IDC_REVERT4, IDC_REVERT5};
				for(int i=0; i<6;i ++)
					rc_setting[i][3] = Button_GetCheck(GetDlgItem(wnd, revert_button_ids[i])) == BST_CHECKED ? 1 : 0;

				if (!remote_calibrating)
					write_rc_settings();

				need_update_revert_button = true;
			}
		}
		break;

	case WM_PAINT:
		return paint_white(hWnd, wParam, lParam);
		break;


	default:
		return FALSE;
	}
	return TRUE;
}