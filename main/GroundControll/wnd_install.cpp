#include <WindowsX.h>
#include "wnd_install.h"
#include "resource.h"
#include "comm.h"
#include "OwnerDraw.h"
#include "common.h"

extern Comm test;

float matrix;
float throttle_idle;

HWND wnd;

int read_install()
{
	test.read_float("mat", &matrix);
	test.read_float("idle", &throttle_idle);

	// update UI
	// matrix
	DWORD matrix_table[] = {IDC_RADIO_MATRIX0, IDC_RADIO_MATRIX1};
	CheckRadioButton(wnd, IDC_RADIO_MATRIX0, IDC_RADIO_MATRIX1, matrix_table[int(matrix)]);

	// idle throtte
	DWORD matrix_table2[] = {IDC_IDLE_0, IDC_IDLE_1, IDC_IDLE_2, IDC_IDLE_3, IDC_IDLE_4};
	CheckRadioButton(wnd, IDC_IDLE_0, IDC_IDLE_4, matrix_table2[int(throttle_idle - 1144)/16]);


	DWORD matrix_table3[] = {IDC_MATRIX_0, IDC_MATRIX_1};
	for(int i=0; i<2; i++)
		Button_SetState(GetDlgItem(wnd, matrix_table3[i]), i == int(matrix));


	return 0;
}


int write_install()
{
	test.write_float("mat", matrix);
	test.write_float("idle", throttle_idle);
	return read_install();
}

int install_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
		read_install();

	return 0;
}

INT_PTR CALLBACK WndProcInstall(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	HANDLE_CTLCOLORSTATIC;
	case WM_LBUTTONDOWN:
		SendMessage(GetParent(GetParent(hWnd)), WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_INITDIALOG:
		wnd = hWnd;
		test.add_callback(install_OnEvent);
		break;

	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;

	case WM_DRAWITEM:
		return draw_window((LPDRAWITEMSTRUCT)lParam);

	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			bool down = false;
			if (id == IDC_MATRIX_0 || id == IDC_RADIO_MATRIX0)
			{
				matrix = 0;
				write_install();
			}
			else if (id == IDC_MATRIX_1 || id == IDC_RADIO_MATRIX1)
			{
				matrix = 1;
				write_install();
			}
			else if (id == IDC_IDLE_0)
			{
				throttle_idle = 1144;
				write_install();
			}
			else if (id == IDC_IDLE_1)
			{
				throttle_idle = 1160;
				write_install();
			}
			else if (id == IDC_IDLE_2)
			{
				throttle_idle = 1176;
				write_install();
			}
			else if (id == IDC_IDLE_3)
			{
				throttle_idle = 1192;
				write_install();
			}
			else if (id == IDC_IDLE_4)
			{
				throttle_idle = 1208;
				write_install();
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