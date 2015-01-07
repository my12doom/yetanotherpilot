#include <stdio.h>
#include "simulator.h"
#include <Windows.h>
#include "resource.h"
#include <math.h>
#include "../common/common.h"

HWND wnd;

HPEN pen = CreatePen(PS_SOLID, 28, RGB(255, 0, 0));
HPEN green_pen = CreatePen(PS_SOLID, 28, RGB(0, 255, 0));
HPEN blue_pen = CreatePen(PS_SOLID, 28, RGB(0, 0, 255));
DWORD CALLBACK remote_update_thread(LPVOID p)
{
	while(true)
	{

		// update controller graph
		HWND graph_wnd = wnd;

		RECT rect;
		GetClientRect(graph_wnd, &rect);
		HDC hdc = GetDC(graph_wnd);
		HDC memDC = CreateCompatibleDC(hdc);
		HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
		HGDIOBJ obj = SelectObject(memDC, bitmap);


		FillRect(memDC, &rect, (HBRUSH)GetStockObject(GRAY_BRUSH));
		//  		BitBlt(memDC, 0, 0, rect.right, rect.bottom, brushDC, 0, 0, SRCCOPY);

		HDC brushDC = CreateCompatibleDC(hdc);
		//HGDIOBJ obj2 = SelectObject(brushDC, bg);
		//StretchBlt(memDC, 0, 0, rect.right, rect.bottom, brushDC, 0, 0, 229, 171, SRCCOPY);
		DeleteDC(brushDC);

		SelectObject(memDC, pen);

// 		int radius = 35;
// 		MoveToEx(memDC, 47, 82, NULL);
// 		LineTo(memDC, 47 + rc[3] * radius, 82 - (mode1?(rc[2]-0.5)*2:rc[1]) * radius);
// 		MoveToEx(memDC, 183, 82, NULL);
// 		LineTo(memDC, 183 + rc[0] * radius, 82 - (mode1?rc[1]:(rc[2]-0.5)*2) * radius);

		float x = rect.right/2 + pos[1] * 50;
		float y = rect.bottom/2 - pos[0] * 50;
		float dx = sin(euler[2])*28;
		float dy = -cos(euler[2])*28;
		float x2 = rect.right/2 + controller.setpoint[1] * 50;
		float y2 = rect.bottom/2 - controller.setpoint[0] * 50;

		SelectObject(memDC, blue_pen);
		MoveToEx(memDC, x2, y2, NULL);
		LineTo(memDC, x2+dx, y2+dy);

		SelectObject(memDC, pen);
 		MoveToEx(memDC, x, y, NULL);
 		LineTo(memDC, x+dx, y+dy);
		SelectObject(memDC, green_pen);
		LineTo(memDC, x+dx, y+dy);


		BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);
		DeleteDC(memDC);
		ReleaseDC(graph_wnd, hdc);
		DeleteObject(bitmap);

		printf("\r%f,%f -- v:%f,%f, tv:%f,%f       ", pos[0], pos[1], velocity[0], velocity[1], controller.target_velocity[0], controller.target_velocity[1]);
		char tmp[200];
		sprintf(tmp, "roll:%.0f/%.0f, pitch:%.0f/%.0f", euler[0] * 180 / PI, target_euler[0] * 180 / PI, euler[1] * 180 / PI, target_euler[1] * 180 / PI);
		SetWindowTextA(graph_wnd, tmp);
		Sleep(17);
	}

	return 0;
}

INT_PTR CALLBACK WndProcRemote(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		wnd = hWnd;
		CreateThread(NULL, NULL, remote_update_thread, NULL, NULL, NULL);
		break;
	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_LBUTTONDOWN:
		toggle_position_controller();
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
		}
		break;

	case WM_PAINT:
		break;


	default:
		return FALSE;
	}
	return TRUE;
}

int main()
{
	init_simulator();

	DialogBox(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, WndProcRemote);

	return 0;
}