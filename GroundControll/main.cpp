#include <Windows.h>
#include "resource.h"
#include "wnd_install.h"
#include "wnd_remote.h"
#include "wnd_pid.h"
#include "wnd_calibration.h"
#include "wnd_profile.h"
#include "comm.h"
#include "OwnerDraw.h"

INT_PTR CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK WndProc2(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#define countof(x) (sizeof(x)/sizeof((x)[0]))

Comm test;
struct
{
	DWORD controllid;
	DWORD dialogid;
	DLGPROC proc;
} pages[] = 
{
	{IDC_INSTALL, IDD_INSTALL, WndProcInstall},
	{IDC_REMOTE, IDD_REMOTE, WndProcRemote},
	{IDC_PID, IDD_PID, WndProcPid},
	{IDC_CALIBRATION, IDD_CALIBRATION, WndProcCalibration},
	{IDC_PROFILE, IDD_PROFILE, WndProcProfile},
	{0, IDD_INTRODUTION, WndProc2},
};

HWND pages_hwnd[100];
HWND g_hwnd;
DWORD active_id = 0;

HBITMAP hBG = LoadBitmap(GetModuleHandle(NULL), MAKEINTRESOURCE(IDB_BG));

HWND last_wnd = NULL;



INT_PTR CALLBACK WndProc2(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		SetTimer(hWnd, 1, 10, NULL);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_TIMER:
		update_ownerdraw_window();
		break;
	case WM_DRAWITEM:
		return draw_window((LPDRAWITEMSTRUCT)lParam);
	case WM_PAINT:
		return paint_white(hWnd, wParam, lParam);
		break;
	default:
		return FALSE;
	}
	return TRUE;
}

int show_page(int id)
{
	HWND page_parent = GetDlgItem(g_hwnd, IDC_PAGES);
	RECT page_rect;
	GetClientRect(page_parent, &page_rect);
	for(int i=0; i<countof(pages); i++)
		if (pages[i].controllid == id)
		{
			MoveWindow(pages_hwnd[i],page_rect.left,page_rect.top,page_rect.right,page_rect.bottom,0);
			ShowWindow(pages_hwnd[i],SW_SHOW);
			EnableWindow(pages_hwnd[i], TRUE);
		}
		else
		{
			EnableWindow(pages_hwnd[i], FALSE);
			ShowWindow(pages_hwnd[i],SW_HIDE);
		}
	return 0;
}


int main_OnEvent(int code, void *extra_data)
{
// 	if (code == WM_DISCONNECT)
// 		show_page(0);
// 	if (code == WM_CONNECT)
// 		show_page(active_id = (active_id ? active_id : IDC_INSTALL));

	return 0;
}

INT_PTR CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_MOUSEMOVE:
		InvalidateRect(hWnd, NULL, FALSE);
		break;
	case WM_INITDIALOG:
		{
			// clip out the border
			RECT client;
			GetClientRect(hWnd, &client);
			RECT screen;
			GetWindowRect(hWnd, &screen);

			POINT topleft = {0,0};
			POINT bottomright = {client.right, client.bottom};

			ClientToScreen(hWnd, &topleft);
			ClientToScreen(hWnd, &bottomright);

			int width = bottomright.x - topleft.x;
			int height = bottomright.y - topleft.y;
			int left = topleft.x - screen.left;
			int top = topleft.y - screen.top;
			int right = left + width;
			int bottom = top + height;

			POINT coords[4] = 
			{
				{left, top},		// top left
				{right, top},		// top right
				{right, bottom},		// bottom right
				{left, bottom},		// bottom left
			};

			HRGN polygon = CreatePolygonRgn(coords, 4, WINDING);
			SetWindowRgn(hWnd, polygon, TRUE);
			SetClassLong(hWnd, GCL_STYLE, GetClassLong(hWnd, GCL_STYLE) | CS_DROPSHADOW);
			

			test.add_callback(main_OnEvent);
			g_hwnd = hWnd;
			HWND page_parent = GetDlgItem(g_hwnd, IDC_PAGES);
			for(int i=0; i<countof(pages); i++)
			{
				pages_hwnd[i] = HWND_BOTTOM;
				pages_hwnd[i] = CreateDialogParam(GetModuleHandle(NULL),MAKEINTRESOURCE(pages[i].dialogid), page_parent, pages[i].proc, 0);
			}
			show_page(0);

			init_owner_draw(hWnd);

		}

		break;
	case WM_LBUTTONDOWN:
		SendMessage(hWnd, WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_ERASEBKGND:
		break;
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			RECT rect;
			GetClientRect(hWnd, &rect);
			HDC hdc = BeginPaint(hWnd, &ps);


			HDC memDC = CreateCompatibleDC(hdc);
			HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
			HGDIOBJ obj2 = SelectObject(memDC, bitmap);

			HDC brushDC = CreateCompatibleDC(hdc);
			HGDIOBJ obj = SelectObject(brushDC, hBG);

			FillRect(memDC, &rect, (HBRUSH)GetStockObject(BLACK_BRUSH));
			StretchBlt(memDC, 0, 0, rect.right, rect.bottom, brushDC, 0, 0, 900, 530, SRCCOPY);

			BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);

			

			DeleteObject(obj);
			DeleteObject(bitmap);
			DeleteDC(brushDC);
			DeleteDC(memDC);

			EndPaint(hWnd, &ps);
		}
		break;
	case WM_DRAWITEM:
		return draw_window((LPDRAWITEMSTRUCT)lParam);
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);

			if (id == IDC_CLOSE)
				EndDialog(hWnd, 0);
			if (id == IDC_MINIMIZE)
				ShowWindow(hWnd, SW_MINIMIZE);

			show_page(id);
			active_id = id;
		}
		break;

	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;

	default:
		return FALSE;
	}
	return TRUE;
}

int main()
{
	return DialogBoxW(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_MAIN), NULL, WndProc);
}

int WINAPI WinMain(HINSTANCE hinstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) 
{
	return main();
}