#include <Windows.h>
#include "resource.h"
#include "wnd_install.h"
#include "wnd_remote.h"
#include "wnd_pid.h"
#include "wnd_calibration.h"
#include "wnd_profile.h"
#include "comm.h"

INT_PTR CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK WndProc2(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#define countof(x) (sizeof(x)/sizeof((x)[0]))

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
	{IDC_PROFILE, IDD_PROFILE, WndProc2},
	{0, IDD_INTRODUTION, WndProc2},
};

HWND pages_hwnd[100];
HWND g_hwnd;


INT_PTR CALLBACK WndProc2(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		break;
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hWnd, &ps);
			EndPaint(hWnd, &ps);
		}
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
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

INT_PTR CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		{
			g_hwnd = hWnd;
			HWND page_parent = GetDlgItem(g_hwnd, IDC_PAGES);
			for(int i=0; i<countof(pages); i++)
			{
				pages_hwnd[i] = HWND_BOTTOM;
				pages_hwnd[i] = CreateDialogParam(GetModuleHandle(NULL),MAKEINTRESOURCE(pages[i].dialogid), page_parent, pages[i].proc, 0);
			}
			show_page(0);
		}
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			show_page(id);
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

Comm test;
int main()
{
	return DialogBoxW(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_MAIN), NULL, WndProc);
}

int WINAPI WinMain(HINSTANCE hinstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) 
{
	return main();
}