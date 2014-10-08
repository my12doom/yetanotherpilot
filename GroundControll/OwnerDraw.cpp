#include "OwnerDraw.h"
#include "resource.h"
#include <map>

using namespace std;
map<DWORD,HBITMAP> bitmaps[3];	// {normal, hover, down} [control_id --> HBITMAP]
const int MAX_COORD = 16;
map<DWORD,HRGN> region;	// {normal, hover, down} [control_id --> HBITMAP]

typedef struct
{
	int controll_id;
	int left;
	int top;
	int width;
	int height;
	POINT coords[MAX_COORD];
	int normal;
	int hover;
	int down;
} OWNERDRAW_BUTTON_DEFINE;

OWNERDRAW_BUTTON_DEFINE od_buttons[] = 
{
	{
		IDC_MINIMIZE,
		812,0,42,42,
		{},
		IDB_MINIMIZE_NORMAL,
		IDB_MINIMIZE_HOVER,
		IDB_MINIMIZE_DOWN,
	},
	{
		IDC_INSTALL,
		812,0,42,42,
		{},
		IDB_MENU0_NORMAL,
		IDB_MENU0_HOVER,
		IDB_MENU0_DOWN,
	},
	{
		IDC_REMOTE,
		812,0,42,42,
		{},
		IDB_MENU1_NORMAL,
		IDB_MENU1_HOVER,
		IDB_MENU1_DOWN,
	},
	{
		IDC_PID,
		812,0,42,42,
		{},
		IDB_MENU2_NORMAL,
		IDB_MENU2_HOVER,
		IDB_MENU2_DOWN,
	},
	{
		IDC_CALIBRATION,
		812,0,42,42,
		{},
		IDB_MENU3_NORMAL,
		IDB_MENU3_HOVER,
		IDB_MENU3_DOWN,
	},
	{
		IDC_PROFILE,
		812,0,42,42,
		{},
		IDB_MENU4_NORMAL,
		IDB_MENU4_HOVER,
		IDB_MENU4_DOWN,
	},
	{
		IDC_CLOSE,
		854,0,42,42,
		{},
		IDB_CLOSE_NORMAL,
		IDB_CLOSE_HOVER,
		IDB_CLOSE_DOWN,
	},
	{
		IDC_SAVE,
		854,0,231,151,
		{
			{67, 0},
			{246,0},
			{246,34},
			{128,34},
			{108,70},
			{133,115},
			{100,170},
			{36,170},
			{1,113},
		},
		IDB_SAVE_NORMAL,
		IDB_SAVE_HOVER,
		IDB_SAVE_DOWN,
	},
	{
		IDC_RESET,
		854,0,271,111,
		{
			{31, 0},
			{97, 0},
			{114, 28},
			{270, 28},
			{270, 63},
			{124, 63},
			{97, 110},
			{31, 110},
			{0, 55},
		},
		IDB_RESET_NORMAL,
		IDB_RESET_HOVER,
		IDB_RESET_DOWN,
	},
	{
		IDC_LOAD,
		854,0,247,171,
		{
			{132, 0},
			{199, 0},
			{230, 55},
			{198, 111},
			{143, 111},
			{120, 150},
			{0, 150},
			{0, 122},
			{123, 122},
			{132, 108},
			{101, 54},
		},
		IDB_IMPORT_NORMAL,
		IDB_IMPORT_HOVER,
		IDB_IMPORT_DOWN,
	},
	{
		IDC_MATRIX_0,
		0,0,250,250,
		{},
		IDB_I_NORMAL,
		IDB_I_MOUSEOVER,
		IDB_I_DOWN,
	},
	{
		IDC_MATRIX_1,
		0,0,250,250,
		{},
		IDB_X_NORMAL,
		IDB_X_mouseover,
		IDB_X_DOWN,
	},
};


extern HWND last_wnd;
BOOL draw_window(LPDRAWITEMSTRUCT lpDIS)
{
	printf("WM_DRAW\n");
	HDC hDC = lpDIS->hDC;
	RECT rectItem = lpDIS->rcItem;
	BOOL down = lpDIS->itemState & ODS_SELECTED;
	BOOL hover = last_wnd == lpDIS->hwndItem;
	DWORD controll_id = lpDIS->CtlID;

	// update region on first draw
	if (region[controll_id])
	{
		SetClassLong(lpDIS->hwndItem, GCL_STYLE, GetClassLong(lpDIS->hwndItem, GCL_STYLE) & (~CS_PARENTDC));
		SetWindowRgn(lpDIS->hwndItem, region[controll_id], TRUE);
		InvalidateRect(lpDIS->hwndItem, NULL, FALSE);
		region[controll_id] = NULL;
	}


	// Draw the bitmap on button

	HBITMAP hBitmap = (down ? bitmaps[2] : (hover ? bitmaps[1] : bitmaps[0]))[controll_id];

	if ( hBitmap  != NULL ) {
		RECT rcImage;
		BITMAP bm;
		LONG cxBitmap, cyBitmap;
		if ( GetObject(hBitmap, sizeof(bm), &bm) ) {
			cxBitmap = bm.bmWidth;
			cyBitmap = bm.bmHeight;
		}

		printf("%x\n", lpDIS->itemState);

		// Center image horizontally  
// 		FillRect(hDC, &rectItem, (HBRUSH)GetStockObject(BLACK_BRUSH));
		CopyRect(&rcImage, &rectItem);
		LONG image_width = rcImage.right - rcImage.left;
		LONG image_height = rcImage.bottom - rcImage.top;
		rcImage.left = (image_width - cxBitmap)/2;
		rcImage.top = (image_height - cyBitmap)/2;            
		DrawState(hDC, NULL, NULL, (LPARAM)hBitmap, 0,
			rcImage.left, rcImage.top,
			rcImage.right - rcImage.left,
			rcImage.bottom - rcImage.top, 
			DSS_NORMAL | DST_BITMAP);
	}

	return TRUE;
}


BOOL CALLBACK enum_proc(HWND hwnd, LPARAM lParam)
{
	DWORD controll_id = GetDlgCtrlID(hwnd);
	if (region[controll_id])
	{
		SetClassLong(hwnd, GCL_STYLE, GetClassLong(hwnd, GCL_STYLE) & (~CS_PARENTDC));
		SetWindowRgn(hwnd, region[controll_id], TRUE);
		InvalidateRect(hwnd, NULL, FALSE);
// 		region[controll_id] = NULL;
	}


	EnumChildWindows(hwnd, enum_proc, NULL);

	return TRUE;
}

HRESULT update_child_regions(HWND hwnd)
{
// 	EnumChildWindows(hwnd, enum_proc, NULL);
	return S_OK;
}

int init_owner_draw(HWND top)
{
	int count = sizeof(od_buttons)/sizeof(od_buttons[0]);
	HMODULE module = GetModuleHandle(NULL);
	POINT zero = {0};
	for(int i=0; i<count; i++)
	{
		bitmaps[0][od_buttons[i].controll_id] = LoadBitmap(module, MAKEINTRESOURCE(od_buttons[i].normal));
		bitmaps[1][od_buttons[i].controll_id] = LoadBitmap(module, MAKEINTRESOURCE(od_buttons[i].hover));
		bitmaps[2][od_buttons[i].controll_id] = LoadBitmap(module, MAKEINTRESOURCE(od_buttons[i].down));

		int j=0;
		for(j=0; j<MAX_COORD; j++)
			if (memcmp(&zero, &od_buttons[i].coords[j], sizeof(zero)) == 0)
				break;

		if (j)
			region[od_buttons[i].controll_id] = CreatePolygonRgn(od_buttons[i].coords, j, WINDING);
		else
			region[od_buttons[i].controll_id] = NULL;
	}

	update_child_regions(top);

	return 0;
}


static int refresh_window(HWND hwnd)
{
	int count = sizeof(od_buttons)/sizeof(od_buttons[0]);
	for(int i=0; i<count; i++)
		if (od_buttons[i].controll_id == GetDlgCtrlID(hwnd))
			InvalidateRect(hwnd, NULL, FALSE);

	return 0;
}

int update_ownerdraw_window()
{
	POINT p;
	GetCursorPos(&p);
	HWND wnd = WindowFromPoint(p);
	while (wnd)
	{
		GetCursorPos(&p);
		ScreenToClient(wnd, &p);
		HWND child_wnd = ChildWindowFromPoint(wnd, p);

		if (child_wnd == wnd)
			break;
		wnd = child_wnd;
	}
	if (wnd != last_wnd)
	{
		refresh_window(wnd);
		refresh_window(last_wnd);
		last_wnd = wnd;
	}

	return 0;
}

int paint_white(HWND hWnd, WPARAM wparam, LPARAM lparam)
{
	PAINTSTRUCT ps;
	RECT rect;
	GetClientRect(hWnd, &rect);
	HDC hdc = BeginPaint(hWnd, &ps);


	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
	HGDIOBJ obj2 = SelectObject(memDC, bitmap);


	FillRect(memDC, &rect, (HBRUSH)GetStockObject(WHITE_BRUSH));

	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);


	DeleteObject(bitmap);
	DeleteDC(memDC);

	EndPaint(hWnd, &ps);

	return FALSE;
}


int handle_WM_CTLCOLORSTATIC(WPARAM wParam)
{
	HDC hdcStatic = (HDC) wParam;
	SetTextColor(hdcStatic, RGB(0,0,0));
	SetBkColor(hdcStatic, RGB(255,255,255));

	static HBRUSH hbrBkgnd = NULL;
	if (hbrBkgnd == NULL)
	{
		hbrBkgnd = CreateSolidBrush(RGB(255,255,255));
	}
	return (INT_PTR)hbrBkgnd;
}