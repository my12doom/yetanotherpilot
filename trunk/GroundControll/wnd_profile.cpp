#include <Windows.h>
#include <float.h>
#include "comm.h"
#include "resource.h"

extern Comm test;
static HWND wnd;
static HBITMAP hBitmap = LoadBitmap(GetModuleHandle(NULL), MAKEINTRESOURCE(IDB_EXPORT));
#define countof(x) (sizeof(x)/sizeof(x[0]))
int profile_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)

	return 0;
}

POINT coords1[] = 
{
	{0,0},
	{0,116},
	{25,163},
	{87,163},
	{123,116},
	{231,116},
	{231,0},
};

static int owner_draw_window(LPDRAWITEMSTRUCT lpDIS)
{

	printf("WM_DRAW\n");
	HDC hDC = lpDIS->hDC;
	RECT rectItem = lpDIS->rcItem;
	BOOL bIsDisabled = FALSE;

	// Draw the bitmap on button
	if ( hBitmap  != NULL ) {
		RECT rcImage;
		BITMAP bm;
		LONG cxBitmap, cyBitmap;
		if ( GetObject(hBitmap, sizeof(bm), &bm) ) {
			cxBitmap = bm.bmWidth;
			cyBitmap = bm.bmHeight;
		}

		// Center image horizontally  
		FillRect(hDC, &rectItem, (HBRUSH)GetStockObject(BLACK_BRUSH));
		CopyRect(&rcImage, &rectItem);
		LONG image_width = rcImage.right - rcImage.left;
		LONG image_height = rcImage.bottom - rcImage.top;
		rcImage.left = (image_width - cxBitmap)/2;
		rcImage.top = (image_height - cyBitmap)/2;            
		DrawState(hDC, NULL, NULL, (LPARAM)hBitmap, 0,
			rcImage.left, rcImage.top,
			rcImage.right - rcImage.left,
			rcImage.bottom - rcImage.top, 
			(bIsDisabled ? DSS_DISABLED : DSS_NORMAL) | DST_BITMAP);
	}

	return TRUE;
}

INT_PTR CALLBACK WndProcProfile(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		wnd = hWnd;
		test.add_callback(profile_OnEvent);
		{
			HWND hButton = GetDlgItem(hWnd, IDC_SAVE);
			SetClassLong(hButton, GCL_STYLE, GetClassLong(hButton, GCL_STYLE) & (~CS_PARENTDC));
			HRGN polygon = CreatePolygonRgn(coords1, countof(coords1), WINDING);
			SetWindowRgn(hButton, polygon, TRUE);
		}

		break;

	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_DRAWITEM:
		return owner_draw_window((LPDRAWITEMSTRUCT)lParam);

	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			if (id == IDC_RESET && MessageBoxW(hWnd, L"Reset, Continue?", L"?", MB_YESNO) == IDYES)
			{
				char cmd[] = "resetmc\n";
				char output[1024] = {0};
				test.command(cmd, strlen(cmd), output);
			}

			else if (id == IDC_SAVE)
			{
				printf("saving\n");
				FILE * f = fopen("Z:\\save.log", "wb");
				char fourcc[10];
				float v;
				int i = 0;
				while(true)
				{
					int o = test.enum_float(i++, fourcc, &v);
					if (o == 1 || o < 0)
						break;

					if (_isnan(v) || !_finite(v))
					{
						printf("NAN %s = %f\n", fourcc, v);
						fprintf(f, "%s=NAN\n", fourcc, v);
					}
					else
					{
						fprintf(f, "%s=%f\n", fourcc, v);
					}
				}
				fclose(f);
			}

			else if (id == IDC_LOAD)
			{
				char cmd[1024];
				char output[20480];
				FILE * f = fopen("Z:\\save.log", "rb");
				while (fgets(cmd, 1024, f))
				{
					const char *p = strstr(cmd, "=");
					if (!p)
						continue;

					test.command(cmd, strlen(cmd), output);
				}

				fclose(f);
				test.disconnect();
			}

		}
		break;

	default:
		return FALSE;
	}
	return TRUE;
}