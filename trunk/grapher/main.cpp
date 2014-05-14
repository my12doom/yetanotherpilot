#include <Windows.h>
#include <WindowsX.h>
#include <stdio.h>
#include "resource.h"
#include "..\RFData.h"

INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam );


HWND g_hwnd;
HWND g_graph;

rf_data *data;
int data_size = 0;
int data_count = 0;

RGBQUAD red = {0,0,255,0};

int main()
{
	FILE * f = fopen("C:\\Tencent Files\\411384033\\FileRecv\\MobileFile\\0002.DAT", "rb");
	fseek(f, 0, SEEK_END);
	data_size = ftell(f);
	fseek(f, 0, SEEK_SET);
	data_count = data_size / sizeof(rf_data);
	data = new rf_data[data_count];
	int c = fread(data, 1, data_size, f);
	fclose(f);

	DialogBoxW(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, main_window_proc);
	return 0;
}

RGBQUAD *bits = NULL;
int width = 0;
int height = 0;

rf_data * find_point(int64_t timestamp)
{
	int l = 0;
	int r = data_count-1;

	if (timestamp < (data[l].time &~TAG_MASK))
		return data+l;
	if (timestamp > (data[r].time &~TAG_MASK))
		return data+r;

	while (r>l)
	{
		int c = (r+l)/2;
		int tl = data[l].time & ~TAG_MASK;
		int tr = data[r].time & ~TAG_MASK;
		if (c == l || c == r)
			break;


		if (timestamp == (data[r].time &~TAG_MASK))
			return data+r;

		if (timestamp == (data[l].time&~TAG_MASK))
			return data+l;

		if (timestamp > (data[l].time&~TAG_MASK) && timestamp <= (data[c].time&~TAG_MASK))
			r = c;
		else if (timestamp >= (data[c].time&~TAG_MASK) && timestamp < (data[r].time&~TAG_MASK))
			l = c;
	}

	return data+l;
}

int64_t time_left;
int64_t time_right;
int drawing()
{
	memset(bits, 0xff, width * height * 4);
	memset(bits + width * height/2, 0, width * 4);

	int p = GetTickCount();
	for(int x=0; x<width; x++)
	{
		rf_data *l = find_point(x*(time_right - time_left)/width + time_left);
		rf_data *r = find_point((x+1)*(time_right-time_left)/width + time_left);

		if (l<=data || r >= data+data_count-1)
			continue;

		for (rf_data * v = l; v<=r; v++)
		{
			if ((v->time & TAG_MASK) == TAG_SENSOR_DATA)
			{
				int y = height/2-1 - (v->data.sensor.current * height / 15000) + height/2;
				if (y>=0 && y<height && x>=0 && x<width)
					bits[y*width+x] = red;
			}
		}
	}

	p = GetTickCount() - p;

	return 0;
}


int draw_graph()
{

	RECT rect;
	HWND hwnd = GetDlgItem(g_hwnd, IDC_GRAPH);
	GetClientRect(hwnd, &rect);
	HDC hdc = GetDC(hwnd);
	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
	HGDIOBJ obj = SelectObject(memDC, bitmap);

	drawing();
	SetBitmapBits(bitmap, rect.right * rect.bottom * 4, bits);

	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(hwnd, hdc);
	return 0;
}

DWORD old_proc;
INT_PTR CALLBACK graph_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch(msg)
	{
	case WM_MOUSEWHEEL:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
		}
		break;
	default:
		break;
	}

	return CallWindowProc((WNDPROC)old_proc,hDlg ,msg, wParam, lParam);
}

int dragging = -9999;
INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_COMMAND:
		draw_graph();
		break;
	case WM_LBUTTONDOWN:
		{
			POINT mouse;
			GetCursorPos(&mouse);
			ScreenToClient(g_graph, &mouse);
			if (mouse.x > 0 && mouse.x < width && mouse.y > 0 && mouse.y < height)
			{
				GetCursorPos(&mouse);
				dragging = mouse.x;
			}
		}
		break;
	case WM_PAINT:
		draw_graph();
		break;
	case WM_LBUTTONUP:
		dragging = -9999;
		break;
	case WM_MOUSEMOVE:
		if (dragging > 0)
		{
			int time_per_pixel = (time_right - time_left)/width;
			POINT mouse;
			GetCursorPos(&mouse);
			int delta = dragging - mouse.x;
			dragging = mouse.x;

			time_left += delta * time_per_pixel;
			time_right += delta * time_per_pixel;
			

			draw_graph();
		}
		break;
	case WM_MOUSEWHEEL:
		{
			POINT mouse;
			GetCursorPos(&mouse);
			ScreenToClient(g_graph, &mouse);

			if (mouse.x > 0 && mouse.x < width && mouse.y > 0 && mouse.y < height)
			{
				short delta = HIWORD(wParam);
				float factor = delta > 0 ? 0.85 : 1.176;
				int64_t center = time_left + (time_right-time_left) * mouse.x / width;
				int64_t width_left = center - time_left;
				int64_t width_right = time_right - center;

				time_left = center - width_left * factor;
				time_right = center + width_right * factor;

			}

			draw_graph();
		}
		break;

	case WM_INITDIALOG:
		{
			g_graph = GetDlgItem(hDlg, IDC_GRAPH);
			g_hwnd = hDlg;
			old_proc = GetWindowLong(g_graph, GWL_WNDPROC);
			SetWindowLongPtr(g_graph, GWL_WNDPROC, (LONG_PTR)graph_proc);
			RECT rec;
			GetClientRect(g_graph, &rec);
			bits = (RGBQUAD*)malloc(rec.right*rec.bottom*4);
			width = rec.right;
			height = rec.bottom;
			time_left = 0;
			time_right= data[data_count-1].time & (~TAG_MASK);
		}
		break;

	case WM_CLOSE:
		EndDialog(hDlg, 0);
		break;

	case WM_SIZE:
		{
			width = LOWORD(lParam);
			height = HIWORD(lParam);

			if (bits)
				free(bits);
			bits = (RGBQUAD*)malloc(width*height*4);
		}
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}