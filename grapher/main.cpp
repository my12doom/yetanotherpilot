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
RGBQUAD blue = {255,0,0,0};
RGBQUAD black = {0,0,0,0};

enum data_type
{
	_uint8,
	_uint16,
	_uint32,
	___int8,
	___int16,
	___int32,
	___float,
};
typedef struct
{
	int64_t tag;
	int offset;
	data_type type;
	float scale;
	RGBQUAD color;
} data_source;

data_source sources[] = 
{
	{
		TAG_PILOT_DATA,
		0,
		___int32,
		1,
		blue,
	},

	{
		TAG_IMU_DATA_V1,
		4,
		___int32,
		0.001,
		red,
	},
	{
		TAG_QUADCOPTER_DATA3,
		20,
		___int16,
		0.1,
		{0,255,0,0},
	},
};

int main()
{
	FILE * f = fopen("C:\\Tencent Files\\411384033\\FileRecv\\MobileFile\\0032.DAT", "rb");
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

int draw_point(int x, int y, RGBQUAD color)
{
	if (y>=0 && y<height && x>=0 && x<width)
		bits[y*width+x] = color;
	return 0;
}

int draw_line(int x1, int y1, int x2, int y2, RGBQUAD color)
{
	int dx = x2 - x1;
	int dy = y2 - y1;
	int ux = ((dx > 0) << 1) - 1;//x的增量方向，取或-1
	int uy = ((dy > 0) << 1) - 1;//y的增量方向，取或-1
	int x = x1, y = y1, eps;//eps为累加误差

	eps = 0;dx = abs(dx); dy = abs(dy); 
	if (dx > dy) 
	{
		for (x = x1; x != x2; x += ux)
		{
			draw_point(x, y, color);
			eps += dy;
			if ((eps << 1) >= dx)
			{
				y += uy; eps -= dx;
			}
		}
	}
	else
	{
		for (y = y1; y != y2; y += uy)
		{
			draw_point(x, y, color);
			eps += dx;
			if ((eps << 1) >= dy)
			{
				x += ux; eps -= dy;
			}
		}
	}

	return 0;
}

int64_t time_left;
int64_t time_right;
int offset_y = 0;
int drawing()
{
	memset(bits, 0xff, width * height * 4);
	draw_line(0, height/2+offset_y, width, height/2+offset_y, black);

	int p = GetTickCount();


	for(int i=0; i<sizeof(sources)/sizeof(sources[0]); i++)
	{
		data_source &source = sources[i];
		int last_x = -1;
		int last_y = -1;

		for(int x=0; x<width; x++)
		{
			rf_data *l = find_point(x*(time_right - time_left)/width + time_left);
			rf_data *r = find_point((x+1)*(time_right-time_left)/width + time_left);

			if (l<=data || r >= data+data_count-1)
				continue;
			for (rf_data * v = l; v<=r; v++)
			{
				if ((v->time & TAG_MASK) == source.tag)
				{
					float data;
					char *p = (char*)v;
					p += source.offset + 8;
					switch(source.type)
					{
					case ___int8:
						data = *(char*)p;
						break;
					case ___int16:
						data = *(short*)p;
						break;
					case ___int32:
						data = *(int*)p;
						break;
					case _uint8:
						data = *(unsigned char*)p;
						break;
					case _uint16:
						data = *(unsigned short*)p;
						break;
					case _uint32:
						data = *(unsigned int*)p;
						break;
					case ___float:
						data = *(float*)p;
						break;
					}

					int y = height/2-1 - (data*source.scale * height / 1500) + offset_y;

					if (last_x>0)
						draw_line(last_x, last_y, x, y, source.color);
					else
						draw_point(x, y, source.color);
					last_x = x;
					last_y = y;
				}
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

int dragging = -9999;
int draggingy = -9999;
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
				draggingy = mouse.y;
			}
		}
		break;
	case WM_PAINT:
		PAINTSTRUCT ps;
		HDC hdc;
		hdc = BeginPaint(hDlg, &ps);
		draw_graph();
		EndPaint(hDlg, &ps);
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
			int deltay = draggingy - mouse.y;
			dragging = mouse.x;
			draggingy = mouse.y;

			time_left += delta * time_per_pixel;
			time_right += delta * time_per_pixel;
			offset_y -= deltay;

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