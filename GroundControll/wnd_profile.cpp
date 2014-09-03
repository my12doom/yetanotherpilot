#include <Windows.h>
#include <float.h>
#include "comm.h"
#include "resource.h"
#include "OwnerDraw.h"

extern Comm test;
static HWND wnd;
#define countof(x) (sizeof(x)/sizeof(x[0]))
int profile_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)

	return 0;
}

INT_PTR CALLBACK WndProcProfile(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_LBUTTONDOWN:
		SendMessage(GetParent(GetParent(hWnd)), WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_INITDIALOG:
		wnd = hWnd;
		test.add_callback(profile_OnEvent);

		break;

	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_DRAWITEM:
		return draw_window((LPDRAWITEMSTRUCT)lParam);

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
	case WM_PAINT:
		return paint_white(hWnd, wParam, lParam);
		break;

	default:
		return FALSE;
	}
	return TRUE;
}