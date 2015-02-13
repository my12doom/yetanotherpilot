#include <Windows.h>
#include <float.h>
#include "comm.h"
#include "resource.h"
#include "OwnerDraw.h"
#include "comm.h"
#include "common.h"

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
	HANDLE_CTLCOLORSTATIC;
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
			if (id == IDC_RESET && MessageBoxW(hWnd, L"确定要为飞控恢复出厂设置？恢复后需要重新连接USB接口", L"?", MB_YESNO | MB_ICONWARNING) == IDYES)
			{
				char cmd[] = "resetmc\n";
				char output[1024] = {0};
				test.command(cmd, strlen(cmd), output, sizeof(output));
			}

			else if (id == IDC_SAVE)
			{
				wchar_t file[MAX_PATH] = {0};
				if (open_file_dlg(file, hWnd, false, NULL))
				{
					printf("saving\n");
					FILE * f = _wfopen(file, L"wb");
					if (!f)
						MessageBoxW(hWnd, L"导出失败，请确认文件未被其他程序占用", L"失败", MB_OK | MB_ICONERROR);
					else
					{
						char fourcc[10];
						float v;
						int i = 0;
						fprintf(f, "yap1.0.0\n");
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
						MessageBoxW(hWnd, L"导出成功，本配置文件包含遥控器和传感器信息，导入到不同遥控器或另一块飞控板请重新校准", L"成功", MB_OK | MB_ICONINFORMATION);
					}
				}
			}

			else if (id == IDC_LOAD)
			{
				wchar_t file[MAX_PATH] = {0};
				if (open_file_dlg(file, hWnd, true, NULL))
				{
					char cmd[1024] = {0};
					char output[20480];
					FILE * f = _wfopen(file, L"rb");
					if (!f)
						MessageBoxW(hWnd, L"导入失败，请确认文件未被其他程序占用", L"失败", MB_OK | MB_ICONERROR);
					else
					{
						fgets(cmd, 1024, f);
						if (strstr(cmd, "yap") != cmd)
						{
							MessageBoxW(hWnd, L"导入失败，无效的配置文件", L"失败", MB_OK | MB_ICONERROR);
							fclose(f);
						}
						else
						{
							while (fgets(cmd, 1024, f))
							{
								const char *p = strstr(cmd, "=");
								if (!p)
									continue;

								test.command(cmd, strlen(cmd), output, sizeof(output));
							}

							fclose(f);
							MessageBoxW(hWnd, L"载入成功，本配置文件包含遥控器和传感器信息，如果导入到不同遥控器或另一块飞控板请重新校准", L"成功", MB_OK | MB_ICONINFORMATION);
							test.disconnect();
						}
					}
				}
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