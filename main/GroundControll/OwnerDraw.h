#pragma once

#include <Windows.h>

#define HANDLE_CTLCOLORSTATIC case WM_CTLCOLORSTATIC: return handle_WM_CTLCOLORSTATIC(wParam);


int update_ownerdraw_window();
BOOL draw_window(LPDRAWITEMSTRUCT lpDIS);
int init_owner_draw(HWND top);
int paint_white(HWND hWnd, WPARAM wparam, LPARAM lparam);
int handle_WM_CTLCOLORSTATIC(WPARAM wParam);
