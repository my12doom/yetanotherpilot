#include "common.h"

bool open_file_dlg(wchar_t *pathname, HWND hDlg, bool do_open /* = false*/, wchar_t *filter/* = NULL*/)
{
	static wchar_t *default_filter = 
		L"·É¿ØÅäÖÃÐÅÏ¢\t(*.yap)\0*.yap\0"
		L"All Files\0*.*\0"
		L"\0";

	if (filter == NULL) filter = default_filter;
	wchar_t strFileName[MAX_PATH] = L"";
	wchar_t strPath[MAX_PATH] = L"";

	wcsncpy(strFileName, pathname, MAX_PATH);
	wcsncpy(strPath, pathname, MAX_PATH);
	for(int i=(int)wcslen(strPath)-2; i>=0; i--)
		if (strPath[i] == L'\\')
		{
			strPath[i+1] = NULL;
			break;
		}

		OPENFILENAMEW ofn = { sizeof(OPENFILENAMEW), hDlg , NULL,
			filter, NULL,
			0, 1, strFileName, MAX_PATH, NULL, 0, strPath,
			(L"Open File"),
			do_open ? OFN_FILEMUSTEXIST|OFN_HIDEREADONLY|OFN_ENABLESIZING : OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY, 0, 0,
			L".mp4", 0, NULL, NULL };

		int o = do_open ? GetOpenFileNameW( &ofn ) : GetSaveFileName( &ofn );
		if (o)
		{
			wcsncpy(pathname, strFileName, MAX_PATH);
			return true;
		}

		return false;
}
