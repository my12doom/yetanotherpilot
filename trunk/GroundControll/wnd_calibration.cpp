#include "wnd_remote.h"
#include "comm.h"
#include "resource.h"
#include <CommCtrl.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include "../common/vector.h"
#include "../common/build.h"
#include "OwnerDraw.h"

#define ACC_Z_1G -2048

extern Comm test;
vector imu_statics[2][4] = {0};		//	[accel, gyro][min, current, max, avg][x,y,z]
int avg_count = 0;
float mpu6050_temperature;
int calibrating = 0;

static float gyro_bias[2][4];	//[p1,p2][temperature,g0,g1,g2]
static float k[3];
static float a[3];
float temperature0 = 0;
float trim[2];
float acc_bias_z = 0;

float client_accel[3] = {0, 0, 0};
vector client_tilt;

HWND hWnd;

#ifdef WIN32
#define isnan(x) (_isnan(x) || !_finite(x))
#endif

char name_table[11][5] =
{
	"gbt1", "gb11", "gb21", "gb31",
	"gbt2", "gb12", "gb22", "gb32",
	"trmR", "trmP",
	"abiz"
};

float *variable_table[] = 
{
	&gyro_bias[0][0], &gyro_bias[0][1], &gyro_bias[0][2], &gyro_bias[0][3],
	&gyro_bias[1][0], &gyro_bias[1][1], &gyro_bias[1][2], &gyro_bias[1][3],
	&trim[0], &trim[1],
	&acc_bias_z,
};

void DoEvents()  
{  
	MSG msg;  
	while(PeekMessage(&msg,NULL,0,0,PM_REMOVE))  
	{  
		DispatchMessage(&msg);  
		TranslateMessage(&msg);  
	}
}  

int read_gyro_bias()
{
	char cmd[200];
	char output[20480] = {0};
	for(int i=0; i<11; i++)
	{
		memset(output, 0, sizeof(output));
		sprintf(cmd, "?%s\n", name_table[i]);
		if (test.command(cmd, strlen(cmd), output) <= 0)
			return -1;

		if (sscanf(output, "%f", variable_table[i]) != 1)
			return -2;
	}

	// update k & a
	if (!isnan((float)gyro_bias[0][0]) && !isnan((float)gyro_bias[1][0]))
	{
		float dt = gyro_bias[1][0] - gyro_bias[0][0];
		if (dt > 1.0f)
		{
			for(int i=0; i<3; i++)
			{
				a[i] = gyro_bias[0][i+1];
				k[i] = (gyro_bias[1][i+1] - gyro_bias[0][i+1]) / dt;
			}
			temperature0 = gyro_bias[0][0];
		}
		else
		{
			// treat as one point
			int group = !isnan(gyro_bias[0][0]) ? 0 : (!isnan(gyro_bias[1][0]) ? 1: -1);
			for(int i=0; i<3; i++)
			{
				a[i] = group >= 0 ? gyro_bias[group][i+1] : 0;
				k[i] = 0;
			}
			temperature0 = group != 0 ? gyro_bias[group][0] : 0;
		}
	}
	else
	{
		int group = !isnan(gyro_bias[0][0]) ? 0 : (!isnan(gyro_bias[1][0]) ? 1: -1);
		for(int i=0; i<3; i++)
		{
			a[i] = group >= 0 ? gyro_bias[group][i+1] : 0;
			k[i] = 0;
		}
		temperature0 = group != 0 ? gyro_bias[group][0] : 0;
	}


	return 0;
}

int write_gyro_bias(int to)
{
	float v[] = {mpu6050_temperature, imu_statics[1][3].array[0], imu_statics[1][3].array[1], imu_statics[1][3].array[2]};

	char cmd[200];
	char output[20480] = {0};
	for(int i=0; i<4; i++)
	{
		memset(output, 0, sizeof(output));
		sprintf(cmd, "%s=%f\n", name_table[i+(to-1)*4], v[i]);
		if (test.command(cmd, strlen(cmd), output) <= 0)
			return -1;

		if (strstr(output, "ok") != output)
			return -2;
	}

	for(int i=0; i<3; i++)
	{
		memset(output, 0, sizeof(output));
		sprintf(cmd, "%s=%f\n", name_table[i+8], *variable_table[8+i]);
		if (test.command(cmd, strlen(cmd), output) <= 0)
			return -1;

		if (strstr(output, "ok") != output)
			return -2;
	}

	read_gyro_bias();

	return 0;
}
int calibration_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
		read_gyro_bias();

	return 0;
}

DWORD CALLBACK calibration_update_thread(LPVOID p)
{
while(true)
	{
		char cmd[] = "imustates\n";
		char output[20480] = {0};
		char *p = output;

		int len = test.command(cmd, strlen(cmd), output);

		// parse result
		p = strstr(p, "imu:");
		if (p)
		{
			p+= 4;
			for(int i=0; i<2; i++)
			{
				for(int j=0; j<4; j++)
				{
					for(int k=0; k<3; k++)
					{
						sscanf(p, "%f,", &imu_statics[i][j].array[k]);
						p = strchr(p, ',');
						if (!p)
							goto fail;
						p++;
					}
				}
			}

			sscanf(p, "%f,%d,", &mpu6050_temperature, &avg_count);
		}



fail:


		// update
		DWORD IDtable[] = {IDC_GYRO0, IDC_GYRO1, IDC_GYRO2};
		float dt = mpu6050_temperature - temperature0;
		float gyro_zero_raw[3] = 
		{
			dt * k[0] + a[0],
			dt * k[1] + a[1],
			dt * k[2] + a[2],
		};
		bool log = false;
		char gyro_str[200] = {0};
		for(int i=0; i<3;i ++)
		{
			wchar_t str[100];
			float v = (imu_statics[1][1].array[i] - gyro_zero_raw[i]) * 250 / 32767;
			v = fabs(v) < 0.07 ? 0 : v;
			sprintf(gyro_str+strlen(gyro_str), "%f,", v);
			if (v>0)
				log = true;
			swprintf(str, L"%s%01.2f ¡ã/s", v>=0?"+":"", v);
			SetDlgItemTextW(hWnd, IDtable[i], str);

			swprintf(str, L"%.1f¡ãC", mpu6050_temperature);
			SetDlgItemTextW(hWnd, IDC_TEMP, str);

			// client side accel low pass for tile display
			client_accel[i] = client_accel[i] * 0 + imu_statics[0][1].array[i] * 1;
		}

		if(log)
		{
			OutputDebugStringA(gyro_str);
			OutputDebugStringA("\n");
		}



		float bias_client_accel[3] = {client_accel[0] - (137-109)/2, client_accel[1] - (-109+102)/2, client_accel[2] - (-2200+1936)/2};
		memcpy(bias_client_accel, client_accel, sizeof(client_accel));
		wchar_t str[100];
		vector accel = {client_accel[0], client_accel[1], client_accel[2]};
		accel_vector_to_euler_angle(accel, &client_tilt);
		float g = sqrt(bias_client_accel[0]*bias_client_accel[0] + bias_client_accel[1]*bias_client_accel[1] + bias_client_accel[2]*bias_client_accel[2]);
		swprintf(str, L"%f\n%f\n%f\n%f\n%f\n%f\n%f\n%fg\n"
			L"gyro:%.1f,%.1f,%.1f\n", client_accel[0], client_accel[1], client_accel[2],
			(client_tilt.array[0]+trim[0])*180/PI, (client_tilt.array[1]+trim[1])*180/PI,
			(trim[0])*180/PI, (trim[1])*180/PI, g/2048,
			imu_statics[1][1].array[0], imu_statics[1][1].array[1], imu_statics[1][1].array[2]);
		SetDlgItemTextW(hWnd, IDC_ACCEL, str);


		// calibrating
		if (calibrating)
		{
			wchar_t test[2048];
			swprintf(test, L"%d%%", min(avg_count/10, 100));
			SetDlgItemTextW(hWnd, calibrating == 1 ? IDC_CAL_T1 : IDC_CAL_T2, test);

			if (avg_count > 1000)
			{
				int to = calibrating;
				calibrating = 0;
				SetDlgItemTextW(hWnd, calibrating == 1 ? IDC_CAL_T1 : IDC_CAL_T2, calibrating == 1 ? L"Cal1" : L"Cal2");

				// check for vibration
				float vibration[6];
				for(int i=0; i<6; i++)
				{
					vibration[i] = imu_statics[i/3][2].array[i%3] - imu_statics[i/3][0].array[i%3];
				}
				float accel_vibration = sqrt(vibration[0]*vibration[0]+vibration[1]*vibration[1]+vibration[2]*vibration[2]);
				float gyro_vibration = sqrt(vibration[3]*vibration[3]+vibration[4]*vibration[4]+vibration[5]*vibration[5]);

				if (accel_vibration > 204 || gyro_vibration > 160)		// 0.1g, 10degree/s
				{
					MessageBoxW(hWnd, L"stay still please!", L"", MB_OK | MB_ICONERROR);
				}

				// calculate
				float dz = ACC_Z_1G - imu_statics[0][3].array[2];
				acc_bias_z += dz;

				// check for extreme tilt
				vector accel = {imu_statics[0][3].array[0], imu_statics[0][3].array[1], imu_statics[0][3].array[2]+dz};
				vector new_trim = {0};
				if (accel_vector_to_euler_angle(accel, &new_trim) < 0)
				{
					// TODO : error message
				}

				if (fabs(new_trim.array[0]) > PI/18 || fabs(new_trim.array[1]) > PI/18)
				{
					// TODO : error message
					MessageBoxW(hWnd, L"stay flat please!", L"", MB_OK | MB_ICONERROR);
				}
				
				trim[0] = -new_trim.array[0];
				trim[1] = -new_trim.array[1];
				


				// save
				write_gyro_bias(to);
				swprintf(test, L"%f,%f,%f,%f", imu_statics[1][3].array[0], imu_statics[1][3].array[1], imu_statics[1][3].array[2], mpu6050_temperature);
				MessageBoxW(hWnd, test, L"info", MB_OK);
			}
		}

		Sleep(50);
	}

	return 0;
}

INT_PTR CALLBACK WndProcCalibration(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_LBUTTONDOWN:
		SendMessage(GetParent(GetParent(hWnd)), WM_NCLBUTTONDOWN, HTCAPTION, 0);
		break;
	case WM_INITDIALOG:
		::hWnd = hWnd;
		CreateThread(NULL, NULL, calibration_update_thread, NULL, NULL, NULL);

		test.add_callback(calibration_OnEvent);
		SetTimer(hWnd, 1, 25, NULL);
		read_gyro_bias();
		break;

	case WM_PAINT:
		return paint_white(hWnd, wParam, lParam);
		break;

	case WM_DESTROY:
		KillTimer(hWnd, 1);
		break;
	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;

	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			int to = 0;
			if (id == IDC_CAL_T1)
			{
				to = 1;
			}
			else if (id == IDC_CAL_T2)
			{
				to = 2;
			}
			else
			{
			}

			if (to>0)
			{
				int previous_avg_count = avg_count;		
				int tick = GetTickCount();
				while (previous_avg_count <= avg_count && GetTickCount()-tick<1000)
				{
					char cmd[] = "imureset\n";
					char output[20480] = {0};
					char *p = output;

					int len = test.command(cmd, strlen(cmd), output);
					if (strstr(output,"ok") != output)
					{
						to = -1;
						break;
					}
					DoEvents();
					Sleep(1);
				}
				if (previous_avg_count<= avg_count)
				{
					// TODO: failed
					to = 0;
				}
				calibrating = to;
			}
		}
		break;

	default:
		return FALSE;
	}
	return TRUE;
}