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
#include "common.h"

#define ACC_Z_1G -2048
#define GYRO_MAX_BIAS 160				// ~1.2degree/s, if we have bias more than this, then calibration warning is shown
#define ACCEL_STABLE_THRESHOLD  205		// ~0.1g, if we have vibration more than this, we consider it vibrating
#define TILT_MAX_BIAS 200				// if we have tilt more than this, then calibration warning is shown

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
float stable_accel[3] = {0, 0, 0};
float stable_gyro[3] = {0, 0, 0};
int last_stable_tick = 0x7fffffff;
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
		else
			continue;



		goto imu_read_ok;
fail:
		continue;
imu_read_ok:

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
			swprintf(str, L"%s%01.2f °/s", v>=0?"+":"", v);
			SetDlgItemTextW(hWnd, IDtable[i], str);

			swprintf(str, L"%.1f°C", mpu6050_temperature);
			SetDlgItemTextW(hWnd, IDC_TEMP, str);

			// client side accel low pass for tile display
			client_accel[i] = client_accel[i] * 0 + imu_statics[0][1].array[i] * 1;

			// calculate stable accelerometer/gyrometer readings
			bool stable = true;
			for(int i=0; i<3; i++)
			{
				if (fabs(stable_accel[i] - imu_statics[0][1].array[i]) > ACCEL_STABLE_THRESHOLD)
					stable = false;
				if (fabs(stable_gyro[i] - imu_statics[1][1].array[i]) > GYRO_MAX_BIAS/3)
					stable = false;
			}

			
			if (stable)
			{
				// do nothing ,reserve latest value
			}
			else
			{
				last_stable_tick = GetTickCount();
				for(int i=0; i<3; i++)
				{
					stable_accel[i] = imu_statics[0][1].array[i];
					stable_gyro[i] = imu_statics[1][1].array[i];
				}
			}

			// show IMU calibration result
			if (GetTickCount() - last_stable_tick > 1000)
			{
				vector accel = {stable_accel[0], stable_accel[1], stable_accel[2]};
				vector tilt_reading = {0};
				accel_vector_to_euler_angle(accel, &tilt_reading);

				bool tilt_ok = fabs(tilt_reading.array[0] + trim[0]) < 5.0f*PI/180 && fabs(tilt_reading.array[1] + trim[1]) < 5.0f*PI/180;
				bool gyro_ok = fabs(stable_gyro[0] - gyro_zero_raw[0]) < GYRO_MAX_BIAS && fabs(stable_gyro[1] - gyro_zero_raw[1]) < GYRO_MAX_BIAS && fabs(stable_gyro[2] - gyro_zero_raw[2]) < GYRO_MAX_BIAS;

				float total_gforce = sqrt(stable_accel[0]*stable_accel[0]+stable_accel[1]*stable_accel[1]+stable_accel[2]*stable_accel[2])/2048.0f;
				if (fabs(total_gforce - 1.0f) > 0.5f)
					SetDlgItemTextW(hWnd, IDC_IMUSTATE, L"传感器有显著问题，请校准");
				else if (tilt_ok && gyro_ok)
					SetDlgItemTextW(hWnd, IDC_IMUSTATE, L"传感器状态良好");
				else
					SetDlgItemTextW(hWnd, IDC_IMUSTATE, L"传感器需要校准");

			}
			else
			{
				SetDlgItemTextW(hWnd, IDC_IMUSTATE, L"请将装有飞控的飞机放在水平面上以获得传感器状态");
			}				

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
				bool save = true;

				// check for vibration
				float vibration[6];
				for(int i=0; i<6; i++)
				{
					vibration[i] = imu_statics[i/3][2].array[i%3] - imu_statics[i/3][0].array[i%3];
				}
				float accel_vibration = sqrt(vibration[0]*vibration[0]+vibration[1]*vibration[1]+vibration[2]*vibration[2]);
				float gyro_vibration = sqrt(vibration[3]*vibration[3]+vibration[4]*vibration[4]+vibration[5]*vibration[5]);

				if (accel_vibration > ACCEL_STABLE_THRESHOLD/* || gyro_vibration > GYRO_MAX_BIAS*/)	
				{
					MessageBoxW(hWnd, L"校准时请保持飞控处于静止状态!", L"", MB_OK | MB_ICONERROR);
					save = false;
				}

				// calculate
				float dz = ACC_Z_1G - imu_statics[0][3].array[2];
				acc_bias_z += dz;

				// check for extreme tilt
				vector accel = {imu_statics[0][3].array[0], imu_statics[0][3].array[1], imu_statics[0][3].array[2]+dz};
				vector new_trim = {0};
				accel_vector_to_euler_angle(accel, &new_trim);

				if (fabs(new_trim.array[0]) > PI/18 || fabs(new_trim.array[1]) > PI/18)
				{
					// TODO : error message
					MessageBoxW(hWnd, L"检测到飞控总倾斜角度大于10度，请将飞机水平放置再进行校准。", L"", MB_OK | MB_ICONERROR);
					save = false;
				}
				
				trim[0] = -new_trim.array[0];
				trim[1] = -new_trim.array[1];
				


				// save
				if (save)
				{
					write_gyro_bias(to);
// 					swprintf(test, L"%f,%f,%f,%f", imu_statics[1][3].array[0], imu_statics[1][3].array[1], imu_statics[1][3].array[2], mpu6050_temperature);
// 					MessageBoxW(hWnd, test, L"info", MB_OK);
				}

				SetDlgItemTextW(hWnd, IDC_CAL_T1, L"开始校准");
				SetDlgItemTextW(hWnd, IDC_CAL_T2, L"开始校准");
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
	HANDLE_CTLCOLORSTATIC;
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
				if (MessageBoxW(hWnd, L"请将飞机平稳放置在水平面上，然后点击确定开始校准传感器", L"平放", MB_OKCANCEL) == IDOK)
				{
					int previous_avg_count = avg_count;		
					int tick = GetTickCount();
					while (previous_avg_count <= avg_count && GetTickCount()-tick<10000)
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
						MessageBoxW(hWnd, L"校准失败，请确定飞控与电脑连接良好", L"失败", MB_OK | MB_ICONERROR);
						to = 0;
					}
					calibrating = to;
				}
			}
		}
		break;

	default:
		return FALSE;
	}
	return TRUE;
}