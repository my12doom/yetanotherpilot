#pragma once

#define MAX_PARAM_COUNT 128

#ifdef __cplusplus
class param
{
public:
	param();
	param(const char *fourcc, float default_value);
	~param();
	void init(const char *fourcc, float default_value);

	inline operator float()
	{
		return *pv;
	}
	float* operator& ()
	{
		return pv;
	}
	inline float& operator= (float in)		// ram operation only
	{
		return *pv = in;
	}
	void save();						// save to eeprom

	static float *find_param(const char *fourcc);
	static const char *enum_params(int pos);
	const char *fourcc();
protected:

	float* volatile pv;
	int pos;		// pos in all_params array

private:
	void init_all();
};

extern "C"
{
#endif

float * find_param(const char *fourcc);
float * create_param(const char *fourcc, float default_value);
int save_param(const char *fourcc);

#ifdef __cplusplus
}
#endif