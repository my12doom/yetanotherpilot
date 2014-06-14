#pragma once

#define MAX_PARAM_COUNT 128

class param
{
public:
	param();
	param(const char *fourcc, float default_value);
	~param();
	void init(const char *fourcc, float default_value);
	operator float();
	float* operator& ();
	float& operator= (float in);		// ram operation only
	void save();						// save to eeprom

	static param *find_param(const char *fourcc);
protected:

	float* volatile pv;
	int pos;		// pos in all_params array

private:
	void init_all();
};