#include "matrix.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

static void matrix_error(const char*msg)
{

}


matrix::matrix()
{
	n = m =0;		// invalid matrix
}

matrix::matrix(int d)
{
	n = m = d;
	for(int i=0; i<d; i++)
		data[d*i+i] = 1.0f;
}

matrix::matrix(const matrix &v)
{
	n = v.n;
	m = v.m;
	memcpy(data, v.data, sizeof(float)*n*m);
}

matrix::~matrix()
{

}

void matrix::operator =(const matrix &v)
{
	n = v.n;
	m = v.m;
	memcpy(data, v.data, sizeof(float)*n*m);
}

void matrix::operator +=(const matrix &v)
{
	assert(n == v.n && m == v.m);
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] += v.data[i];
}
matrix matrix::operator +(const matrix &v)
{
	matrix o(*this);
	o += v;
	return o;
}
void matrix::operator -=(const matrix &v)
{
	assert(n == v.n && m == v.m);
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] -= v.data[i];
}
matrix matrix::operator -(const matrix &v)
{
	matrix o(*this);
	o -= v;
	return o;
}
void matrix::operator *=(const matrix &v)
{
	assert(n == v.m);	// invalid matrix_mul
	matrix o;
	o.m = m;
	o.n = v.n;

	for(int x1 = 0; x1<v.n; x1++)
	{
		for(int y1 = 0; y1<m; y1++)
		{
			o.data[y1*v.n+x1] = 0;
			for(int k = 0; k<n; k++)
				o.data[y1*v.n+x1] += data[y1*n+k] * v.data[k*v.n+x1];
		}
	}

	*this = o;
}
void matrix::operator *=(const float &v)
{
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] *= v;
}
matrix matrix::operator *(const matrix &v)
{
	matrix o(*this);
	o *= v;
	return o;
}
matrix matrix::operator *(const float &v)
{
	matrix o(*this);
	o *= v;
	return o;
}

float matrix::det()
{
	if (n == 2)
		return data[0] * data[3] - data[1] * data[2];

	if (n == 1)
		return data[0];

	float det = 0;
	for(int i=0; i<n; i++)
	{
		int symbol = i % 2 == 0 ? 1 : -1;
		matrix a_star = cofactor(i+1, 1);
		det += a_star.det() * symbol * data[i];
	}

	return det;
}

matrix matrix::inverse()
{
	assert(m==n);
	matrix out(*this);
	float *o = (float*) out.data;
	for(int y=0; y<n; y++)
		for(int x=0; x<n; x++)
		{
			int daishu = x+1 + y+1;
			int symbol = daishu % 2 == 1 ? -1 : 1;
			matrix a_star = cofactor(x+1, y+1);
			o[x*n+y] =  a_star.det() * symbol;
		}

	out /= det();

	return out;
}

matrix matrix::cofactor(int x, int y)
{
	matrix out;
	out.n = n-1;
	out.m = m-1;

	float *p = (float*) data;
	float *o = (float*) out.data;

	int yy = 0;
	for(int iy=0; iy<n; iy++)
	{
		if (y != iy+1)
		{
			int xx = 0;
			for(int ix=0; ix<n; ix++)
			{
				if (x != ix+1)
				{
					o[yy*out.n+xx] = p[iy*n+ix];
					xx++;
				}
			}

			yy ++;
		}
	}

	return out;

}


void matrix::operator /=(const matrix &v)
{
	assert(v.n == v.m);
	matrix tmp(v);
	*this *= tmp.inverse();
}

matrix matrix::operator /(const matrix &v)
{
	matrix o(*this);
	o /= v;
	return o;
}
void matrix::operator /=(const float &v)
{
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] /= v;
}

matrix matrix::operator /(const float &v)
{
	matrix o(*this);
	o /= v;
	return o;
}