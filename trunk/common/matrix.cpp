#include "matrix.h"
#include <stdio.h>
namespace mymatrix
{

matrix2x2 sub_matrix3x3(matrix3x3 in, int x, int y)
{
	matrix2x2 out;
	float *p = (float*) &in;
	float *o = (float*) &out;

	int yy = 0;
	for(int iy=0; iy<3; iy++)
	{
		if (y != iy+1)
		{
			int xx = 0;
			for(int ix=0; ix<3; ix++)
			{
				if (x != ix+1)
				{
					o[yy*2+xx] = p[iy*3+ix];
					xx++;
				}
			}

			yy ++;
		}
	}

	return out;
}

matrix3x3 multiply_matrix3x3(matrix3x3 a, matrix3x3 b)
{
	matrix3x3 out;
	float *aa = (float*) &a;
	float *bb = (float*) &b;
	float *o = (float*) &out;

	for(int x1 = 0; x1<3; x1++)
	{
		for(int y1 = 0; y1<3; y1++)
		{
			o[y1*3+x1] = 0;
			for(int x2 = 0; x2<3; x2++)
				o[y1*3+x1] += bb[x2*3+x1] * aa[y1*3+x2];
		}
	}
	return out;
}

vector3 multiply_matrix3x3(matrix3x3 m, vector3 v)
{
	matrix3x3 m2 = {v.a1, 0, 0, v.a2, 0, 0, v.a3, 0, 0};

	matrix3x3 tmp = multiply_matrix3x3(m, m2);

	vector3 o = {tmp.a11, tmp.a21, tmp.a31};

	return o;
}
matrix3x3 divide_matrix3x3(matrix3x3 in, float divider)
{
	matrix3x3 out;
	float *p = (float*) &in;
	float *o = (float*) &out;

	for(int i=0; i<9; i++)
		o[i] = p[i] / divider;

	return out;		
}

matrix3x3 reverse_matrix3x3(matrix3x3 in)
{
	matrix3x3 out;
	float *o = (float*) &out;

	for(int y=0; y<3; y++)
		for(int x=0; x<3; x++)
		{
			int daishu = x+1 + y+1;
			int symbol = daishu % 2 == 1 ? -1 : 1;
			matrix2x2 a_star = sub_matrix3x3(in, x+1, y+1);
			float det_a_star = det_matrix2x2(a_star);
			o[x*3+y] =  det_a_star * symbol;
		}

	out = divide_matrix3x3(out, det_matrix3x3(in));

	return out;
}



float det_matrix2x2(matrix2x2 in)
{
	return in.a11 * in.a22 - in.a12 * in.a21;
}

float det_matrix3x3(matrix3x3 in)
{
	float o = 0;
	for(int x=0; x<3; x++)
	{
		float a = det_matrix2x2(sub_matrix3x3(in, x+1, 1));
		int symbol = (x+1) % 2 == 1 ? 1 : -1;

		o += a * symbol * ((float*)&in)[x];
	}

	return o;
}












matrix sub_matrix(matrix in, int x, int y)
{
	matrix out;
	out.n = in.n-1;
	out.n = in.n-1;
	float *p = (float*) in.data;
	float *o = (float*) out.data;

	int yy = 0;
	for(int iy=0; iy<in.n; iy++)
	{
		if (y != iy+1)
		{
			int xx = 0;
			for(int ix=0; ix<in.n; ix++)
			{
				if (x != ix+1)
				{
					o[yy*out.n+xx] = p[iy*in.n+ix];
					xx++;
				}
			}

			yy ++;
		}
	}

	return out;
}

matrix multiply_matrix(matrix a, matrix b)
{
	matrix out;
	out.n = a.n;
	out.n = a.n;
	float *aa = (float*) a.data;
	float *bb = (float*) b.data;
	float *o = (float*) out.data;

	for(int x1 = 0; x1<out.n; x1++)
	{
		for(int y1 = 0; y1<out.n; y1++)
		{
			o[y1*out.n+x1] = 0;
			for(int x2 = 0; x2<out.n; x2++)
				o[y1*out.n+x1] += bb[x2*out.n+x1] * aa[y1*out.n+x2];
		}
	}
	return out;
}

vector multiply_matrix(matrix m, vector v)
{
	matrix m2 = {0};
	m2.n = m2.n = v.n;
	
	for(int i=0; i<v.n; i++)
		m2.data[m2.n*i] = v.data[i];

	matrix tmp = multiply_matrix(m, m2);


	for(int i=0; i<v.n; i++)
		v.data[i] = tmp.data[tmp.n*i];

	return v;
}
matrix divide_matrix(matrix in, float divider)
{
	float *p = (float*) in.data;

	for(int i=0; i<in.n*in.n; i++)
		p[i] /= divider;

	return in;
}

matrix reverse_matrix(matrix in)
{
	matrix out = in;
	float *o = (float*) out.data;

	for(int y=0; y<in.n; y++)
		for(int x=0; x<in.n; x++)
		{
			int daishu = x+1 + y+1;
			int symbol = daishu % 2 == 1 ? -1 : 1;
			matrix a_star = sub_matrix(in, x+1, y+1);
			float det_a_star = det_matrix(a_star);
			o[x*in.n+y] =  det_a_star * symbol;
		}

	out = divide_matrix(out, det_matrix(in));

	return out;
}

void print_matrix(matrix m)
{
	for(int y=0; y<m.n; y++)
	{
		for(int x=0; x<m.n; x++)
		{
			printf("%f,", m.data[y*m.n+x]);
		}
		printf("\n");
	}
}


float det_matrix(matrix in)
{
	if (in.n == 2)
		return in.data[0] * in.data[3] - in.data[1] * in.data[2];

	if (in.n == 1)
		return in.data[0];

	float det = 0;
	for(int i=0; i<in.n; i++)
	{
		int symbol = i % 2 == 0 ? 1 : -1;
		matrix a_star = sub_matrix(in, i+1, 1);
		float det_a_star = det_matrix(a_star);
		det += det_a_star * symbol * in.data[i];
	}

	return det;
}

}
