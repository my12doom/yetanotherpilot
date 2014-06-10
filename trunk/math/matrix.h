#pragma once

#define MAX_MATRIX_ELEMENTS 256

class matrix
{
public:
	int m;		// m rows
	int n;		// n columns
	float data[MAX_MATRIX_ELEMENTS];		// m * n matrix, row first

	matrix();
	matrix(int d);		// d*d identity matrix
	matrix(const matrix&v);		// copy constructor
	~matrix();

	void operator =(const matrix &v);
	void operator +=(const matrix &v);
	matrix operator +(const matrix &v);
	void operator -=(const matrix &v);
	matrix operator -(const matrix &v);
	void operator *=(const matrix &v);
	matrix operator *(const matrix &v);
	void operator *=(const float &v);
	matrix operator *(const float &v);
	void operator /=(const float &v);
	void operator /=(const matrix &v);
	matrix operator /(const matrix &v);
	matrix operator /(const float &v);
	matrix inverse();
	float det();
	matrix cofactor(int a, int b);

private:
	float det2x2();
	float det3x3();
};

namespace mymatrix
{
typedef struct
{
	float a11, a12, a13;
	float a21, a22, a23;
	float a31, a32, a33;
} matrix3x3;
typedef struct
{
	float a11, a12;
	float a21, a22;
} matrix2x2;

typedef struct
{
	float a1,a2,a3;
} vector3;

matrix3x3 reverse_matrix3x3(matrix3x3 in);

float det_matrix2x2(matrix2x2 in);

float det_matrix3x3(matrix3x3 in);

matrix2x2 sub_matrix3x3(matrix3x3 in, int x, int y);

matrix3x3 multiply_matrix3x3(matrix3x3 a, matrix3x3 b);
vector3 multiply_matrix3x3(matrix3x3 m, vector3 v);

matrix3x3 divide_matrix3x3(matrix3x3 in, float divider);

typedef struct
{
	int n;		// n columns
	int m;		// m rows
	float data[MAX_MATRIX_ELEMENTS];	// m * n matrix
}matrix;

typedef struct
{
	int n;
	float data[MAX_MATRIX_ELEMENTS];
} vector;

matrix reverse_matrix(matrix in);

float det_matrix(matrix in);

matrix sub_matrix(matrix in, int x, int y);

matrix multiply_matrix(matrix a, matrix b);
vector multiply_matrix(matrix m, vector v);

matrix divide_matrix3x3(matrix in, float divider);
}
