#ifndef __VECTOR_H__
#define __VECTOR_H__


typedef union
{
	float array[3];
	struct
	{
		float x;
		float y;
		float z;
	}V;
}vector;

#ifdef __cplusplus
extern "C" {
#endif
	
void vector_add(vector *a, vector *b);		// a = a+b
void vector_sub(vector *a, vector *b);		// a = a-b
void vector_divide(vector *a, float b);		// a = a/b
void vector_multiply(vector *a, float b);	// a = a*b
void vector_rotate(vector *v, float *delta);
float vector_length(vector *v);
float vector_angle(vector *v1, vector *v2);	// return cos(angle(v1, v2))
void vector_normalize(vector *v);

#ifdef __cplusplus
}
#endif

#endif
