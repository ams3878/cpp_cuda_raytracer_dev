#pragma once
#ifndef _QUATERNION_H_
#define _QUATERNION_H_
#include "Vector.h"
class Quaternion
{
private:
	cudaError_t initialize_CUDA();
	bool isCUDA;

public:	
	VEC4_CUDA<VEC4_CUDA<T_fp>>*rot_m, *d_rot_m;
	u32 size;
	VEC4_CUDA<T_fp>* vec, *d_vec;

	Quaternion();
	Quaternion(u32 size);
	Quaternion(u32 size, int constructor_flags);
	
	T_fp _i(T_uint index) const;	T_fp _j (T_uint index) const;	T_fp _k(T_uint index) const; 	T_fp _w(T_uint index) const;
	int _free(int flags);

	int set_transformation_matrix_rot();
	int set_transformation_matrix_tran(VEC4<T_fp> delta);
	int _memset(VEC4<T_fp>* v);
	int _memset(T_fp* _w, T_fp* _i, T_fp* _j, T_fp* _k);
	int _memset(int value);
};
#endif
