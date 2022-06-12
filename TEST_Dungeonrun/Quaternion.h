#pragma once
#include "platform_common.h"
#include "Vector.h"

class Quaternion
{
	int set_rot_matrix();
	cudaError_t set_rot_matrix_CUDA();
	cudaError_t set_rot_matrix_CUDA(VEC3_CUDA<VEC3_CUDA<T_fp>>* _rot_m);
	cudaError_t initialize_CUDA();
	cudaError_t _memset_CUDA(VEC4_CUDA<T_fp>* v);
	bool isCUDA;

public:	
	VEC3_CUDA<VEC3_CUDA<T_fp>>*rot_m, *d_rot_m;
	/*
	struct rotation_matrix {
		VEC3_CUDA<T_fp> x, y, z;
		rotation_matrix(T_uint s) {
			x = VEC3_CUDA<T_fp>(s);
			y = VEC3_CUDA<T_fp>(s);
			z = VEC3_CUDA<T_fp>(s);
		};
	} *rot_m;*/
	u32 size;
	VEC4_CUDA<T_fp>* vec, *d_vec;

	Quaternion();
	Quaternion(u32 size);
	Quaternion(u32 size, int constructor_flags);
	
	T_fp _i(T_uint index) const;	T_fp _j (T_uint index) const;	T_fp _k(T_uint index) const; 	T_fp _w(T_uint index) const;
	int _free(int flags);

	int set_rot_matrix(Quaternion q);

	int _memset(VEC4_CUDA<T_fp>* v);
	int _memset(T_fp* _w, T_fp* _i, T_fp* _j, T_fp* _k);
	int _memset(int value);

};

