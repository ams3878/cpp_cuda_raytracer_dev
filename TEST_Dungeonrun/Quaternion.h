#pragma once
#include "platform_common.h"
class Quaternion
{
	int set_rot_matrix();
	cudaError_t set_rot_matrix_CUDA();

public:
	struct quaternion_vec { T_fp* w, * i, * j, * k; quaternion_vec() :w(NULL), i(NULL), j(NULL), k(NULL) {} }h_vec, d_vec;
	T_fp* h_rot_m, *d_rot_m;
	u32 size;
	Quaternion();
	Quaternion(u32 size);
	Quaternion(u32 size, int constructor_flags);

	int _free();
	cudaError_t initialize_CUDA();
	cudaError_t _memset_CUDA();
	int set_rot_matrix(int flags);

	int _memset(Quaternion::quaternion_vec v);
	int _memset(T_fp* _w, T_fp* _i, T_fp* _j, T_fp* _k);
	int _memset(int value);

};

