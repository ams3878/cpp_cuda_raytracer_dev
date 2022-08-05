#pragma once
#ifndef _QUATERNION_H_
#define _QUATERNION_H_
#include "Vector.h"
class Quaternion
{
private:
	cudaError_t initialize_CUDA(VEC4<T_fp>* init_x, VEC4<T_fp>* init_y, VEC4<T_fp>* init_z);
	bool isCUDA;

public:	
	VEC4<VEC4<T_fp>*>*rot_m, *d_rot_m;
	VEC4<T_fp>* vec, *d_vec;

	Quaternion();
	Quaternion(int constructor_flags);
	Quaternion(const Quaternion &q);
	
	T_fp _i(T_uint index) const;	T_fp _j (T_uint index) const;	T_fp _k(T_uint index) const; 	T_fp _w(T_uint index) const;
	cudaError_t set_device_rotation(VEC4<VEC4<T_fp>*>* host_rot_matrix);//only call on device matrixes
	int set_transformation_matrix_rot();
	int set_transformation_matrix_tran(VEC4<T_fp> delta);

};

#endif
