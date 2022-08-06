#pragma once
#ifndef CUDA_VECTOR_H
#define CUDA_VECTOR_H
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#include "typedefs.h"
#include <typeinfo>

#define MOLLER_TRUMBORE_DEVICE_EPSILON 1e-16
#define DEVICE_EPSILON_SINGLE 1e-16
//This can be changed for release mode to bigger, but voxel traversal too many threads in debug mode
#define BLOCK_SIZE 512 

#define status_lauch_and_sync(a) cudaStatus = cudaGetLastError();\
if (cudaStatus != cudaSuccess) {printf("%s launch failed: %s\n", #a, cudaGetErrorString(cudaStatus));}\
cudaStatus = cudaDeviceSynchronize();\
if (cudaStatus != cudaSuccess) {printf("cudaDeviceSynchronize returned error code %d after launching %s!\n",cudaStatus, #a);};

 /*
 template<typename T>
 __device__ VEC4<T>::VEC4(const u8 arbitrary_null_arg) {  };*/
 template<typename T>
 template <typename h_T>
 __device__ void VEC3_CUDA<T>::device_rotate(h_T* rm, T_uint m_index, int reverse) { 
	 T temp_x = reverse * x[m_index], temp_y = reverse * y[m_index], temp_z = reverse * z[m_index];
	 x[m_index] = temp_x * rm->x->i + temp_y * rm->x->j + temp_z * rm->x->k;
	 y[m_index] = temp_x * rm->y->i + temp_y * rm->y->j + temp_z * rm->y->k;
	 z[m_index] = temp_x * rm->z->i + temp_y * rm->z->j + temp_z * rm->z->k;
	 x[m_index] *= reverse;
	 y[m_index] *= reverse;
	 z[m_index] *= reverse;
 };

 template<typename T>
 template <typename h_T>
 __device__ void VEC3<T>::device_rotate(h_T* rm ,T_uint index) {
	 T temp_x = x, temp_y = y, temp_z = z;
	 x = temp_x * rm->x->i[index] + temp_y * rm->x->j[index] + temp_z * rm->x->k[index];
	 y = temp_x * rm->y->i[index] + temp_y * rm->y->j[index] + temp_z * rm->y->k[index];
	 z = temp_x * rm->z->i[index] + temp_y * rm->z->j[index] + temp_z * rm->z->k[index];
 };
 template<typename T>
 template <typename h_T>
 __device__ void VEC4<T>::device_rotate(h_T qx, h_T qy, h_T qz) {
	 T temp_x = x, temp_y = y, temp_z = z;
	 x = temp_x * qx->i + temp_y * qx->j + temp_z * qx->k;
	 y = temp_x * qy->i + temp_y * qy->j + temp_z * qy->k;
	 z = temp_x * qz->i + temp_y * qz->j + temp_z * qz->k;
 };
 template <typename T>
 __device__ void quaternion_mul(VEC4<T>* a, VEC4<T> b) {
	 T t_i = a->i;	 T t_j = a->j;	 T t_k = a->k;	 T t_w = a->w;

	 a->i = t_j * b.z - t_k * b.y + t_i * b.w + t_w * b.x;
	 a->j = t_k * b.x - t_i * b.z + t_j * b.w + t_w * b.y;
	 a->k = t_i * b.y - t_j * b.x + t_k * b.w + t_w * b.z;
	 a->w = t_w * b.w - t_i * b.x - t_j * b.y - t_k * b.z;
 }

 template <typename T>
 __device__ void quaternion_mul(VEC4<T>* a, VEC4<T>* b) {
	 T t_i = a->i;	 T t_j = a->j;	 T t_k = a->k;	 T t_w = a->w;

	 a->i = t_j * b->z - t_k * b->y + t_i * b->w + t_w * b->x;
	 a->j = t_k * b->x - t_i * b->z + t_j * b->w + t_w * b->y;
	 a->k = t_i * b->y - t_j * b->x + t_k * b->w + t_w * b->z;
	 a->w = t_w * b->w - t_i * b->x - t_j * b->y - t_k * b->z;
 }


template <typename T>
static __device__ void device_cross(T* cx, T* cy, T* cz, T ax, T ay, T az, T bx, T by, T bz) {
	*cx = ay * bz - az * by;
	*cy = az * bx - ax * bz;
	*cz = ax * by - ay * bx;
}
#if PPP_TAG == 0
static __device__ float device_inverse_sqrt(float x, float y, float z) {
	union { float x; s32 i; } u;
	u.x = (x * x) + (y * y) + (z * z); //(float)vector_normsq(v); //this cant be negative
	//if (u.x < DEVICE_EPSILON_SINGLE) { return 0.0; } //not sure if i need to do anything, if errors change this
	float invrs_sqrt_half = 0.5f * u.x;
	u.x = invrs_sqrt_half;

	u.i = 0x5f375a86 - (u.i >> 1); // 32 bit
	//u.i = 0x5FE6EB50C7B537A9 - (u.i >> 1); // 64 bit

	/* The next line can be repeated any number of times to increase accuracy */
	u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	for (int i = 0; i < 20; i++) {
		u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	}
	return u.x;
}
#elif PPP_TAG == 1
static __device__ double device_inverse_sqrt(double x, double y, double z) {
	union { double x; s64 i; } u;
	u.x = (x * x) + (y * y) + (z * z); //(float)vector_normsq(v); //this cant be negative
	if (u.x < DEVICE_EPSILON_SINGLE) { return 0.0; } //not sure if i need to do anything, if errors change this
	double invrs_sqrt_half = 0.5f * u.x;
	u.x = invrs_sqrt_half;

	//u.i = 0x5f375a86 - (u.i >> 1); // 32 bit
	u.i = 0x5FE6EB50C7B537A9 - (u.i >> 1); // 64 bit

	// The next line can be repeated any number of times to increase accuracy 
	u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	for(int i = 0; i < 10; i++){
		u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	}
	return u.x;
}
#endif

template <typename T>
static __device__ void device_normalize_vector(T* x, T* y, T* z) {
	T i_sqrt = device_inverse_sqrt(*x, *y, *z);
	*x *= i_sqrt; *y *= i_sqrt; *z *= i_sqrt;
}
template <typename T>
static __device__ T device_dot(T ax, T ay, T az, T bx, T by, T bz) {
	return (ax * bx) + (ay * by) + (az * bz);
}
#endif