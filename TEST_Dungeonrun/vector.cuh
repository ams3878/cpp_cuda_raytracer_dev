#pragma once
#ifndef CUDA_VECTOR_H
#define CUDA_VECTOR_H
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "platform_common.h"
#include <stdio.h>
#include <typeinfo>

#define MOLLER_TRUMBORE_DEVICE_EPSILON 1e-80
#define DEVICE_EPSILON_SINGLE 1e-35
//This can be changed for release mode to bigger, but voxel traversal too many threads in debug mode
#define BLOCK_SIZE 512 

#define status_lauch_and_sync(a) cudaStatus = cudaGetLastError();\
if (cudaStatus != cudaSuccess) {printf("%s launch failed: %s\n", #a, cudaGetErrorString(cudaStatus));}\
cudaStatus = cudaDeviceSynchronize();\
if (cudaStatus != cudaSuccess) {printf("cudaDeviceSynchronize returned error code %d after launching %s!\n",cudaStatus, #a);};
template <typename T>
 struct __device__ d_VEC3 {
	union { T x; T r; T i; }; union { T y; T g; T j; }; union { T z; T b; T k; };
	__device__ d_VEC3() : x(), y(), z() {};
	__device__ d_VEC3(T _x, T _y, T _z) { x = _x; y = _y; z = _z; };
	template <typename h_T>
	__device__ void rotate(h_T qx, h_T qy, h_T qz) {
		T temp_x = x, temp_y = y, temp_z = z;
		x = temp_x * qx.i + temp_y * qx.j + temp_z * qx.k;
		y = temp_x * qy.i + temp_y * qy.j + temp_z * qy.k;
		z = temp_x * qz.i + temp_y * qz.j + temp_z * qz.k;
	};
};

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