#pragma once
#ifndef CUDA_VECTOR_H
#define CUDA_VECTOR_H
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "platform_common.h"
#include <stdio.h>
#define MOLLER_TRUMBORE_DEVICE_EPSILON 1e-15
#define DEVICE_EPSILON_SINGLE 1e-15
//This can be changed for release mode to bigger, but voxel traversal too many threads in debug mode
#define BLOCK_SIZE 512 

#define status_lauch_and_sync(a) cudaStatus = cudaGetLastError();\
if (cudaStatus != cudaSuccess) {printf("%s launch failed: %s\n", #a, cudaGetErrorString(cudaStatus));}\
cudaStatus = cudaDeviceSynchronize();\
if (cudaStatus != cudaSuccess) {printf("cudaDeviceSynchronize returned error code %d after launching %s!\n",cudaStatus, #a);};


static __device__ void device_cross(double* cx, double* cy, double* cz, double ax, double ay, double az, double bx, double by, double bz) {
	*cx = ay * bz - az * by;
	*cy = az * bx - ax * bz;
	*cz = ax * by - ay * bx;
}
static __device__ double device_inverse_sqrt(double x, double y, double z) {
	union { double x; s64 i; } u;
	u.x = (x * x) + (y * y) + (z * z); //(float)vector_normsq(v); //this cant be negative
	if (u.x < DEVICE_EPSILON_SINGLE) { return 0.0; } //not sure if i need to do anything, if errors change this
	double invrs_sqrt_half = 0.5f * u.x;
	u.x = invrs_sqrt_half;
	//u.i = 0x5f375a86 - (u.i >> 1); // 32 bit
	u.i = 0x5FE6EB50C7B537A9 - (u.i >> 1); // 64 bit

	/* The next line can be repeated any number of times to increase accuracy */
	u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	for(int i = 0; i < 10; i++){
		u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
	}
	return u.x;
}
static __device__ void device_normalize_vector(double* x, double* y, double* z) {
	double i_sqrt = device_inverse_sqrt(*x, *y, *z);
	*x *= i_sqrt; *y *= i_sqrt; *z *= i_sqrt;
}

static __device__ double device_dot(double ax, double ay, double az, double bx, double by, double bz) {
	return (ax * bx) + (ay * by) + (az * bz);
}
#endif