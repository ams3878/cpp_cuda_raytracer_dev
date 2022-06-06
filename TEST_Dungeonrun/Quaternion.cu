#include "framework.h"
#include "framework.cuh"

cudaError_t Quaternion::set_rot_matrix_CUDA() {
	cudaMemcpy(d_rot_m, h_rot_m, sizeof(T_fp) * 9 * size, cudaMemcpyHostToDevice);
	return cudaPeekAtLastError();
}

cudaError_t Quaternion::initialize_CUDA() {
	cudaMalloc((void**)&d_vec.i, sizeof(T_fp) * size);
	cudaMalloc((void**)&d_vec.j, sizeof(T_fp) * size);
	cudaMalloc((void**)&d_vec.k, sizeof(T_fp) * size);
	cudaMalloc((void**)&d_vec.w, sizeof(T_fp) * size);
	cudaMalloc((void**)&d_rot_m, sizeof(T_fp) * 9 * size);
	return cudaPeekAtLastError();
}

cudaError_t Quaternion::_memset_CUDA() {
	cudaMemcpy(d_vec.i, h_vec.i, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_vec.i, h_vec.i, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_vec.i, h_vec.i, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_vec.i, h_vec.i, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	return cudaPeekAtLastError();
}


