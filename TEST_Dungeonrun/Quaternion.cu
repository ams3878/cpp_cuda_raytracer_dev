#include "framework.h"
#include "framework.cuh"

__global__ void set_rotation_matrix(VEC4<VEC4<T_fp>*>* r, VEC4<T_fp> rx, VEC4<T_fp> ry, VEC4<T_fp> rz,  T_uint max_threads) {
	u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (voxel_index > max_threads) { return; }
	*r->x = rx;
	*r->y = ry;
	*r->z = rz;
}


__global__ void init_rotation_matrix(VEC4<VEC4<T_fp>*>* r, VEC4<T_fp>* rx, VEC4<T_fp>* ry, VEC4<T_fp>* rz,  T_uint max_threads) {
	u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (voxel_index > max_threads) { return; }
	r->x = rx;
	r->y = ry;
	r->z = rz;

}
cudaError_t Quaternion::set_device_rotation(VEC4<VEC4<T_fp>*>* host_rot_matrix){
	set_rotation_matrix << < 1, 1 >> > (d_rot_m, *host_rot_matrix->x, *host_rot_matrix->y, *host_rot_matrix->z, 1);
	return cudaPeekAtLastError();

}

cudaError_t Quaternion::initialize_CUDA(VEC4<T_fp>* init_x, VEC4<T_fp>* init_y, VEC4<T_fp>* init_z) {
	cudaError_t cudaStatus;
	cudaMalloc((void**)&d_vec, sizeof(VEC4<T_fp>));	
	cudaMemcpy(d_vec, vec, sizeof(VEC4<T_fp>), cudaMemcpyHostToDevice);
	
	VEC4<VEC4<T_fp>*>* t_rot_m = new VEC4<VEC4<T_fp>*>();

	cudaMalloc((void**)&d_rot_m, sizeof(VEC4<VEC4<T_fp>*>));

	cudaMalloc((void**)&t_rot_m->x, sizeof(VEC4<T_fp>));
	cudaMalloc((void**)&t_rot_m->y, sizeof(VEC4<T_fp>));
	cudaMalloc((void**)&t_rot_m->z, sizeof(VEC4<T_fp>));
	cudaMalloc((void**)&t_rot_m->w, sizeof(VEC4<T_fp>));

	cudaMemcpy(d_rot_m, t_rot_m, sizeof(VEC4<VEC4<T_fp>*>), cudaMemcpyHostToDevice);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		printf("init_QUATERNION launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	set_rotation_matrix << < 1, 1 >> > (d_rot_m, *init_x, *init_y, *init_z, 1);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		printf("set_QUATERNION launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
	delete(t_rot_m);

	return cudaPeekAtLastError();
}




