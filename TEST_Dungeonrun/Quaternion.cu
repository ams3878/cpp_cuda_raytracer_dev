#include "framework.h"
#include "framework.cuh"



cudaError_t Quaternion::set_rot_matrix_CUDA() {
	return cudaPeekAtLastError();
}
cudaError_t Quaternion::set_rot_matrix_CUDA(VEC3_CUDA<VEC3_CUDA<T_fp>>* _rot_m) {
	//cudaMemcpy(rot_m, _rot_m, sizeof(T_fp) * 9 * size, cudaMemcpyHostToDevice);
	return cudaPeekAtLastError();
}
__global__ void init_rotation_matrix(VEC3_CUDA<VEC3_CUDA<T_fp>>* r, T_fp* rx, T_fp* ry, T_fp* rz, int sw, T_uint max_threads) {
	u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (voxel_index > max_threads) { return; }
	switch (sw) {
	case 0:
		r->x->i = rx;
		r->x->j = ry;
		r->x->k = rz;
		break;
	case 1:
		r->y->i = rx;
		r->y->j = ry;
		r->y->k = rz;
		break;
	case 2:
		r->z->i = rx;
		r->z->j = ry;
		r->z->k = rz;
		break;
	case 3:
		r->x->i[voxel_index] = (T_fp)1.0;		r->x->j[voxel_index] = (T_fp)0.0;		r->x->k[voxel_index] = (T_fp)0.0;
		r->y->i[voxel_index] = (T_fp)0.0;		r->y->j[voxel_index] = (T_fp)1.0;		r->y->k[voxel_index] = (T_fp)0.0;
		r->z->i[voxel_index] = (T_fp)0.0;		r->z->j[voxel_index] = (T_fp)0.0;		r->z->k[voxel_index] = (T_fp)1.0;
	}
}

cudaError_t Quaternion::initialize_CUDA() {
	vec = new VEC4_CUDA<T_fp>();
	cudaMalloc((void**)&vec->i, sizeof(T_fp) * size);
	cudaMalloc((void**)&vec->j, sizeof(T_fp) * size);
	cudaMalloc((void**)&vec->k, sizeof(T_fp) * size);
	cudaMalloc((void**)&vec->w, sizeof(T_fp) * size);
	cudaMalloc((void**)&d_vec, sizeof(VEC4_CUDA<T_fp>));
	cudaMemcpy(d_vec, vec, sizeof(VEC4_CUDA<T_fp>), cudaMemcpyHostToDevice);

	rot_m = new VEC3_CUDA<VEC3_CUDA<T_fp>>();

	cudaMalloc((void**)&d_rot_m, sizeof(VEC3_CUDA<VEC3_CUDA<T_fp>>));

	cudaMalloc((void**)&rot_m->x, sizeof(VEC3_CUDA<T_fp>));
	cudaMalloc((void**)&rot_m->y, sizeof(VEC3_CUDA<T_fp>));
	cudaMalloc((void**)&rot_m->z, sizeof(VEC3_CUDA<T_fp>));
	cudaMemcpy(d_rot_m, rot_m, sizeof(VEC3_CUDA<VEC3_CUDA<T_fp>>), cudaMemcpyHostToDevice);

	rot_m->x = new VEC3_CUDA<T_fp>();
	rot_m->y = new VEC3_CUDA<T_fp>();
	rot_m->z = new VEC3_CUDA<T_fp>();
	cudaMalloc((void**)&rot_m->x->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->x->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->x->k, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->x->i, rot_m->x->j, rot_m->x->k, 0, 1);
	delete(rot_m->x);
	cudaMalloc((void**)&rot_m->y->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->y->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->y->k, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->y->i, rot_m->y->j, rot_m->y->k, 1, 1);
	delete(rot_m->y);

	cudaMalloc((void**)&rot_m->z->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->z->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->z->k, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->z->i, rot_m->z->j, rot_m->z->k, 2, 1);
	delete(rot_m->z);
	delete(rot_m);
	init_rotation_matrix << < 1 + (size / 512), 512 >> > (d_rot_m, 0, 0, 0, 3, size);


	return cudaPeekAtLastError();
}

cudaError_t Quaternion::_memset_CUDA(VEC4_CUDA<T_fp>* v) {
	cudaMemcpy(vec->complex.i, v->complex.i, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(vec->complex.j, v->complex.j, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(vec->complex.k, v->complex.k, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	cudaMemcpy(vec->w, v->w, sizeof(T_fp) * size, cudaMemcpyHostToDevice);
	return cudaPeekAtLastError();
}


