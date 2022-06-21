#include "framework.h"
#include "framework.cuh"


__global__ void init_rotation_matrix(VEC4_CUDA<VEC4_CUDA<T_fp>>* r, T_fp* rx, T_fp* ry, T_fp* rz, T_fp* rw, int sw, T_uint max_threads) {
	u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (voxel_index > max_threads) { return; }
	switch (sw) {
	case 0:
		r->x->i = rx;
		r->x->j = ry;
		r->x->k = rz;
		r->x->w = rw;
		break;
	case 1:
		r->y->i = rx;
		r->y->j = ry;
		r->y->k = rz;
		r->y->w = rw;
		break;
	case 2:
		r->z->i = rx;
		r->z->j = ry;
		r->z->k = rz;
		r->z->w = rw;
		break;
	case 3:
		r->x->i[voxel_index] = (T_fp)1.0; r->x->j[voxel_index] = (T_fp)0.0;	r->x->k[voxel_index] = (T_fp)0.0; r->x->w[voxel_index] = (T_fp)0.0;
		r->y->i[voxel_index] = (T_fp)0.0; r->y->j[voxel_index] = (T_fp)1.0;	r->y->k[voxel_index] = (T_fp)0.0; r->y->w[voxel_index] = (T_fp)0.0;
		r->z->i[voxel_index] = (T_fp)0.0; r->z->j[voxel_index] = (T_fp)0.0;	r->z->k[voxel_index] = (T_fp)1.0; r->z->w[voxel_index] = (T_fp)0.0;
		//r->w->i[voxel_index] = (T_fp)0.0; r->w->j[voxel_index] = (T_fp)0.0;	r->w->k[voxel_index] = (T_fp)0.0; r->w->w[voxel_index] = (T_fp)1.0;
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

	rot_m = new VEC4_CUDA<VEC4_CUDA<T_fp>>();

	cudaMalloc((void**)&d_rot_m, sizeof(VEC4_CUDA<VEC4_CUDA<T_fp>>));

	cudaMalloc((void**)&rot_m->x, sizeof(VEC4_CUDA<T_fp>));
	cudaMalloc((void**)&rot_m->y, sizeof(VEC4_CUDA<T_fp>));
	cudaMalloc((void**)&rot_m->z, sizeof(VEC4_CUDA<T_fp>));
	cudaMalloc((void**)&rot_m->w, sizeof(VEC4_CUDA<T_fp>));

	cudaMemcpy(d_rot_m, rot_m, sizeof(VEC4_CUDA<VEC4_CUDA<T_fp>>), cudaMemcpyHostToDevice);

	rot_m->x = new VEC4_CUDA<T_fp>();
	rot_m->y = new VEC4_CUDA<T_fp>();
	rot_m->z = new VEC4_CUDA<T_fp>();
	//rot_m->w = new VEC4_CUDA<T_fp>();

	cudaMalloc((void**)&rot_m->x->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->x->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->x->k, sizeof(T_fp) * size); cudaMalloc((void**)&rot_m->x->w, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->x->i, rot_m->x->j, rot_m->x->k, rot_m->x->w, 0, 1);
	delete(rot_m->x);
	cudaMalloc((void**)&rot_m->y->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->y->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->y->k, sizeof(T_fp) * size); cudaMalloc((void**)&rot_m->y->w, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->y->i, rot_m->y->j, rot_m->y->k, rot_m->y->w, 1, 1);
	delete(rot_m->y);
	cudaMalloc((void**)&rot_m->z->i, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->z->j, sizeof(T_fp) * size);	cudaMalloc((void**)&rot_m->z->k, sizeof(T_fp) * size); cudaMalloc((void**)&rot_m->z->w, sizeof(T_fp) * size);
	init_rotation_matrix << < 1, 1 >> > (d_rot_m, rot_m->z->i, rot_m->z->j, rot_m->z->k, rot_m->z->w, 2, 1);
	delete(rot_m->z);
	delete(rot_m);
	init_rotation_matrix << < 1 + (size / 512), 512 >> > (d_rot_m, 0, 0, 0, 0,3, size);

	return cudaPeekAtLastError();
}




