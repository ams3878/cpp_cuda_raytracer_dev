#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "platform_common.h"
#include <stdio.h>
#define DEVICE_EPSILON_SINGLE .0001

#ifndef CUDA_VECTOR_H
#include "vector.cuh"
#endif
#include "Camera.h"
#include "Trixel.h"
#define BLOCK_SIZE 512

__global__ void init_tri_mem_cuda(Trixel::trixel_memory* t, double* point_data) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	u64 p1 = i * 9;
	t->d_p1[i].x = point_data[p1]; 	t->d_p1[i].y = point_data[p1 + 1]; 	t->d_p1[i].z = point_data[p1 + 2];

	t->d_edges.e1.x[i] = point_data[p1 + 3] - point_data[p1];
	t->d_edges.e1.y[i] = point_data[p1 + 4] - point_data[p1 + 1];
	t->d_edges.e1.z[i] = point_data[p1 + 5] - point_data[p1 + 2];

	t->d_edges.e2.x[i] = point_data[p1 + 6] - point_data[p1];
	t->d_edges.e2.y[i] = point_data[p1 + 7] - point_data[p1 + 1];
	t->d_edges.e2.z[i] = point_data[p1 + 8] - point_data[p1 + 2];

	device_cross(&t->d_n.x[i], &t->d_n.y[i], &t->d_n.z[i], t->d_edges.e1.x[i], t->d_edges.e1.y[i], t->d_edges.e1.z[i], t->d_edges.e2.x[i], t->d_edges.e2.y[i], t->d_edges.e2.z[i]);
	device_normalize_vector(&t->d_n.x[i], &t->d_n.y[i], &t->d_n.z[i]);


}
__global__ void init_cam_tri_mem_cuda(Trixel::trixel_memory* tm, Camera::trixel_memory* cm, Camera::orientation_properties::position c_pos) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	cm->d_t.x[i] = c_pos.x - tm->d_p1[i].x;	cm->d_t.y[i] = c_pos.y - tm->d_p1[i].y;	cm->d_t.z[i] = c_pos.z - tm->d_p1[i].z;
	device_cross(&cm->d_q.x[i], &cm->d_q.y[i],&cm->d_q.z[i], cm->d_t.x[i], cm->d_t.y[i], cm->d_t.z[i], tm->d_edges.e1.x[i], tm->d_edges.e1.y[i], tm->d_edges.e1.z[i]);
	cm->d_w[i] = device_dot(cm->d_q.x[i], cm->d_q.y[i], cm->d_q.z[i], tm->d_edges.e2.x[i], tm->d_edges.e2.y[i], tm->d_edges.e2.z[i]);
}

__global__ void intersect_tri_cuda(Trixel::trixel_memory* tm, Camera::pixel_memory* cm, Camera::trixel_memory* ctm, u64 tcount) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	u64 pattern = i % 8;

	u64 tri_index = -1;
	double d = 400.0;//**TODO** get rid of hardcode
	double pe1,f, p_x, p_y, p_z, u, v, w;
	//DO moller TUMBORE ON EACH TRIANGLE, set index to closest one.
	for (u64 tri_i = 0; tri_i < tcount; tri_i++) {
		device_cross(&p_x, &p_y, &p_z,
			cm->rmd.d_x[i], cm->rmd.d_y[i], cm->rmd.d_z[i],
			tm->d_edges.e2.x[tri_i], tm->d_edges.e2.y[tri_i], tm->d_edges.e2.z[tri_i]);
		f = device_dot(p_x, p_y, p_z,
			tm->d_edges.e1.x[tri_i], tm->d_edges.e1.y[tri_i], tm->d_edges.e1.z[tri_i]);
		if (!(f < DEVICE_EPSILON_SINGLE && f > -DEVICE_EPSILON_SINGLE)) {
			pe1 = 1.0 / f;
			u = pe1 * device_dot(p_x, p_y, p_z,
				ctm->d_t.x[tri_i], ctm->d_t.y[tri_i], ctm->d_t.z[tri_i]);
			v = pe1 * device_dot(cm->rmd.d_x[i], cm->rmd.d_y[i], cm->rmd.d_z[i],
				ctm->d_q.x[tri_i], ctm->d_q.y[tri_i], ctm->d_q.z[tri_i]);
			w = pe1 * ctm->d_w[tri_i];
			if ((w < d) && !((u < DEVICE_EPSILON_SINGLE) || (v < DEVICE_EPSILON_SINGLE) || ((u + v) > 1) || (w < DEVICE_EPSILON_SINGLE))) {
				d = w; tri_index = tri_i;
			}
		}
	}
	cm->d_rmi.index[i] = tri_index;
	cm->d_dist.d[i] = d;
	if (tri_index != (s32)-1) {
		cm->pnt.d_x[i] = d * cm->rmd.d_x[i];
		cm->pnt.d_y[i] = d * cm->rmd.d_y[i];
		cm->pnt.d_z[i] = d * cm->rmd.d_z[i];
		cm->norm.d_x[i] = tm->d_n.x[tri_index];
		cm->norm.d_y[i] = tm->d_n.y[tri_index];
		cm->norm.d_z[i] = tm->d_n.z[tri_index];
	}
}
cudaError_t intersect_trixels_device(Trixel* t, Camera* c) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
	}
	u64 bl_size = (c->f_prop.res.count < BLOCK_SIZE) ? c->f_prop.res.count : BLOCK_SIZE;
	intersect_tri_cuda << < c->f_prop.res.count / bl_size, bl_size >> > (
		(Trixel::trixel_memory*)t->d_mem,
		(Camera::pixel_memory*)c->d_mem,
		(Camera::trixel_memory*)c->d_trixels,
		t->num_trixels);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		printf("intersect_trixels_device launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		printf("cudaDeviceSynchronize returned error code %d after launching intersect_trixels_device!\n", cudaStatus);
	}
	cudaMemcpy(c->h_mem.h_color.c, c->h_mem.d_color.c, c->f_prop.res.count * sizeof(u32), cudaMemcpyDeviceToHost);


	return cudaStatus;

}

cudaError_t init_camera_trixel_device_memory(Trixel* t, Camera* c) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
	}
	u64 bl_size = (t->num_trixels < BLOCK_SIZE) ? t->num_trixels : BLOCK_SIZE;
	init_cam_tri_mem_cuda << < t->num_trixels / bl_size, bl_size >> > ((Trixel::trixel_memory*)t->d_mem, (Camera::trixel_memory*)c->d_trixels,  c->o_prop.pos);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		printf("init_camera_trixel_device_memory launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		printf("cudaDeviceSynchronize returned error code %d after launching init_camera_trixel_device_memory!\n", cudaStatus);
	}
	return cudaStatus;
}

cudaError_t init_trixels_device_memory(Trixel* t) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
	}
	u64 bl_size = (t->num_trixels < BLOCK_SIZE) ? t->num_trixels : BLOCK_SIZE;
	double** temp = &(t->d_points_init_data);
	init_tri_mem_cuda << < t->num_trixels / bl_size + 1, bl_size >> > ((Trixel::trixel_memory *) t->d_mem, t->d_points_init_data);
	// Check for any errors launching the kernel
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		printf("init_trixels_device_memory launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		printf("cudaDeviceSynchronize returned error code %d after launching init_tri_mem_cuda!\n", cudaStatus);
	}
	cudaMemcpy(t->h_mem.h_p1, t->h_mem.d_p1, t->num_trixels * sizeof(double), cudaMemcpyDeviceToHost);

	return cudaStatus;
}