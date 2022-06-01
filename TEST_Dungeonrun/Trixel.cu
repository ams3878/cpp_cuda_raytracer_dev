#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "platform_common.h"
#include <math.h>
#include <stdio.h>
#include <cuda_profiler_api.h>

#ifndef CUDA_VECTOR_H
#include "vector.cuh"
#endif
#include "Camera.h"
#include "Trixel.h"

__global__ void init_tri_mem_cuda(Trixel::trixel_memory* t, double* point_data, u64 max_threads) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }
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
__global__ void init_cam_tri_mem_cuda(Trixel::trixel_memory* tm, Camera::trixel_memory* cm, vector_xyz c_pos, u64 max_threads) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }

	cm->d_t.x[i] = c_pos.x - tm->d_p1[i].x;	cm->d_t.y[i] = c_pos.y - tm->d_p1[i].y;	cm->d_t.z[i] = c_pos.z - tm->d_p1[i].z;
	device_cross(&cm->d_q.x[i], &cm->d_q.y[i],&cm->d_q.z[i], cm->d_t.x[i], cm->d_t.y[i], cm->d_t.z[i], tm->d_edges.e1.x[i], tm->d_edges.e1.y[i], tm->d_edges.e1.z[i]);
	cm->d_w[i] = device_dot(cm->d_q.x[i], cm->d_q.y[i], cm->d_q.z[i], tm->d_edges.e2.x[i], tm->d_edges.e2.y[i], tm->d_edges.e2.z[i]);
}

__global__ void intersect_voxel_cuda( Camera::pixel_memory* cm,  Camera::voxel_memory* cvm, Camera::trixel_memory* ctm, Trixel::trixel_memory* tm, s64 max_threads) {
	s64 i = (s64)threadIdx.x + ((s64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }

	double d = 400.0;//**TODO** get rid of hardcode
	double t0x, t1x, t0y, t1y, t0z, t1z, maxt0, mint1, dir, s1, s2;
	s64 cni, index_front = i * cvm->index_queue_offset, index_start = i * cvm->index_queue_offset;
	s64 tri_i;
	double pe1, f, p_x, p_y, p_z, u, v, w;
	cvm->d_voxel_index_queue[index_front] = 0;
	//cm->d_color.rad[i].r = 0;
	//cm->d_color.rad[i].g = 0;
	//cm->d_color.rad[i].b = 0;
	cm->d_rmi.index[i] = (s64)-1;
	cm->d_dist.d[i] = d;

	while ((index_front - index_start) >= 0) {
		cni = cvm->d_voxel_index_queue[index_front--];
		if (cvm->is_leaf[cni]) {
			tri_i = cvm->children[cni].triangle;
			device_cross(&p_x, &p_y, &p_z,
				cm->rmd.d_x[i], cm->rmd.d_y[i], cm->rmd.d_z[i],
				tm->d_edges.e2.x[tri_i], tm->d_edges.e2.y[tri_i], tm->d_edges.e2.z[tri_i]);
			f = device_dot(p_x, p_y, p_z,
				tm->d_edges.e1.x[tri_i], tm->d_edges.e1.y[tri_i], tm->d_edges.e1.z[tri_i]);
			if (!(f < MOLLER_TRUMBORE_DEVICE_EPSILON && f > -MOLLER_TRUMBORE_DEVICE_EPSILON)) {
				pe1 = 1.0 / f;
				u = pe1 * device_dot(p_x, p_y, p_z,
					ctm->d_t.x[tri_i], ctm->d_t.y[tri_i], ctm->d_t.z[tri_i]);
				v = pe1 * device_dot(cm->rmd.d_x[i], cm->rmd.d_y[i], cm->rmd.d_z[i],
					ctm->d_q.x[tri_i], ctm->d_q.y[tri_i], ctm->d_q.z[tri_i]);
				w = pe1 * ctm->d_w[tri_i];
				if ((w < d) && !((u < MOLLER_TRUMBORE_DEVICE_EPSILON) || (v < MOLLER_TRUMBORE_DEVICE_EPSILON) || ((u + v) > 1 + MOLLER_TRUMBORE_DEVICE_EPSILON) || (w < MOLLER_TRUMBORE_DEVICE_EPSILON))) {
					//cm->d_rmi.index[i] = -2;
					//return;
					cm->d_rmi.index[i] = tri_i;
					cm->d_dist.d[i] = w;
					cm->d_color.rad[i].r = tm->d_color.rad[tri_i].r;
					cm->d_color.rad[i].g = tm->d_color.rad[tri_i].g;
					cm->d_color.rad[i].b = tm->d_color.rad[tri_i].b;
					cm->pnt.d_x[i] = d * cm->rmd.d_x[i];
					cm->pnt.d_y[i] = d * cm->rmd.d_y[i];
					cm->pnt.d_z[i] = d * cm->rmd.d_z[i];
					cm->norm.d_x[i] = tm->d_n.x[tri_i];
					cm->norm.d_y[i] = tm->d_n.y[tri_i];
					cm->norm.d_z[i] = tm->d_n.z[tri_i];
					return;
				}
			}	
			continue;
		}
		//YAY NO BRANCHES  ???? is it worth??? NO IDEA
			//swap t0, t1 if ray in negative direction
			//**TODO** precomput sign * rmd to remove more operations
		t0x = ((cvm->d_Bo[cni].t0x * (1 - cm->sign_rmd.d_x[i])) + (cvm->d_Bo[cni].t1x * (cm->sign_rmd.d_x[i]))) * cm->inv_rmd.d_x[i];
		t1x = ((cvm->d_Bo[cni].t1x * (1 - cm->sign_rmd.d_x[i])) + (cvm->d_Bo[cni].t0x * (cm->sign_rmd.d_x[i]))) * cm->inv_rmd.d_x[i];

		t0y = ((cvm->d_Bo[cni].t0y * (1 - cm->sign_rmd.d_y[i])) + (cvm->d_Bo[cni].t1y * (cm->sign_rmd.d_y[i]))) * cm->inv_rmd.d_y[i];
		t1y = ((cvm->d_Bo[cni].t1y * (1 - cm->sign_rmd.d_y[i])) + (cvm->d_Bo[cni].t0y * (cm->sign_rmd.d_y[i]))) * cm->inv_rmd.d_y[i];

		t0z = ((cvm->d_Bo[cni].t0z * (1 - cm->sign_rmd.d_z[i])) + (cvm->d_Bo[cni].t1z * (cm->sign_rmd.d_z[i]))) * cm->inv_rmd.d_z[i];
		t1z = ((cvm->d_Bo[cni].t1z * (1 - cm->sign_rmd.d_z[i])) + (cvm->d_Bo[cni].t0z * (cm->sign_rmd.d_z[i]))) * cm->inv_rmd.d_z[i];
		//select entrance (maxt0) and exit(mint1) planes of voxel, and then get coordinate in split direction ( t * dir)
		//if ((t0x > t1y) || (t0y > t1x)) { return; }
		maxt0 = fmax(t0z, fmax(t0x, t0y));
		mint1 = fmin(t1z, fmin(t1x, t1y));
		//if ((maxt0 > t1z) || (t0z > mint1)) { return; }

		if (mint1 > maxt0 - DEVICE_EPSILON_SINGLE && maxt0 > -DEVICE_EPSILON_SINGLE) {
			//cm->d_rmi.index[i] = (s64)-2;
			dir = ((cm->rmd.d_x[i] * cvm->cut_flags[cni].x) + (cm->rmd.d_y[i] * cvm->cut_flags[cni].y) + (cm->rmd.d_z[i] * cvm->cut_flags[cni].z));
			maxt0 *= dir; mint1 *= dir;

			s1 = cvm->s1[cni] + DEVICE_EPSILON_SINGLE;
			s2 = cvm->s2[cni] ;
			if (maxt0  < s2 + DEVICE_EPSILON_SINGLE) {
				//cm->d_color.rad[i].r += .001;

				if (mint1 > s2 - DEVICE_EPSILON_SINGLE) { 
					cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].right;
					//cm->d_color.rad[i].r += .0005;

				}
				cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].left;
			}
			else {
				//cm->d_color.rad[i].b += .001;
				if (mint1  < s1 || maxt0 < s1 ) {
					cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].left;
					//cm->d_color.rad[i].r += .0005;

				}
				cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].right;
			}
		}
	}
	return;
}
__global__ void intersect_tri_cuda(Trixel::trixel_memory* tm, Camera::pixel_memory* cm, Camera::trixel_memory* ctm,
	s64 num_trixels, u64 max_threads) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }
	s64 tri_index = -1;
	double d = 400.0;//**TODO** get rid of hardcode
	double pe1,f, p_x, p_y, p_z, u, v, w;
	//DO moller TUMBORE ON EACH TRIANGLE, set index to closest one.
	for (int tri_i = 0; tri_i < num_trixels; tri_i++) {
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
	if (tri_index != (s64)-1) {
		cm->pnt.d_x[i] = d * cm->rmd.d_x[i];
		cm->pnt.d_y[i] = d * cm->rmd.d_y[i];
		cm->pnt.d_z[i] = d * cm->rmd.d_z[i];
		cm->norm.d_x[i] = tm->d_n.x[tri_index];
		cm->norm.d_y[i] = tm->d_n.y[tri_index];
		cm->norm.d_z[i] = tm->d_n.z[tri_index];
	}
}
cudaError_t intersect_trixels_device(Trixel* t, Camera* c, u32 mode) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?\n");
	}

	if (mode == 0) {
		intersect_voxel_cuda << < 1 + (u32)(c->f_prop.res.count / 128), 128 >> > (
			(Camera::pixel_memory*)c->d_mem,
			(Camera::voxel_memory*)c->d_voxels,
			(Camera::trixel_memory*)c->d_trixels,
			(Trixel::trixel_memory*)t->d_mem,
			c->f_prop.res.count);		
	}
	else {
		intersect_tri_cuda << < 1 + (u32)(c->f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >> > (
			(Trixel::trixel_memory*)t->d_mem,
			(Camera::pixel_memory*)c->d_mem,
			(Camera::trixel_memory*)c->d_trixels,
			t->num_trixels, c->f_prop.res.count);
	}
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


	return cudaStatus;

}

cudaError_t init_camera_trixel_device_memory(Trixel* t, Camera* c) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
	}
	init_cam_tri_mem_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > ((Trixel::trixel_memory*)t->d_mem, (Camera::trixel_memory*)c->d_trixels,  c->o_prop.pos, t->num_trixels);
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
	init_tri_mem_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > ((Trixel::trixel_memory*)t->d_mem, t->d_points_init_data, t->num_trixels);
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

