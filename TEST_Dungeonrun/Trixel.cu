#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <math.h>
#include <stdio.h>
#include <cuda_profiler_api.h>
#include <cuda_runtime.h>
#include "cuda_profiler_api.h"
#include "framework.h"
#include "framework.cuh"

__global__ void init_tri_mem_cuda(Trixel::trixel_memory* t, T_fp* point_data, u64 max_threads) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }
	u64 p1 = i * 9;
	t->d_p1.x[i] = point_data[p1]; 	t->d_p1.y[i] = point_data[p1 + 1]; 	t->d_p1.z[i] = point_data[p1 + 2];

	t->d_edges.e1.x[i] = point_data[p1 + 3] - point_data[p1];
	t->d_edges.e1.y[i] = point_data[p1 + 4] - point_data[p1 + 1];
	t->d_edges.e1.z[i] = point_data[p1 + 5] - point_data[p1 + 2];

	t->d_edges.e2.x[i] = point_data[p1 + 6] - point_data[p1];
	t->d_edges.e2.y[i] = point_data[p1 + 7] - point_data[p1 + 1];
	t->d_edges.e2.z[i] = point_data[p1 + 8] - point_data[p1 + 2];

	device_cross(&t->d_n.x[i], &t->d_n.y[i], &t->d_n.z[i], t->d_edges.e1.x[i], t->d_edges.e1.y[i], t->d_edges.e1.z[i], t->d_edges.e2.x[i], t->d_edges.e2.y[i], t->d_edges.e2.z[i]);
	device_normalize_vector(&t->d_n.x[i], &t->d_n.y[i], &t->d_n.z[i]);
}

__global__ void init_cam_tri_mem_cuda(Trixel::trixel_memory* tm, Camera::trixel_memory* cm, VEC3<T_fp> c_pos, u64 max_threads) {
	u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }

	cm->d_t.x[i] = c_pos.x - tm->d_p1.x[i];	cm->d_t.y[i] = c_pos.y - tm->d_p1.y[i];	cm->d_t.z[i] = c_pos.z - tm->d_p1.z[i];
	device_cross(&cm->d_q.x[i], &cm->d_q.y[i],&cm->d_q.z[i], cm->d_t.x[i], cm->d_t.y[i], cm->d_t.z[i], tm->d_edges.e1.x[i], tm->d_edges.e1.y[i], tm->d_edges.e1.z[i]);
	cm->d_w[i] = device_dot(cm->d_q.x[i], cm->d_q.y[i], cm->d_q.z[i], tm->d_edges.e2.x[i], tm->d_edges.e2.y[i], tm->d_edges.e2.z[i]);
}

__global__ void intersect_voxel_cuda( Camera::pixel_memory* cm,  Camera::voxel_memory* cvm, Camera::trixel_memory* ctm, VEC4<VEC4<T_fp>*>* rot_m, Trixel::trixel_memory* tm, s64 max_threads) {
	s64 i = (s64)threadIdx.x + ((s64)threadIdx.y * blockDim.x) + ((s64)blockIdx.x * blockDim.x * blockDim.y);
	//s64 i = (s64)threadIdx.x + ((s64)blockIdx.x * blockDim.x );

	if (i >= max_threads) { return; }

	T_fp d = 400.0;//**TODO** get rid of hardcode
	T_fp t0x, t1x, t0y, t1y, t0z, t1z, maxt0, mint1, dir, s1, s2, ds;
	T_fp tmpqx, tmpqy, tmpqz;
	s32 cni, index_front = i * cvm->index_queue_offset, index_start = i * cvm->index_queue_offset;
	s64 tri_i;
	T_fp pe1, f, p_x, p_y, p_z, u, v, w;
	cvm->d_voxel_index_queue[index_front] = 0;
	//cm->d_color.rad[i].r = 0;
	//cm->d_color.rad[i].g = 0;
	//cm->d_color.rad[i].b = 0;
	cm->d_rmi.index[i] = (s64)-1;
	//cm->d_dist.d[i] = d;

	T_fp obj_dx = rot_m->x->w;
	T_fp obj_dy = rot_m->y->w;
	T_fp obj_dz = rot_m->z->w;

	T_fp rmd_x = -1 * (rot_m->x->i * -cm->rmd.x[i] + rot_m->x->j * -cm->rmd.y[i] + rot_m->x->k * -cm->rmd.z[i]);// +obj_dx);
	T_fp rmd_y = -1 * (rot_m->y->i * -cm->rmd.x[i] + rot_m->y->j * -cm->rmd.y[i] + rot_m->y->k * -cm->rmd.z[i]);// + obj_dy);
	T_fp rmd_z = -1 * (rot_m->z->i * -cm->rmd.x[i] + rot_m->z->j * -cm->rmd.y[i] + rot_m->z->k * -cm->rmd.z[i]);// + obj_dz);



	while ((index_front - index_start) >= 0) {
		cni = cvm->d_voxel_index_queue[index_front--];
				//YAY NO BRANCHES  ???? is it worth??? NO IDEA
			//swap t0, t1 if ray in negative direction
			//**TODO** precomput sign * rmd to remove more operations
		
		t0x = rmd_x > 0 ? cvm->d_Bo[cni].t0x * (1 / rmd_x) : cvm->d_Bo[cni].t1x * (1 / rmd_x);
		t1x = rmd_x > 0 ? cvm->d_Bo[cni].t1x * (1 / rmd_x) : cvm->d_Bo[cni].t0x * (1 / rmd_x);

		t0y = rmd_y > 0 ? cvm->d_Bo[cni].t0y * (1 / rmd_y) : cvm->d_Bo[cni].t1y * (1 / rmd_y);
		t1y = rmd_y > 0 ? cvm->d_Bo[cni].t1y * (1 / rmd_y) : cvm->d_Bo[cni].t0y * (1 / rmd_y);

		t0z = rmd_z > 0 ? cvm->d_Bo[cni].t0z * (1 / rmd_z) : cvm->d_Bo[cni].t1z * (1 / rmd_z);
		t1z = rmd_z > 0 ? cvm->d_Bo[cni].t1z * (1 / rmd_z) : cvm->d_Bo[cni].t0z * (1 / rmd_z);

		//select entrance (maxt0) and exit(mint1) planes of voxel, and then get coordinate in split direction ( t * dir)

		//if ((t0x > t1y) || (t0y > t1x)) { return; }
		dir = ((rmd_x * cvm->cut_flags[cni].x) + (rmd_y * cvm->cut_flags[cni].y) + (rmd_z * cvm->cut_flags[cni].z));
		ds = 0;
		ds = ((obj_dx * cvm->cut_flags[cni].x) + (obj_dy * cvm->cut_flags[cni].y) + (obj_dz * cvm->cut_flags[cni].z));
		
		maxt0 = fmax(t0z , fmax(t0x , t0y ));
		mint1 = fmin(t1z , fmin(t1x , t1y ));
		maxt0 = fmax(t0z + obj_dz / rmd_z, fmax(t0x + obj_dx / rmd_x, t0y + obj_dy / rmd_y));	mint1 = fmin(t1z + obj_dz / rmd_z, fmin(t1x + obj_dx / rmd_x, t1y + obj_dy / rmd_y));


		if (cvm->is_leaf[cni]) {
			//d_t is ray_origin - vertex0
			tri_i = cvm->children[cni].triangle;
			device_cross(&p_x, &p_y, &p_z,
				rmd_x, rmd_y, rmd_z,
				tm->d_edges.e2.x[tri_i], tm->d_edges.e2.y[tri_i], tm->d_edges.e2.z[tri_i]);
			f = device_dot(p_x, p_y, p_z,
				tm->d_edges.e1.x[tri_i], tm->d_edges.e1.y[tri_i], tm->d_edges.e1.z[tri_i]);
			if (!(f < MOLLER_TRUMBORE_DEVICE_EPSILON && f > -MOLLER_TRUMBORE_DEVICE_EPSILON)) {
				pe1 = 1.0 / f;
				u = pe1 * device_dot(p_x, p_y, p_z,
					ctm->d_t.x[tri_i] - obj_dx, ctm->d_t.y[tri_i] - obj_dy, ctm->d_t.z[tri_i] - obj_dz);
				//ctm->d_t.x[tri_i] , ctm->d_t.y[tri_i] , ctm->d_t.z[tri_i] );

				device_cross(&tmpqx, &tmpqy, &tmpqz,
					ctm->d_t.x[tri_i] - obj_dx, ctm->d_t.y[tri_i] - obj_dy, ctm->d_t.z[tri_i] - obj_dz,
				tm->d_edges.e1.x[tri_i], tm->d_edges.e1.y[tri_i], tm->d_edges.e1.z[tri_i]);
				if (i == 319278) {
				//	printf("new(%d) : %f %f %f \n", i, tmpqx, tmpqy, tmpqz);
				//	printf("old(%d) : %f %f %f \n", i, ctm->d_q.x[tri_i], ctm->d_q.y[tri_i], ctm->d_q.z[tri_i]);
				}
				v = pe1 * device_dot(rmd_x, rmd_y, rmd_z,
					//ctm->d_q.x[tri_i], ctm->d_q.y[tri_i], ctm->d_q.z[tri_i]);
					tmpqx, tmpqy, tmpqz);

				w = pe1 * ctm->d_w[tri_i];
				w = pe1 * device_dot(tm->d_edges.e2.x[tri_i], tm->d_edges.e2.y[tri_i], tm->d_edges.e2.z[tri_i],
					tmpqx, tmpqy, tmpqz);

				if ((w < d) && !((u < MOLLER_TRUMBORE_DEVICE_EPSILON) || (v < MOLLER_TRUMBORE_DEVICE_EPSILON) || ((u + v) > 1 + MOLLER_TRUMBORE_DEVICE_EPSILON) || (w < MOLLER_TRUMBORE_DEVICE_EPSILON))) {
					d = w;
					cm->d_rmi.index[i] = tri_i;
					cm->d_dist.d[i] = w;
					cm->d_color.rad[i].r = tm->d_color.rad[tri_i].r;
					cm->d_color.rad[i].g = tm->d_color.rad[tri_i].g;
					cm->d_color.rad[i].b = tm->d_color.rad[tri_i].b;
					cm->pnt.x[i] = d * rmd_x + obj_dx;
					cm->pnt.y[i] = d * rmd_y + obj_dy;
					cm->pnt.z[i] = d * rmd_z + obj_dz;
					cm->norm.x[i] = tm->d_n.x[tri_i];
					cm->norm.y[i] = tm->d_n.y[tri_i];
					cm->norm.z[i] = tm->d_n.z[tri_i];
					cm->norm.device_rotate(rot_m, i, -1);
					//return;
				}
			}
			continue;
		}
		if (mint1 >= maxt0 - DEVICE_EPSILON_SINGLE && maxt0 > -DEVICE_EPSILON_SINGLE) {

			//cm->d_rmi.index[i] = (s64)-2;
			maxt0 *= dir; mint1 *= dir;		
			s1 = cvm->s1[cni] + DEVICE_EPSILON_SINGLE + ds;
			s2 = cvm->s2[cni] + ds;
			//cm->d_color.rad[i].r += .001;

		
			if (maxt0  < s2 + DEVICE_EPSILON_SINGLE) {
				if (mint1 > s2 - DEVICE_EPSILON_SINGLE) { 
					cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].right;
					//cm->d_color.rad[i].r += .0005;
				}
				cvm->d_voxel_index_queue[++index_front] = cvm->children[cni].left;
			}
			else {
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
__global__ void intersect_trixel_cuda(Camera::pixel_memory* cm, u32 num_tris, Camera::trixel_memory* ctm, Trixel::trixel_memory* tm, s64 max_threads) {
	s64 i = (s64)threadIdx.x + ((s64)blockIdx.x * blockDim.x);
	if (i >= max_threads) { return; }
	T_fp d = 400.0;//**TODO** get rid of hardcode
	T_fp pe1, f, p_x, p_y, p_z, u, v, w;
	cm->d_rmi.index[i] = -1;
	for (int tri_i = 0; tri_i < num_tris; tri_i++) {
		device_cross(&p_x, &p_y, &p_z,
			cm->rmd.x[i], cm->rmd.y[i], cm->rmd.z[i],
			tm->d_edges.e2.x[tri_i], tm->d_edges.e2.y[tri_i], tm->d_edges.e2.z[tri_i]);
		f = device_dot(p_x, p_y, p_z,
			tm->d_edges.e1.x[tri_i], tm->d_edges.e1.y[tri_i], tm->d_edges.e1.z[tri_i]);
		if (!(f < MOLLER_TRUMBORE_DEVICE_EPSILON && f > -MOLLER_TRUMBORE_DEVICE_EPSILON)) {
			pe1 = 1.0 / f;
			u = pe1 * device_dot(p_x, p_y, p_z,
				ctm->d_t.x[tri_i], ctm->d_t.y[tri_i], ctm->d_t.z[tri_i]);
			v = pe1 * device_dot(cm->rmd.x[i], cm->rmd.y[i], cm->rmd.z[i],
				ctm->d_q.x[tri_i], ctm->d_q.y[tri_i], ctm->d_q.z[tri_i]);
			w = pe1 * ctm->d_w[tri_i];
			if ((w < d) && !((u < MOLLER_TRUMBORE_DEVICE_EPSILON) || (v < MOLLER_TRUMBORE_DEVICE_EPSILON) || ((u + v) > 1 + MOLLER_TRUMBORE_DEVICE_EPSILON) || (w < MOLLER_TRUMBORE_DEVICE_EPSILON))) {
				//cm->d_rmi.index[i] = -2;
				d = w;
				cm->d_rmi.index[i] = tri_i;
				cm->d_dist.d[i] = w;
				cm->d_color.rad[i].r = tm->d_color.rad[tri_i].r;
				cm->d_color.rad[i].g = tm->d_color.rad[tri_i].g;
				cm->d_color.rad[i].b = tm->d_color.rad[tri_i].b;
				cm->pnt.x[i] = d * cm->rmd.x[i];
				cm->pnt.y[i] = d * cm->rmd.y[i];
				cm->pnt.z[i] = d * cm->rmd.z[i];
				cm->norm.x[i] = tm->d_n.x[tri_i];
				cm->norm.y[i] = tm->d_n.y[tri_i];
				cm->norm.z[i] = tm->d_n.z[tri_i];
			}
		}
	}
}
cudaError_t intersect_trixels_device(Trixel* t, Camera* c, u32 mode) {
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?\n");
	}

	intersect_voxel_cuda << < 1 + (u32)(c->f_prop.res.count / (512)), (512) >> > (
		c->d_mem,
		c->d_voxels,
		c->d_trixels,
		c->object_list[0]->quat->d_rot_m,
		(Trixel::trixel_memory*)t->d_mem,
		c->f_prop.res.count);		
	
	
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
	init_cam_tri_mem_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > ((Trixel::trixel_memory*)t->d_mem, c->d_trixels,  c->o_prop.pos, t->num_trixels);
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
	//cudaMemcpy(t->h_mem.h_p1, t->h_mem.d_p1, t->num_trixels * sizeof(T_fp), cudaMemcpyDeviceToHost);

	return cudaStatus;
}

