#pragma once
#include "platform_common.h"
#include "Vector.h"
#ifndef CUDA_KERNEL
#define CUDA_KERNEL
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#endif
#include "Camera.h"
struct Trixel;
cudaError_t intersect_trixels_device(Trixel* t, Camera* c);
cudaError_t init_trixels_device_memory(Trixel* t);
cudaError_t init_camera_trixel_device_memory(Trixel* t, Camera* C);


class Trixel
{
public:
	u64 num_trixels;
	double* d_points_init_data;
	struct trixel_memory {
		struct points { double x; double y; double z; }*h_p1, *d_p1;
		struct edges {
			struct edge { double* x; double* y; double* z; }e1, e2;//,e3;
		}d_edges;
		struct surface_normals { double* x; double* y; double* z; }d_n;
		struct color { union { struct { u8 b; u8 g; u8 r; u8 a; }*argb; u32* c; }; }d_color;
	}h_mem;void* d_mem;

	Trixel(u64 num_t, double* points_data, u32* color_data) {
		num_trixels = num_t;

		cudaMalloc((void**)&h_mem.d_p1, sizeof(trixel_memory::points) * num_trixels);
		h_mem.h_p1 = (trixel_memory::points*)malloc(sizeof(trixel_memory::points) * num_trixels);

		cudaMalloc((void**)&h_mem.d_edges.e1.x, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_edges.e1.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_edges.e1.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&h_mem.d_edges.e2.x, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_edges.e2.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_edges.e2.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&h_mem.d_n.x, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_n.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_mem.d_n.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&h_mem.d_color.c, sizeof(u32) * num_trixels);

		cudaMalloc((void**)&d_mem, sizeof(trixel_memory));
		cudaMemcpy(d_mem, &h_mem, sizeof(trixel_memory), cudaMemcpyHostToDevice);


		cudaMalloc((void**)&d_points_init_data, sizeof(double) * num_trixels);
		cudaMemcpy(d_points_init_data, points_data, sizeof(double) * num_trixels, cudaMemcpyHostToDevice);

		cudaMemcpy(h_mem.d_color.c, color_data, sizeof(u32) * num_trixels, cudaMemcpyHostToDevice);

		init_trixels_device_memory(this);
	};
	cudaError_t init_camera_trixel_data(Camera* c) {
		cudaMalloc((void**)&c->h_trixels.d_q.x, sizeof(double) * num_trixels);
		cudaMalloc((void**)&c->h_trixels.d_q.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&c->h_trixels.d_q.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&(c->h_trixels.d_t.x), sizeof(double) * num_trixels);
		cudaMalloc((void**)&c->h_trixels.d_t.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&c->h_trixels.d_t.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&c->h_trixels.d_w, sizeof(double) * num_trixels);

		cudaMalloc((void**)&(c->d_trixels), sizeof(trixel_memory));
		cudaMemcpy(c->d_trixels, &(c->h_trixels), sizeof(trixel_memory), cudaMemcpyHostToDevice);

		return init_camera_trixel_device_memory(this, c);
	}
	cudaError_t intersect_trixels(Camera* c) {
		return intersect_trixels_device(this, c);
	}
};


