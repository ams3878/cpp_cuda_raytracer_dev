#pragma once
#ifndef TRIXEL_H
#define TRIXEL_H
#include "Vector.h"
#include "cuda_runtime.h"
#include "sort.h"

struct Trixel;
struct Camera;
extern "C" cudaError_t intersect_trixels_device(Trixel * t, Camera * c);
extern "C" cudaError_t init_trixels_device_memory(Trixel* t);
struct trixel_point_xyz { double x; double y; double z; };

class Trixel
{
	trixel_point_xyz* sorted_x_points;
	trixel_point_xyz* sorted_y_points;
	trixel_point_xyz* sorted_z_points;
	u64 num_vertices;
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

	struct k_d_tree {}tree;

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


		cudaMalloc((void**)&d_points_init_data, sizeof(double) * num_trixels * 3 * 3);
		cudaMemcpy(d_points_init_data, points_data, sizeof(double) * num_trixels * 3 * 3, cudaMemcpyHostToDevice);

		cudaMemcpy(h_mem.d_color.c, color_data, sizeof(u32) * num_trixels, cudaMemcpyHostToDevice);

		init_trixels_device_memory(this);
	};

	int set_sorted_points(double* vertex_list, u64 num_vert) {
		num_vertices = num_vert;
		trixel_point_xyz* w_list = (trixel_point_xyz*)malloc(sizeof(trixel_point_xyz) * num_vert);

		sorted_x_points = (trixel_point_xyz*)malloc(sizeof(trixel_point_xyz) * num_vert);
		cudaMemcpy(sorted_x_points, vertex_list, sizeof(trixel_point_xyz) * num_vert, cudaMemcpyHostToHost);
		merge_sort(sorted_x_points, w_list, num_vert , SORT_X_TAG);

		sorted_y_points = (trixel_point_xyz*)malloc(sizeof(trixel_point_xyz) * num_vert);
		cudaMemcpy(sorted_y_points, vertex_list, sizeof(trixel_point_xyz) * num_vert, cudaMemcpyHostToHost);
		merge_sort(sorted_y_points, w_list, num_vert , SORT_Y_TAG);

		sorted_z_points = (trixel_point_xyz*)malloc(sizeof(trixel_point_xyz) * num_vert);
		cudaMemcpy(sorted_z_points, vertex_list, sizeof(trixel_point_xyz) * num_vert, cudaMemcpyHostToHost);
		merge_sort(sorted_z_points, w_list, num_vert , SORT_Z_TAG);

		return 0;
	}
	cudaError_t intersect_trixels(Camera* c) {
		return intersect_trixels_device(this, c);	}
};

#endif
