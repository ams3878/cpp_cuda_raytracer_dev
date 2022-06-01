#pragma once
#ifndef TRIXEL_H
#define TRIXEL_H
#include "Vector.h"
#include "cuda_runtime.h"
#include "sort.h"
#include <stdio.h>

class Trixel;
class Camera;
extern "C" cudaError_t intersect_trixels_device(Trixel * t, Camera * c, u32 mode);
extern "C" cudaError_t init_trixels_device_memory(Trixel* t);
extern "C" cudaError_t transform_trixels_device(Trixel * t, Camera * c, Input::translate_vector scale_factor, u8 transform_select);


struct kd_vertex { double x; double y; double z; };
 //FOR SOME REASON THIS SEEMS TO BREAK IN GPU. but i think works fine in cpu
// merge sort needs x,y,z to work
struct kd_leaf_sort {
	double x0; double x1;
	double y0; double y1;
	double z0; double z1;

	s64 sorted_x0_index, sorted_y0_index, sorted_z0_index, sorted_x1_index, sorted_y1_index, sorted_z1_index, tri_list_index;

	kd_leaf_sort() : x1(0.0), x0(0.0), y1(0.0), y0(0.0), z1(0.0), z0(0.0),
		sorted_x1_index(0), sorted_x0_index(0), sorted_y1_index(0), sorted_y0_index(0), sorted_z1_index(0), sorted_z0_index(0),
		tri_list_index(0) {}
};
struct kd_leaf {
	double x0; double x1;
	double y0; double y1;
	double z0; double z1;
	s64 tri_list_index;
	kd_leaf() : x1(0.0), x0(0.0), y1(0.0), y0(0.0), z1(0.0), z0(0.0), tri_list_index(0) {}
};
class Trixel
{
	kd_leaf_sort* sorted_x0_leafs;
	kd_leaf_sort* sorted_y0_leafs;
	kd_leaf_sort* sorted_z0_leafs;
	kd_leaf_sort* sorted_x1_leafs;
	kd_leaf_sort* sorted_y1_leafs;
	kd_leaf_sort* sorted_z1_leafs;
	kd_leaf_sort* indexed_leafs;
public:
	s64 num_trixels;
	s64 num_vertices;
	s64 num_voxels;

	double* d_points_init_data; //tri list data [p0x, p0y, p0z, p1x, p1ym p1z, p2x, p2y p3z,....for each triangle]
	double* h_points_init_data; // i.e num_trixel * 3(points) * 3(component)
	struct trixel_memory {
		struct points { double x; double y; double z; }*h_p1, *d_p1;
		struct edges {
			struct edge { double* x; double* y; double* z; }e1, e2;//,e3;
		}d_edges;
		color d_color;
		struct surface_normals { double* x; double* y; double* z; }d_n;
		trixel_memory() : d_color(), d_n(), d_edges(), h_p1(NULL), d_p1(NULL) {}
	}h_mem;void* d_mem;
	struct kd_tree {
		struct kd_tree_node {
			s64 tri_index = -1;
			kd_leaf h_bound;
			int cut_flag;
			double s1,s2;
			u8 is_leaf = 0;
			s64  l, m, r;
			s64 left_node = -2;
			s64 right_node = -2;
			s64 parent = NULL;
			kd_tree_node() : parent(0), right_node(0), left_node(0), l(0), m(0), r(0), is_leaf(0), s1(0.0), s2(0.0), cut_flag(0), h_bound(), tri_index(-1) {}
		}*h_nodes, *d_nodes;
		u8 min_node_size = 1; //**TODO** generalize this resolution
		kd_tree() : d_nodes(NULL), h_nodes(NULL), min_node_size(1) {}
	}h_tree; void* d_tree;
	Trixel() : d_tree(NULL), d_mem(NULL), d_points_init_data(NULL), h_tree(), h_mem(), h_points_init_data(NULL),
		sorted_x0_leafs(NULL), sorted_x1_leafs(NULL), sorted_y0_leafs(NULL), sorted_y1_leafs(NULL), sorted_z0_leafs(NULL), sorted_z1_leafs(NULL),
		indexed_leafs(NULL), num_trixels(0), num_vertices(0), num_voxels(0) {}

	Trixel(s64 num_t, double* points_data, color* color_data) : Trixel() {
		num_trixels = num_t;

		h_mem.h_p1 = (trixel_memory::points*)malloc(sizeof(trixel_memory::points) * num_trixels);
		cudaMalloc((void**)&h_mem.d_p1, sizeof(trixel_memory::points) * num_trixels);

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
		cudaMalloc((void**)&h_mem.d_color.rad, sizeof(color::radiance) * num_trixels);

		cudaMalloc((void**)&d_mem, sizeof(trixel_memory));
		cudaMemcpy(d_mem, &h_mem, sizeof(trixel_memory), cudaMemcpyHostToDevice);


		num_voxels = num_trixels * 2 - 1;
		h_tree.h_nodes = (kd_tree::kd_tree_node*)malloc(sizeof(kd_tree::kd_tree_node) * num_voxels);
		cudaMalloc((void**)&h_tree.d_nodes, sizeof(kd_tree::kd_tree_node) * num_voxels);

		cudaMalloc((void**)&d_tree, sizeof(kd_tree));
		cudaMemcpy(d_tree, &h_tree, sizeof(kd_tree), cudaMemcpyHostToDevice);


		cudaMalloc((void**)&d_points_init_data, sizeof(double) * num_trixels * 3 * 3);
		h_points_init_data = (double*)malloc(sizeof(double) * num_trixels * 3 * 3);

		cudaMemcpy(h_points_init_data, points_data, sizeof(double) * num_trixels * 3 * 3, cudaMemcpyHostToHost);
		cudaMemcpy(d_points_init_data, points_data, sizeof(double) * num_trixels * 3 * 3, cudaMemcpyHostToDevice);

		cudaMemcpy(h_mem.d_color.c, color_data->c, sizeof(u32) * num_trixels, cudaMemcpyHostToDevice);
		cudaMemcpy(h_mem.d_color.rad, color_data->rad, sizeof(color::radiance) * num_trixels, cudaMemcpyHostToDevice);

		init_trixels_device_memory(this);
	};

	int create_kd() {
		if (sorted_x1_leafs == NULL) { return -12; }
		if (sorted_x0_leafs == NULL) { return -12; }
		if (sorted_y1_leafs == NULL) { return -12; }
		if (sorted_y0_leafs == NULL) { return -12; }
		if (sorted_z1_leafs == NULL) { return -12; }
		if (sorted_z0_leafs == NULL) { return -12; }

		s64 read_index = 0, write_index = 1;
		s64 l, m, r;
		kd_tree::kd_tree_node* cur_node;

		h_tree.h_nodes[read_index].l = 0;
		h_tree.h_nodes[read_index].m = (num_trixels-1) / 2;
		h_tree.h_nodes[read_index].r = num_trixels - 1;

		h_tree.h_nodes[read_index].parent = 0; // Careful this can cause infinte loop if not careful.
		h_tree.h_nodes[read_index].cut_flag = 5;
		h_tree.h_nodes[read_index].is_leaf = 0;

		h_tree.h_nodes[read_index].h_bound.z1 = sorted_z1_leafs[num_trixels - 1].z1;
		h_tree.h_nodes[read_index].h_bound.z0 = sorted_z0_leafs[0].z0;
		h_tree.h_nodes[read_index].h_bound.y1 = sorted_y1_leafs[num_trixels - 1].y1;
		h_tree.h_nodes[read_index].h_bound.y0 = sorted_y0_leafs[0].y0;
		h_tree.h_nodes[read_index].h_bound.x0 = sorted_x0_leafs[0].x0;
		h_tree.h_nodes[read_index].h_bound.x1 = sorted_x1_leafs[num_trixels-1].x1;

		kd_leaf_sort** temp_lists_list = (kd_leaf_sort**)malloc(sizeof(kd_leaf_sort*) * 6);
		// maybe move this malloc back into loopp if you need smaller concurent memory footprint (it shrinks as you cut the lists)
		// But thats a lot of malloc/frees log(n) ? maybe not that many i guess. wait...per node? 2n ? hmm not sure.
		//kd_leaf_sort* temp_d_list = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * (h_tree.h_nodes[read_index].r - h_tree.h_nodes[read_index].l + 1));

		while (read_index < write_index) {
			cur_node = &(h_tree.h_nodes[read_index]);
			l = cur_node->l;
			m = cur_node->m;
			r = cur_node->r;
			double max_split = (sorted_x1_leafs[r].x1 - sorted_x1_leafs[l].x1);
			int max_cut = 0;
			if (sorted_x0_leafs[r].x0 - sorted_x0_leafs[l].x0 > max_split) {
				max_split = sorted_x0_leafs[r].x0 - sorted_x0_leafs[l].x0;
				max_cut = 3;
			}
			if (sorted_y1_leafs[r].y1 - sorted_y1_leafs[l].y1 > max_split) {
				max_split = sorted_y1_leafs[r].y1 - sorted_y1_leafs[l].y1;
				max_cut = 1;
			}			
			if (sorted_y0_leafs[r].y0 - sorted_y0_leafs[l].y0 > max_split) {
				max_split = sorted_y0_leafs[r].y0 - sorted_y0_leafs[l].y0;
				max_cut = 4;
			}	
			if (sorted_z1_leafs[r].z1 - sorted_z1_leafs[l].z1 > max_split) {
				max_split = sorted_z1_leafs[r].z1 - sorted_z1_leafs[l].z1;
				max_cut = 2;
			}
			if (sorted_z0_leafs[r].z0 - sorted_z0_leafs[l].z0 > max_split) {
				max_split = sorted_z0_leafs[r].z0 - sorted_z0_leafs[l].z0;
				max_cut = 5;
			}
			cur_node->cut_flag = (r - l) != (h_tree.min_node_size - 1) ? max_cut : h_tree.h_nodes[cur_node->parent].cut_flag;

			//cur_node->cut_flag = (r - l) != (h_tree.min_node_size - 1) ? (h_tree.h_nodes[cur_node->parent].cut_flag + 1) % 6 : h_tree.h_nodes[cur_node->parent].cut_flag;

			if (r - l == h_tree.min_node_size - 1) {
				cur_node->left_node = -1;
				cur_node->right_node = -1;
				cur_node->is_leaf = 1;
				cur_node->tri_index = sorted_x1_leafs[l].tri_list_index;
				read_index++;
				continue;
			}

			kd_leaf_sort* f_list, r_list;
			s64 temp_index, index_to_pivot;
			if ((r - l + 20) == NULL) { return 0; }
			kd_leaf_sort* temp_d_list = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * (r - l + 20));

			//divide voxels aroudn the median

			for (int list_index = 0; list_index <= 5; list_index++) {
				//SELECT WHICH LIST IS GOING TO BE MODIFIED AROUND PIVOT
				if (cur_node->cut_flag == list_index) { continue; }
				switch (list_index) {
				case 0:
					f_list = sorted_x1_leafs;
					break;
				case 1:
					f_list =  sorted_y1_leafs;
;					break;
				case 2:
					f_list = sorted_z1_leafs;
					break;
				case 3:
					f_list = sorted_x0_leafs;
					break;
				case 4:
					f_list = sorted_y0_leafs;
					break;
				case 5:
					f_list = sorted_z0_leafs;
					break;
				}
				for (s64 i = l, l_i = 0, r_i = m - l+1; i <= r; i++) {
					// SELECT THE PIVOT POINT
					// right now only around right (x1,y1,z1) points, could be changed to consider bother left and right points
					// and select the btter to pivot. Likely do an are based huersitic (see Joes notes) instead though.
					switch (cur_node->cut_flag) {
					case 0:
						index_to_pivot = f_list[i].sorted_x1_index;
						break;
					case 1:
						index_to_pivot = f_list[i].sorted_y1_index;
						break;
					case 2:
						index_to_pivot = f_list[i].sorted_z1_index;
						break;
					case 3:
						index_to_pivot = f_list[i].sorted_x0_index;
						break;
					case 4:
						index_to_pivot = f_list[i].sorted_y0_index;
						break;
					case 5:
						index_to_pivot = f_list[i].sorted_z0_index;
						break;
					}
					temp_index = index_to_pivot  <= m ? l_i++ : r_i++; // ele at m goes right
					//temp_d_list will be the new list replacing the one to be sorted around pivot. All other lists stay the same expect that one.
					// x1, y1, z1 could be cut planes. if they are they do not change
					temp_d_list[temp_index].sorted_x1_index = list_index == 0 ? l + temp_index : f_list[i].sorted_x1_index;
					temp_d_list[temp_index].sorted_y1_index = list_index == 1 ? l + temp_index : f_list[i].sorted_y1_index;
					temp_d_list[temp_index].sorted_z1_index = list_index == 2 ? l + temp_index : f_list[i].sorted_z1_index;
					temp_d_list[temp_index].sorted_x0_index = list_index == 3 ? l + temp_index : f_list[i].sorted_x0_index;
					temp_d_list[temp_index].sorted_y0_index = list_index == 4 ? l + temp_index : f_list[i].sorted_y0_index;
					temp_d_list[temp_index].sorted_z0_index = list_index == 5 ? l + temp_index : f_list[i].sorted_z0_index;

					temp_lists_list[0] = &sorted_x1_leafs[temp_d_list[temp_index].sorted_x1_index];
					temp_lists_list[1] = &sorted_y1_leafs[temp_d_list[temp_index].sorted_y1_index];
					temp_lists_list[2] = &sorted_z1_leafs[temp_d_list[temp_index].sorted_z1_index];
					temp_lists_list[3] = &sorted_x0_leafs[temp_d_list[temp_index].sorted_x0_index];
					temp_lists_list[4] = &sorted_y0_leafs[temp_d_list[temp_index].sorted_y0_index];
					temp_lists_list[5] = &sorted_z0_leafs[temp_d_list[temp_index].sorted_z0_index];
					// For each list that needs updated with new re-sorted positions
					for (s64 sorted_list_select = 0; sorted_list_select <= 5; sorted_list_select++) {
	
						if (sorted_list_select == list_index) { continue; } // if  its the list that got sorted, skip it

						switch (list_index) {// list_index is the list being re-sorted
						case 0: // x1 is the list being re - sorted
							temp_lists_list[sorted_list_select]->sorted_x1_index = l + temp_index;
							break;
						case 1: // y1 is the list being re - sorted
							temp_lists_list[sorted_list_select]->sorted_y1_index = l + temp_index;
							break;
						case 2:
							temp_lists_list[sorted_list_select]->sorted_z1_index = l + temp_index;
							break;
						case 3:
							temp_lists_list[sorted_list_select]->sorted_x0_index = l + temp_index;
							break;
						case 4:
							temp_lists_list[sorted_list_select]->sorted_y0_index = l + temp_index;
							break;
						case 5:
							temp_lists_list[sorted_list_select]->sorted_z0_index = l + temp_index;
							break;
						}
					}
					temp_d_list[temp_index].z1 = f_list[i].z1;
					temp_d_list[temp_index].z0 = f_list[i].z0;
					temp_d_list[temp_index].x0 = f_list[i].x0;
					temp_d_list[temp_index].x1 = f_list[i].x1;
					temp_d_list[temp_index].y1 = f_list[i].y1;
					temp_d_list[temp_index].y0 = f_list[i].y0;
					temp_d_list[temp_index].tri_list_index = f_list[i].tri_list_index;
				}
				//Once all leafs have been re sorted around pivot, update original list.
				//cudaMemcpy(f_list + l, temp_d_list, sizeof(kd_leaf_sort)* (r - l) + 1, cudaMemcpyHostToHost);
				
				for (s64 i = l; i <= r; i++) {
					f_list[i].z1 = temp_d_list[i - l].z1;
					f_list[i].z0 = temp_d_list[i - l].z0;
					f_list[i].x0 = temp_d_list[i - l].x0;
					f_list[i].x1 = temp_d_list[i - l].x1;
					f_list[i].y1 = temp_d_list[i - l].y1;
					f_list[i].y0 = temp_d_list[i - l].y0;
					f_list[i].tri_list_index = temp_d_list[i - l].tri_list_index;
					f_list[i].sorted_x1_index = temp_d_list[i - l].sorted_x1_index;
					f_list[i].sorted_y1_index = temp_d_list[i - l].sorted_y1_index;
					f_list[i].sorted_z1_index = temp_d_list[i - l].sorted_z1_index;
					f_list[i].sorted_x0_index = temp_d_list[i - l].sorted_x0_index;
					f_list[i].sorted_y0_index = temp_d_list[i - l].sorted_y0_index;
					f_list[i].sorted_z0_index = temp_d_list[i - l].sorted_z0_index;
				}
			}
			free(temp_d_list);
			for (int branch = 0; branch < 2; branch++) {
				h_tree.h_nodes[write_index].parent = read_index;
				h_tree.h_nodes[write_index].is_leaf = 0;
				u64 new_left, new_right;
				
				if (branch == 0) {
					new_left = l; new_right = m;
					cur_node->left_node = write_index; 
				}else {
					new_left = m + 1; new_right = r;
					cur_node->right_node = write_index;
				}
				h_tree.h_nodes[write_index].l = new_left;
				h_tree.h_nodes[write_index].r = new_right;
				h_tree.h_nodes[write_index].m = ((new_right - new_left) / 2) + new_left;

				h_tree.h_nodes[write_index].h_bound.x1 = sorted_x1_leafs[new_right].x1;
				h_tree.h_nodes[write_index].h_bound.x0 = sorted_x0_leafs[new_left].x0;
				h_tree.h_nodes[write_index].h_bound.y1 = sorted_y1_leafs[new_right].y1;
				h_tree.h_nodes[write_index].h_bound.y0 = sorted_y0_leafs[new_left].y0;
				h_tree.h_nodes[write_index].h_bound.z1 = sorted_z1_leafs[new_right].z1;
				h_tree.h_nodes[write_index].h_bound.z0 = sorted_z0_leafs[new_left].z0;
				write_index++;
			}
			u8 left = 2, right = 1;
			switch (cur_node->cut_flag) {
				//s1 is normal split s2 is extra adjusted plane
			case 3:
			case 0:
				h_tree.h_nodes[read_index].s2 = h_tree.h_nodes[write_index - right].h_bound.x0;
				h_tree.h_nodes[read_index].s1 = h_tree.h_nodes[write_index - left].h_bound.x1;
				break;
			case 4:
			case 1:
				h_tree.h_nodes[read_index].s2 = h_tree.h_nodes[write_index - right].h_bound.y0;
				h_tree.h_nodes[read_index].s1 = h_tree.h_nodes[write_index - left].h_bound.y1;
				break;
			case 5:
			case 2:
				h_tree.h_nodes[read_index].s2 = h_tree.h_nodes[write_index - right].h_bound.z0;
				h_tree.h_nodes[read_index].s1 = h_tree.h_nodes[write_index - left].h_bound.z1;
				break;
			}
			
			read_index++;
		}
		cudaMemcpy(h_tree.d_nodes, h_tree.h_nodes, sizeof(kd_tree::kd_tree_node) * num_voxels, cudaMemcpyHostToDevice);
		free(sorted_x1_leafs);	free(sorted_y1_leafs);	free(sorted_z1_leafs);
		free(sorted_x0_leafs);	free(sorted_y0_leafs);	free(sorted_z0_leafs);
		free(temp_lists_list);
		return 0;
	}
	int set_sorted_voxels(kd_leaf_sort* voxel_list, u64 num_leaf_voxels) {

		indexed_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(indexed_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);

		num_vertices = num_leaf_voxels;
		kd_leaf_sort* w_list = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		
		sorted_x0_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		sorted_x1_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(sorted_x0_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		cudaMemcpy(sorted_x1_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		merge_sort(sorted_x0_leafs, w_list, num_leaf_voxels, SORT_X0_TAG);
		merge_sort(sorted_x1_leafs, w_list, num_leaf_voxels, SORT_X1_TAG);

		sorted_y0_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(sorted_y0_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		merge_sort(sorted_y0_leafs, w_list, num_leaf_voxels, SORT_Y0_TAG);
		sorted_y1_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(sorted_y1_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		merge_sort(sorted_y1_leafs, w_list, num_leaf_voxels, SORT_Y1_TAG);

		sorted_z0_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(sorted_z0_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		merge_sort(sorted_z0_leafs, w_list, num_leaf_voxels, SORT_Z0_TAG);
		sorted_z1_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * num_leaf_voxels);
		cudaMemcpy(sorted_z1_leafs, voxel_list, sizeof(kd_leaf_sort) * num_leaf_voxels, cudaMemcpyHostToHost);
		merge_sort(sorted_z1_leafs, w_list, num_leaf_voxels, SORT_Z1_TAG);

		for (int i = 0; i < num_leaf_voxels; i++) {
			indexed_leafs[sorted_x0_leafs[i].tri_list_index].sorted_x0_index = i;
			indexed_leafs[sorted_y0_leafs[i].tri_list_index].sorted_y0_index = i;
			indexed_leafs[sorted_z0_leafs[i].tri_list_index].sorted_z0_index = i;
			indexed_leafs[sorted_x1_leafs[i].tri_list_index].sorted_x1_index = i;
			indexed_leafs[sorted_y1_leafs[i].tri_list_index].sorted_y1_index = i;
			indexed_leafs[sorted_z1_leafs[i].tri_list_index].sorted_z1_index = i;
		}
		kd_leaf_sort* f_list;
		for (int list_select = 0; list_select <= 5; list_select++) {
			switch (list_select) {
			case 0:
				f_list = sorted_x1_leafs;
				break;
			case 1:
				f_list = sorted_y1_leafs;
				break;
			case 2:
				f_list = sorted_z1_leafs;
				break;
			case 3:
				f_list = sorted_x0_leafs;
				break;
			case 4:
				f_list = sorted_y0_leafs;
				break;
			case 5:
				f_list = sorted_z0_leafs;
				break;
			}		
			for (int i = 0; i < num_leaf_voxels; i++) {
				//SET X0 LEAF position for all other lists
				f_list[i].sorted_x1_index = list_select == 0 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_x1_index;
				f_list[i].sorted_y1_index = list_select == 1 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_y1_index;
				f_list[i].sorted_z1_index = list_select == 2 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_z1_index;
				f_list[i].sorted_x0_index = list_select == 3 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_x0_index;
				f_list[i].sorted_y0_index = list_select == 4 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_y0_index;
				f_list[i].sorted_z0_index = list_select == 5 ? i : indexed_leafs[f_list[i].tri_list_index].sorted_z0_index;
			}
		}
		free(indexed_leafs);
		return 0;
	}
	cudaError_t intersect_trixels(Camera* c, u32 m) {
		return intersect_trixels_device(this, c, m);	
	}

};

#endif


