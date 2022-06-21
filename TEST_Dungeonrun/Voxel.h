#pragma once
#include "typedefs.h"

class Voxel
{
	struct voxel_memory {
		struct voxel_vector {
			double t0x; double t0y; double t0z;
			double t1x; double t1y; double t1z;
		}*d_Bo; //bounds
		double* s1, * s2; // cuts
		u8* is_leaf;
		struct childs {
			s64 left, right, triangle, parent;
		}*children;
		s32* d_voxel_index_queue;
		u32 index_queue_offset;
		struct flag_tag { u8 x; u8 y; u8 z; }*cut_flags;
		s64 num_voxels;
		voxel_memory() : d_Bo(), s1(NULL), s2(NULL), is_leaf(0), children(NULL), d_voxel_index_queue(NULL), index_queue_offset(0), cut_flags(NULL), num_voxels(0) {}
	}h_voxels; void* d_voxels = NULL;
};

