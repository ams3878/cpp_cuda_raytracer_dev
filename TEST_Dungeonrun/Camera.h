#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "Vector.h"
#include <math.h>
#include "cuda_runtime.h"
#include "typedefs.h"
#include "Color.h"
#include "Quaternion.h"
class Trixel;
class Input;
class Camera
{
public:

	//Camera Properties
	struct film_properties{// film/screen
		struct resolution { u32 w; u32 h; u64 count;}res; //number of pixels
		struct pixel { T_fp w; T_fp h; }pix; //size of pixels
		struct film { T_fp w; T_fp h; }film; //
		film_properties() : film(), pix(), res() {}
	}f_prop;
	struct lens_properties {
		T_fp focal_length;
		lens_properties() : focal_length() {}
	}l_prop;
	struct orientation_properties {
		//world cords
		VEC3<T_fp> pos, la, up;//position
		// n = norm(look_at - position)     n_mod = direction at bottom left pixel
		// v = norm(n cross ( up cross n))  v_mod = v / pixel height
		// u = n cross v                    u_mod = u / pixel width
		VEC3<T_fp> n, v, u, n_mod, v_mod, u_mod;
		VEC3<T_fp>** rotation_help_array;

		orientation_properties() : n(), v(), u(), n_mod(), v_mod(), u_mod(), up(), la(), pos(), rotation_help_array(NULL) {}
	}o_prop;

	struct render_properites {
		T_fp draw_distance;
		int sample_rate;
		render_properites() :sample_rate(0), draw_distance(400) {}
	}r_prop;
	struct pixel_memory {
		//each value in here should be a matrix of size pix num
		// i.e each pointer should have a value alloced for each pixel
		VEC3_CUDA<T_fp> rad, norm, pnt, rmd, inv_rmd, sign_rmd; //Radiance at pixel
		Color h_color, d_color; //color at pixel
		struct distance { T_fp* d; }d_dist;
		// Recalculate rmd on Camera rotation (look_at change)
		struct intersection_object_index { s64* index; }d_rmi, h_rmi;//index into list of triangles
		pixel_memory() : d_rmi(), h_rmi(), rmd(), inv_rmd(), sign_rmd(), pnt(), norm(), d_dist(), h_color(), d_color(), rad() {}
	}h_mem, *d_mem;
	Trixel* trixels_list = NULL;
	//struct voxel_traverse_list { u64 ray_index, node_tri; s32 node_cur, node_left, node_right; } *h_next_voxels_first, * h_next_voxels_second, * d_next_voxels_first, * d_next_voxels_second,*h_trixel_ray_check_list, *d_trixel_ray_check_list;
	//Recalcuate on Camera position move
	struct trixel_memory {
		VEC3_CUDA<T_fp> d_t, d_q;
		T_fp* d_w;
		trixel_memory() : d_t(), d_q(), d_w(NULL) {}
	}h_trixels, *d_trixels = NULL;
	struct voxel_memory {
		struct voxel_vector {			
			T_fp t0x; T_fp t0y; T_fp t0z;
			T_fp t1x; T_fp t1y; T_fp t1z;
		}*d_Bo,*h_Bo;
		Quaternion* q,*d_q, * next_rotation;
		T_fp* s1, *s2;
		u8* is_leaf;
		VEC4<T_fp> n,u,v,cur_transform, cur_rotation,init_face, obj_center;
		struct childs { s64 left, right, triangle, parent;
		}*children;
		s32* d_voxel_index_queue;
		u32 index_queue_offset;
		struct flag_tag{ u8 x; u8 y; u8 z; }*cut_flags;
		s64 num_voxels;
		voxel_memory() : d_Bo(), s1(NULL), s2(NULL), is_leaf(0), children(NULL), d_voxel_index_queue(NULL),index_queue_offset(0),cut_flags(NULL),num_voxels(0) {}
	}h_voxels, *d_voxels;

	Camera(s32 r_w, s32 r_h,
		T_fp f_w, T_fp f_h, T_fp fclen,
		T_fp p_x, T_fp p_y, T_fp p_z,
		T_fp la_x, T_fp la_y, T_fp la_z,
		T_fp up_x, T_fp up_y, T_fp up_z
	);

	cudaError_t init_camera_trixel_data(Trixel* t, s64 num_trixels);

	cudaError_t init_camera_voxel_data(Trixel* t, s64 num_voxels);
	
	//ALSO REMBER IF YOU WANT TO DO THIS FOR PRIMITVIES MAKE SURE TO CALL TRANSFORM ON ALL CAMERAS, or else you will move the camera adn not the object
	cudaError_t transform(Input* input, u8 transform_select);

	cudaError_t color_pixels();
	cudaError_t rotate(Quaternion* q, int select);
};

extern "C" cudaError_t init_camera_device_memory(Camera * c);
extern "C" cudaError_t init_camera_trixel_device_memory(Trixel * t, Camera * c);
extern "C" cudaError_t init_camera_voxel_device_memory(Trixel * t, Camera * c);
extern "C" cudaError_t transform_camera_voxel_device_memory(Camera * c, Trixel * t, VEC4<T_fp>* tv, Quaternion * q,  u8 transform_select);
extern "C" cudaError_t color_camera_device(Camera * c);
//extern "C" cudaError_t transform_trixels_device(Camera * c, Trixel * t, VEC4<T_fp>* tv, Quaternion * q, u8 transform_select);
#endif