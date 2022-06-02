#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "Vector.h"
#include "cuda_runtime.h"
#include "Color.h"
#include <math.h>
class Camera;
class Trixel;
extern "C" cudaError_t init_camera_device_memory(Camera* c);
extern "C" cudaError_t init_camera_trixel_device_memory(Trixel* t, Camera* c);
extern "C" cudaError_t init_camera_voxel_device_memory(Trixel* t, Camera* c);
extern "C" cudaError_t transform_camera_voxel_device_memory(Camera * c, Input::translate_vector tv, u8 transform_select);
extern "C" cudaError_t color_camera_device(Camera * c);
extern "C" cudaError_t transform_trixels_device(Trixel * t, Camera * c, Input::translate_vector scale_factor, u8 transform_select);


class Camera
{
public:
	//Camera Properties
	struct film_properties{// film/screen
		struct resolution { u32 w; u32 h; u64 count;}res; //number of pixels
		struct pixel { double w; double h; }pix; //size of pixels
		struct film { double w; double h; }film; //
		film_properties() : film(), pix(), res() {}
	}f_prop;
	struct lens_properties {
		double focal_length;
		lens_properties() : focal_length() {}
	}l_prop;
	struct orientation_properties {
		vector_xyz pos; //world cords
		struct look_at { double x; double y; double z; }la;//position
		struct up { double x; double y; double z; }up;//position
		// n = norm(look_at - position)     n_mod = direction at bottom left pixel
		// v = norm(n cross ( up cross n))  v_mod = v / pixel height
		// u = n cross v                    u_mod = u / pixel width
		struct o_vector { double x; double y; double z;}n, v, u, n_mod, v_mod, u_mod;
		orientation_properties() : n(), v(), u(), n_mod(), v_mod(), u_mod(), up(), la(), pos() {}
	}o_prop;
	struct render_properites {
		double draw_distance;
		int sample_rate;
		render_properites() :sample_rate(0), draw_distance(400) {}
	}r_prop;
	struct pixel_memory {
		//each value in here should be a matrix of size pix num
		// i.e each pointer should have a value alloced for each pixel
		struct radiance { double* d_r; double* d_g; double* d_b; }rad; //Radiance at pixel
		Color h_color, d_color; //color at pixel
		struct distance { double* d; }d_dist;
		struct surface_normal { double* d_x; double* d_y; double* d_z; }norm;
		struct intersection_point { double* d_x; double* d_y; double* d_z; }pnt;
		// Recalculate rmd on Camera rotation (look_at change)
		struct ray_direction { double* d_x; double* d_y; double* d_z; }rmd,inv_rmd,sign_rmd;
		struct intersection_object_index { s64* index; }d_rmi, h_rmi;//index into list of triangles
		pixel_memory() : d_rmi(), h_rmi(), rmd(), inv_rmd(), sign_rmd(), pnt(), norm(), d_dist(), h_color(), d_color(), rad() {}
	}h_mem; void* d_mem = NULL;
	Trixel* trixels_list = NULL;
	//struct voxel_traverse_list { u64 ray_index, node_tri; s32 node_cur, node_left, node_right; } *h_next_voxels_first, * h_next_voxels_second, * d_next_voxels_first, * d_next_voxels_second,*h_trixel_ray_check_list, *d_trixel_ray_check_list;
	//Recalcuate on Camera position move
	struct trixel_memory {
		struct trixel_vector { double* x; double* y; double* z; trixel_vector() :x(0), y(0), z(0) {} }d_t, d_q;
		double* d_w;
		trixel_memory() : d_t(), d_q(), d_w(NULL) {}
	}h_trixels; void* d_trixels = NULL;
	struct voxel_memory {
		struct voxel_vector {			
			double t0x; double t0y; double t0z;
			double t1x; double t1y; double t1z;
		}*d_Bo;
		double* s1, *s2;
		u8* is_leaf;
		struct childs { s64 left, right, triangle, parent;
		}*children;
		s32* d_voxel_index_queue;
		u32 index_queue_offset;
		struct flag_tag{ u8 x; u8 y; u8 z; }*cut_flags;
		s64 num_voxels;
		voxel_memory() : d_Bo(), s1(NULL), s2(NULL), is_leaf(0), children(NULL), d_voxel_index_queue(NULL),index_queue_offset(0),cut_flags(NULL),num_voxels(0) {}
	}h_voxels; void* d_voxels = NULL;

	Camera(s32 r_w, s32 r_h,
		double f_w, double f_h, double fclen,
		double p_x, double p_y, double p_z,
		double la_x, double la_y, double la_z,
		double up_x, double up_y, double up_z
	);

	cudaError_t init_camera_trixel_data(Trixel* t, s64 num_trixels);
	cudaError_t init_camera_voxel_data(Trixel* t, s64 num_voxels);
	
	//ALSO REMBER IF YOU WANT TO DO THIS FOR PRIMITVIES MAKE SURE TO CALL TRANSFORM ON ALL CAMERAS, or else you will move the camera adn not the object
	cudaError_t transform(Input::translate_vector tv, u8 transform_select);

	cudaError_t color_pixels();

};
#endif