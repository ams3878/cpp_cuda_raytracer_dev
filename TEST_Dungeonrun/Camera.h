#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "Vector.h"
#include "cuda_runtime.h"
#include <math.h>
class Camera;
class Trixel;
extern "C" cudaError_t init_camera_device_memory(Camera* c);
extern "C" cudaError_t init_camera_trixel_device_memory(Trixel* t, Camera* c);
extern "C" cudaError_t init_camera_voxel_device_memory(Trixel* t, Camera* c);
extern "C" cudaError_t update_camera_voxel_device_memory(Input::translate_vector tv, Camera * c);
extern "C" cudaError_t color_camera_device(Camera * c);


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
		struct color { union  { struct { u8 b; u8 g; u8 r; u8 a; }*argb; u32* c; }; }h_color, d_color; //color at pixel
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
		struct childs { s64 left, right,triangle; }*children;
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
	) {
		f_prop.res.w = r_w;
		f_prop.res.h = r_h;
		f_prop.res.count = static_cast<u64>(r_w) * static_cast<u64>(r_h);
		f_prop.film.w = f_w;
		f_prop.film.w = f_h;
		f_prop.pix.w = f_w / (double)r_w;
		f_prop.pix.h = f_h / (double)r_h;
		l_prop.focal_length = fclen;

		o_prop.pos.x = p_x;
		o_prop.pos.y = p_y;
		o_prop.pos.z = p_z;

		o_prop.la.x = la_x;
		o_prop.la.y = la_y;
		o_prop.la.z = la_z;

		o_prop.up.x = up_x;
		o_prop.up.y = up_y;
		o_prop.up.z = up_z;

		double* temp_n = normalize_Vector(la_x - p_x, la_y - p_y, la_z - p_z);	
		o_prop.n.x = o_prop.n_mod.x = temp_n[0];
		o_prop.n.y = o_prop.n_mod.y = temp_n[1];
		o_prop.n.z = o_prop.n_mod.z = temp_n[2];
		double* temp_up = normalize_Vector(up_x, up_y, up_z);
		cross_Vector(temp_up, temp_n);
		cross_Vector(temp_n, temp_up);
		free(temp_up); temp_up = NULL;
		double* temp_v = _normalize_Vector(temp_n);
		free(temp_n); temp_n = NULL;
		o_prop.v.x = temp_v[0];
		o_prop.v.y = temp_v[1];
		o_prop.v.z = temp_v[2];
		o_prop.v_mod.x = o_prop.v.x * f_prop.pix.h;
		o_prop.v_mod.y = o_prop.v.y * f_prop.pix.h;
		o_prop.v_mod.z = o_prop.v.z * f_prop.pix.h;
		double* temp_u = normalize_Vector(la_x - p_x, la_y - p_y, la_z - p_z);
		cross_Vector(temp_v, temp_u);
		o_prop.u.x = temp_v[0];
		o_prop.u.y = temp_v[1];
		o_prop.u.z = temp_v[2];
		o_prop.u_mod.x = o_prop.u.x * f_prop.pix.w;
		o_prop.u_mod.y = o_prop.u.y * f_prop.pix.w;
		o_prop.u_mod.z = o_prop.u.z * f_prop.pix.w;
		free(temp_u); temp_u = NULL;
		free(temp_v); temp_v = NULL;

		double adjust_y = (double)(f_prop.res.h >> 1); double adjust_x = (double)(f_prop.res.w >> 1);
		if (!(f_prop.res.h & (u64)1)) { adjust_y -= .5; } //move vpd down half pixel if even
		if (!(f_prop.res.w & (u64)1)) { adjust_x -= .5; } //move vpd left half pixel if even
	
		o_prop.n_mod.x = (o_prop.n.x * fclen) - (o_prop.v_mod.x * adjust_y) - (o_prop.u_mod.x * adjust_x);
		o_prop.n_mod.y = (o_prop.n.y * fclen) - (o_prop.v_mod.y * adjust_y) - (o_prop.u_mod.y * adjust_x);
		o_prop.n_mod.z = (o_prop.n.z * fclen) - (o_prop.v_mod.z * adjust_y) - (o_prop.u_mod.z * adjust_x);

		r_prop.draw_distance = 400;
		r_prop.sample_rate = 0;
		cudaMalloc((void**)&h_mem.rad.d_r, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.rad.d_g, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.rad.d_b, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.d_color.c, sizeof(u32) * f_prop.res.count);
		h_mem.h_color.c = (u32*)malloc(sizeof(u32) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.d_dist.d, sizeof(double) * f_prop.res.count);	

		cudaMalloc((void**)&h_mem.norm.d_x, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.norm.d_y, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.norm.d_z, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.pnt.d_x, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.pnt.d_y, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.pnt.d_z, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.rmd.d_x, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.rmd.d_y, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.rmd.d_z, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.inv_rmd.d_x, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.inv_rmd.d_y, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.inv_rmd.d_z, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.sign_rmd.d_x, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.sign_rmd.d_y, sizeof(double) * f_prop.res.count);
		cudaMalloc((void**)&h_mem.sign_rmd.d_z, sizeof(double) * f_prop.res.count);

		cudaMalloc((void**)&h_mem.d_rmi.index, sizeof(s64) * f_prop.res.count);
		h_mem.h_rmi.index = (s64*)malloc(sizeof(s64) * f_prop.res.count);


		//This works but i think maybe better way to do this
		cudaMalloc((void**)&d_mem, sizeof(pixel_memory));
		cudaMemcpy(d_mem, &h_mem, sizeof(pixel_memory), cudaMemcpyHostToDevice);


		init_camera_device_memory(this);
	};

	cudaError_t init_camera_trixel_data(Trixel* t, s64 num_trixels) {
		cudaMalloc((void**)&h_trixels.d_q.x, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_trixels.d_q.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_trixels.d_q.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&(h_trixels.d_t.x), sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_trixels.d_t.y, sizeof(double) * num_trixels);
		cudaMalloc((void**)&h_trixels.d_t.z, sizeof(double) * num_trixels);

		cudaMalloc((void**)&h_trixels.d_w, sizeof(double) * num_trixels);
		trixels_list = t;

		
		cudaMalloc((void**)&(d_trixels), sizeof(trixel_memory));
		cudaMemcpy(d_trixels, &(h_trixels), sizeof(trixel_memory), cudaMemcpyHostToDevice);
		return init_camera_trixel_device_memory(trixels_list, this);
	}
	cudaError_t init_camera_voxel_data(Trixel* t, s64 num_voxels) {
		cudaMalloc((void**)&h_voxels.s1, sizeof(double) * num_voxels);
		cudaMalloc((void**)&h_voxels.s2, sizeof(double) * num_voxels);

		cudaMalloc((void**)&h_voxels.children, sizeof(Camera::voxel_memory::childs) * num_voxels);
		cudaMalloc((void**)&h_voxels.is_leaf, sizeof(u8) * num_voxels);

		cudaMalloc((void**)&h_voxels.cut_flags, sizeof(Camera::voxel_memory::flag_tag) * num_voxels);

		cudaMalloc((void**)&h_voxels.d_Bo, sizeof(voxel_memory::voxel_vector) * num_voxels);
		cudaMalloc((void**)&(d_voxels), sizeof(voxel_memory));
		
		cudaMalloc((void**)&h_voxels.d_voxel_index_queue, sizeof(s32) * f_prop.res.count * (int)ceil(sqrt((double)num_voxels)));
		h_voxels.index_queue_offset = (u32)ceil(sqrt((double)num_voxels));
		h_voxels.num_voxels = num_voxels;
		cudaMemcpy(d_voxels, &(h_voxels), sizeof(voxel_memory), cudaMemcpyHostToDevice);
		return init_camera_voxel_device_memory(trixels_list, this);
		
	}

	cudaError_t update_camera_trixel_data() {
		//cam trixel data is simple a function of cam orientaion and triangel data. so update and init are same
		//I think because the cross product needs re done and is not a simple addition to translate.
		//Also note this funtion is in trixel.cu not camera.cu for...reasons...maybe will move later
		return init_camera_trixel_device_memory(trixels_list, this);
	}
	cudaError_t update_camera_voxel_data(Input::translate_vector tv) {
		return update_camera_voxel_device_memory(tv, this);
	}
	cudaError_t translate(Input::translate_vector tv) {
		o_prop.pos.x += tv.dx; o_prop.pos.y += tv.dy;	o_prop.pos.z += tv.dz;
		update_camera_voxel_data(tv);
		return update_camera_trixel_data();
	};

	cudaError_t color_pixels() { return color_camera_device(this); }

};
#endif