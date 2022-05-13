#pragma once
#include "platform_common.h"
#include "Vector.h"
#ifndef CUDA_KERNEL
#define CUDA_KERNEL
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#endif
struct Camera;
struct Trixel;
cudaError_t init_camera_device_memory(Camera* c);
cudaError_t init_camera_device_memory(Camera* c, Trixel* t);

class Camera
{
public:
	//Camera Properties
	struct film_properties{// film/screen
		struct resolution { u32 w; u32 h; u64 count;}res; //number of pixels
		struct pixel { double w; double h; }pix; //size of pixels
		struct film { double w; double h; }film; //
	}f_prop;
	struct lens_properties {
		double focal_length;
	}l_prop;
	struct orientation_properties {
		struct position { double x; double y; double z; }pos; //world cords
		struct look_at { double x; double y; double z; }la;//position
		struct up { double x; double y; double z; }up;//position
		// n = norm(look_at - position)     n_mod = direction at bottom left pixel
		// v = norm(n cross ( up cross n))  v_mod = v / pixel height
		// u = n cross v                    u_mod = u / pixel width
		struct o_vector { double x; double y; double z;}n, v, u, n_mod, v_mod, u_mod;
	}o_prop;
	struct render_properites {
		double draw_distance;
		int sample_rate;
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
		struct ray_direction { double* d_x; double* d_y; double* d_z; }rmd;
		struct intersection_object_index { s64* index; }d_rmi, h_rmi;//index into list of triangles
	}h_mem; void* d_mem;

	//Recalcuate on Camera position move
	struct trixel_memory {
		struct trixel_vector { double* x; double* y; double* z; }d_t, d_q;
		double* d_w = NULL;
	}h_trixels; void* d_trixels;


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
		cross_Vector(temp_u, temp_v);
		o_prop.u.x = temp_u[0];
		o_prop.u.y = temp_u[1];
		o_prop.u.z = temp_u[2];
		o_prop.u_mod.x = o_prop.u.x * f_prop.pix.w;
		o_prop.u_mod.y = o_prop.u.y * f_prop.pix.w;
		o_prop.u_mod.z = o_prop.u.z * f_prop.pix.w;
		free(temp_u); temp_u = NULL;
		free(temp_v); temp_v = NULL;

		double adjust_y = (double)(f_prop.res.h >> 1); double adjust_x = (double)(f_prop.res.w >> 1);
		if (!(f_prop.res.h & (u64)1)) { adjust_y -= .5; } //move vpd down half pixel if even
		if (!(f_prop.res.w & (u64)1)) { adjust_x -= .5; } //move vpd left half pixel if even
	
		o_prop.n_mod.x = o_prop.n.x - (o_prop.v_mod.x * adjust_y) + (o_prop.u_mod.x * adjust_x);
		o_prop.n_mod.y = o_prop.n.y - (o_prop.v_mod.y * adjust_y) + (o_prop.u_mod.y * adjust_x);
		o_prop.n_mod.z = o_prop.n.z - (o_prop.v_mod.z * adjust_y) + (o_prop.u_mod.z * adjust_x);

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

		cudaMalloc((void**)&h_mem.d_rmi.index, sizeof(s64) * f_prop.res.count);
		h_mem.h_rmi.index = (s64*)malloc(sizeof(s64) * f_prop.res.count);

		//This works but i think maybe better way to do this
		cudaMalloc((void**)&d_mem, sizeof(pixel_memory));
		cudaMemcpy(d_mem, &h_mem, sizeof(pixel_memory), cudaMemcpyHostToDevice);


		init_camera_device_memory(this);
	};
};


