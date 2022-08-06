#ifndef CAMERA_CPP_TAG
#define CAMERA_CPP_TAG
#include "framework.h"
#include <stdio.h>
Camera::Camera(s32 r_w, s32 r_h,
	T_fp f_w, T_fp f_h, T_fp fclen,
	T_fp p_x, T_fp p_y, T_fp p_z,
	T_fp la_x, T_fp la_y, T_fp la_z,
	T_fp up_x, T_fp up_y, T_fp up_z
) {
	f_prop.res.w = r_w;
	f_prop.res.h = r_h;
	f_prop.res.count = static_cast<u64>(r_w) * static_cast<u64>(r_h);
	f_prop.film.w = f_w;
	f_prop.film.w = f_h;
	f_prop.pix.w = f_w / (T_fp)r_w;
	f_prop.pix.h = f_h / (T_fp)r_h;
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

	T_fp* temp_n = normalize_Vector<T_fp>(la_x - p_x, la_y - p_y, la_z - p_z);
	o_prop.n.x = o_prop.n_mod.x = temp_n[0];
	o_prop.n.y = o_prop.n_mod.y = temp_n[1];
	o_prop.n.z = o_prop.n_mod.z = temp_n[2];
	T_fp* temp_up = normalize_Vector<T_fp>(up_x, up_y, up_z);
	cross_Vector(temp_up, temp_n);
	cross_Vector(temp_n, temp_up);
	free(temp_up); temp_up = NULL;
	T_fp* temp_v = _normalize_Vector(temp_n);
	free(temp_n); temp_n = NULL;
	o_prop.v.x = temp_v[0];
	o_prop.v.y = temp_v[1];
	o_prop.v.z = temp_v[2];
	o_prop.v_mod.x = o_prop.v.x * f_prop.pix.h;
	o_prop.v_mod.y = o_prop.v.y * f_prop.pix.h;
	o_prop.v_mod.z = o_prop.v.z * f_prop.pix.h;
	T_fp* temp_u = normalize_Vector<T_fp>(la_x - p_x, la_y - p_y, la_z - p_z);
	cross_Vector(temp_v, temp_u);
	o_prop.u.x = temp_v[0];
	o_prop.u.y = temp_v[1];
	o_prop.u.z = temp_v[2];
	o_prop.u_mod.x = o_prop.u.x * f_prop.pix.w;
	o_prop.u_mod.y = o_prop.u.y * f_prop.pix.w;
	o_prop.u_mod.z = o_prop.u.z * f_prop.pix.w;
	free(temp_u); temp_u = NULL;
	free(temp_v); temp_v = NULL;

	T_fp adjust_y = (T_fp)(f_prop.res.h >> 1); T_fp adjust_x = (T_fp)(f_prop.res.w >> 1);
	if (!(f_prop.res.h & (u64)1)) { adjust_y -= .5; } //move vpd down half pixel if even
	if (!(f_prop.res.w & (u64)1)) { adjust_x -= .5; } //move vpd left half pixel if even

	o_prop.n_mod.x = (o_prop.n.x * fclen) - (o_prop.v_mod.x * adjust_y) - (o_prop.u_mod.x * adjust_x);
	o_prop.n_mod.y = (o_prop.n.y * fclen) - (o_prop.v_mod.y * adjust_y) - (o_prop.u_mod.y * adjust_x);
	o_prop.n_mod.z = (o_prop.n.z * fclen) - (o_prop.v_mod.z * adjust_y) - (o_prop.u_mod.z * adjust_x);

	o_prop.rotation_help_array = (VEC3<T_fp>**)malloc(sizeof(VEC3<T_fp>*) * 3);
	r_prop.draw_distance = 400;
	r_prop.sample_rate = 0;
	cudaMalloc((void**)&h_mem.rad.r, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.rad.g, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.rad.b, sizeof(T_fp) * f_prop.res.count);


	cudaMalloc((void**)&h_mem.d_color.c, sizeof(u32) * f_prop.res.count);
	h_mem.h_color.c = (u32*)malloc(sizeof(u32) * f_prop.res.count);
	

	cudaMalloc((void**)&h_mem.d_color.rad, sizeof(Color::radiance) * f_prop.res.count);
	h_mem.h_color.rad = (Color::radiance*)malloc(sizeof(Color::radiance) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.d_dist.d, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.norm.x, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.norm.y, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.norm.z, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.pnt.x, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.pnt.y, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.pnt.z, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.rmd.x, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.rmd.y, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.rmd.z, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.inv_rmd.x, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.inv_rmd.y, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.inv_rmd.z, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.sign_rmd.x, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.sign_rmd.y, sizeof(T_fp) * f_prop.res.count);
	cudaMalloc((void**)&h_mem.sign_rmd.z, sizeof(T_fp) * f_prop.res.count);

	cudaMalloc((void**)&h_mem.d_rmi.index, sizeof(s64) * f_prop.res.count);
	h_mem.h_rmi.index = (s64*)malloc(sizeof(s64) * f_prop.res.count);
	//**TODO** make quaterion flags readable constants
	o_prop.quat = Quaternion(3);
	//This works but i think maybe better way to do this
	cudaMalloc((void**)&d_mem, sizeof(pixel_memory));
	cudaMemcpy(d_mem, &h_mem, sizeof(pixel_memory), cudaMemcpyHostToDevice);
	d_voxels = NULL;

	init_camera_device_memory(this);
};
cudaError_t Camera::add_object(Object* new_object) {
	cudaError_t c_err = cudaPeekAtLastError();
	if (num_objects++ == 0) {
		object_list = (Object**)malloc(sizeof(Object*));
		if (!object_list) { return c_err; }
		object_list[0] = new_object;
	}else {
		void* temp = (void*)object_list;
		object_list = (Object**)realloc(object_list, sizeof(Object*) * num_objects);
		if (!object_list) { free(temp); return c_err; }
		object_list[num_objects - 1] = new_object;
	}

	new_object->quat = new Quaternion(o_prop.quat);

	switch (new_object->getTag()) {
	case TRIXEL_OBJECT_TAG:
		init_camera_trixel_data(new_object->trixel_list, new_object->trixel_list->num_trixels);
		c_err = init_camera_voxel_data(new_object->trixel_list, new_object->trixel_list->num_voxels);	}

	return c_err;
}
cudaError_t Camera::init_camera_trixel_data(Trixel* t, s64 num_trixels) {
	
	cudaMalloc((void**)&h_trixels.d_q.x, sizeof(T_fp) * num_trixels);
	cudaMalloc((void**)&h_trixels.d_q.y, sizeof(T_fp) * num_trixels);
	cudaMalloc((void**)&h_trixels.d_q.z, sizeof(T_fp) * num_trixels);

	cudaMalloc((void**)&(h_trixels.d_t.x), sizeof(T_fp) * num_trixels);
	cudaMalloc((void**)&h_trixels.d_t.y, sizeof(T_fp) * num_trixels);
	cudaMalloc((void**)&h_trixels.d_t.z, sizeof(T_fp) * num_trixels);

	cudaMalloc((void**)&h_trixels.d_w, sizeof(T_fp) * num_trixels);


	cudaMalloc((void**)&(d_trixels), sizeof(trixel_memory));
	cudaMemcpy(d_trixels, &(h_trixels), sizeof(trixel_memory), cudaMemcpyHostToDevice);
	return init_camera_trixel_device_memory(t, this);
}

cudaError_t Camera::init_camera_voxel_data(Trixel* t, s64 num_voxels) {
	cudaError_t cuda_err;
	h_voxels.obj_center.w = (T_fp)1.0;
	h_voxels.obj_center.x = (T_fp)0.0;
	h_voxels.obj_center.y = (T_fp)0.0;
	h_voxels.obj_center.z = (T_fp)0.0;
	T_fp* temp_n = normalize_Vector<T_fp>(o_prop.pos.x - h_voxels.obj_center.x, o_prop.pos.y - h_voxels.obj_center.y, o_prop.pos.z - h_voxels.obj_center.z);
	h_voxels.init_face = -VEC4<T_fp>(temp_n);
	memcpy(&h_voxels.n, &h_voxels.init_face, sizeof(VEC4<T_fp>));
	memcpy(&h_voxels.cur_transform, &h_voxels.init_face, sizeof(VEC4<T_fp>));

	T_fp* temp_up = normalize_Vector<T_fp>((T_fp)0.0, (T_fp)1.0, (T_fp)0.0);
	
	cross_Vector(temp_up, temp_n);
	cross_Vector(temp_n, temp_up);
	free(temp_up); temp_up = NULL;
	T_fp* temp_v = _normalize_Vector(temp_n);
	free(temp_n); temp_n = NULL;
	h_voxels.v = VEC4<T_fp>(temp_v);
	T_fp* temp_u = normalize_Vector<T_fp>(o_prop.pos.x - h_voxels.obj_center.x, o_prop.pos.y - h_voxels.obj_center.y, o_prop.pos.z - h_voxels.obj_center.z);
	cross_Vector(temp_v, temp_u);
	h_voxels.u = VEC4<T_fp>(temp_v);
	free(temp_u); temp_u = NULL;	free(temp_v); temp_v = NULL;


	cudaMalloc((void**)&h_voxels.s1, sizeof(T_fp) * num_voxels);
	cudaMalloc((void**)&h_voxels.s2, sizeof(T_fp) * num_voxels);

	cudaMalloc((void**)&h_voxels.children, sizeof(voxel_memory::childs) * num_voxels);
	cudaMalloc((void**)&h_voxels.is_leaf, sizeof(u8) * num_voxels);

	cudaMalloc((void**)&h_voxels.cut_flags, sizeof(voxel_memory::flag_tag) * num_voxels);



	cudaMalloc((void**)&h_voxels.d_Bo, sizeof(voxel_memory::voxel_vector) * num_voxels);

	cudaMalloc((void**)&h_voxels.d_voxel_index_queue, sizeof(s32) * f_prop.res.count * (s32)ceil(log2((double)num_voxels)));
	h_voxels.index_queue_offset = (s32)ceil(log2((double)num_voxels));
	h_voxels.num_voxels = num_voxels;

	cudaMalloc((void**)&(d_voxels), sizeof(voxel_memory));
	cudaMemcpy(d_voxels, &(h_voxels), sizeof(voxel_memory), cudaMemcpyHostToDevice);
	cuda_err = init_camera_voxel_device_memory(t, this);
	return cuda_err;
}
//**TODO** right now this just does ALL primitives, change to select primitives
//ALSO REMBER IF YOU WANT TO DO THIS FOR PRIMITVIES MAKE SURE TO CALL TRANSFORM ON ALL CAMERAS, or else you will move the camera adn not the object
cudaError_t Camera::transform(Input* input, volatile u8 transform_select) {
	VEC4<T_fp>* tv = input->t_vec;
	Quaternion* q = input->t_quat;
	cudaError_t cuda_err = cudaPeekAtLastError();
	switch (transform_select) {
	case TRANSLATE_XYZ:	
	case TRANSLATE_Z:
	case TRANSLATE_X:
	case ROTATE_TRI_PY:
	case ROTATE_TRI_NY:
		//**TODO** change from firs to arbitrary
		cuda_err = transform_camera_voxel_device_memory(this, object_list[0]->trixel_list, tv, q, transform_select);
		break;
	};
	return cuda_err;
};
cudaError_t Camera::color_pixels() { return color_camera_device(this); }

#endif