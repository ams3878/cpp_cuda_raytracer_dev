#include "framework.h"

T_fp Quaternion::_i(T_uint index) const { return vec->i;}
T_fp Quaternion::_j(T_uint index) const { return vec->j;}
T_fp Quaternion::_k(T_uint index) const { return vec->k;}
T_fp Quaternion::_w(T_uint index) const { return vec->w;}


Quaternion::Quaternion(){
	vec = new VEC4<T_fp>(0,0,0,1);
	rot_m = new VEC4<VEC4<T_fp>*>();
	rot_m->x = new VEC4<T_fp>(1, 0, 0, 0);
	rot_m->y = new VEC4<T_fp>(0, 1, 0, 0);
	rot_m->z = new VEC4<T_fp>(0, 0, 1, 0);
}
Quaternion::Quaternion(int c_flag) {
	if (c_flag & 2) {
		vec = new VEC4<T_fp>(0, 0, 0, 1);
		rot_m = new VEC4<VEC4<T_fp>*>();
		rot_m->x = new VEC4<T_fp>(1, 0, 0, 0);
		rot_m->y = new VEC4<T_fp>(0, 1, 0, 0);
		rot_m->z = new VEC4<T_fp>(0, 0, 1, 0);
	}else{
		vec = NULL;
		d_vec = NULL;
		rot_m = NULL;
		isCUDA = false;
	}
	if (c_flag & 1) { 
		isCUDA = true;
		initialize_CUDA(&VEC4<T_fp>(1,0,0,0), &VEC4<T_fp>(0, 1, 0, 0), &VEC4<T_fp>(0, 0, 1, 0));
	}
}
Quaternion::Quaternion(const Quaternion& q) {

	vec = new VEC4<T_fp>(*q.vec);
	rot_m = new VEC4<VEC4<T_fp>*>();
	rot_m->x = new VEC4<T_fp>(*q.rot_m->x);
	rot_m->y = new VEC4<T_fp>(*q.rot_m->y);
	rot_m->z = new VEC4<T_fp>(*q.rot_m->z);
	//rot_m->w = &VEC4_CUDA<T_fp>(*q.rot_m->w);
	initialize_CUDA(rot_m->x, rot_m->y, rot_m->z);
}

int Quaternion::set_transformation_matrix_tran(VEC4<T_fp> delta) {
	rot_m->x->w += delta.x;
	rot_m->y->w += delta.y;
	rot_m->z->w += delta.z;	
	return 0;
};
int Quaternion::set_transformation_matrix_rot() {
	for (T_uint i = 0; i < 1; i++) {

		rot_m->x->i = 1 - 2 * _j(i) * _j(i) - 2 * _k(i) * _k(i);
		rot_m->x->j = 2 * _i(i) * _j(i) - 2 * _k(i) * _w(i);
		rot_m->x->k = 2 * _i(i) * _k(i) + 2 * _j(i) * _w(i);

		rot_m->y->i = 2 * _i(i) * _j(i) + 2 * _k(i) * _w(i);
		rot_m->y->k = 1 - 2 * _i(i) * _i(i) - 2 * _k(i) * _k(i);
		rot_m->y->j = 2 * _j(i) * _k(i) - 2 * _i(i) * _w(i);

		rot_m->z->i = 2 * _i(i) * _k(i) - 2 * _j(i) * _w(i);
		rot_m->z->j = 2 * _j(i) * _k(i) + 2 * _i(i) * _w(i);
		rot_m->z->k = 1 - 2 * _i(i) * _i(i) - 2 * _j(i) * _j(i);

	}
	return 0;}



