#include "framework.h"

T_fp Quaternion::_i(T_uint index) const { return vec->complex.i[index];}
T_fp Quaternion::_j(T_uint index) const { return vec->complex.j[index];}
T_fp Quaternion::_k(T_uint index) const { return vec->complex.k[index];}
T_fp Quaternion::_w(T_uint index) const { return vec->w[index];}

Quaternion::Quaternion() {
	vec = NULL;
	d_vec = NULL;
	rot_m = NULL;
	size = 0;
	isCUDA = false;
}
Quaternion::Quaternion(u32 s) : Quaternion() {
	if (s > 0) {
		vec = new VEC4_CUDA<T_fp>(s);
		rot_m = new VEC4_CUDA<VEC4_CUDA<T_fp>>(1);
		rot_m->x = new VEC4_CUDA<T_fp>(s);
		rot_m->y = new VEC4_CUDA<T_fp>(s);
		rot_m->z = new VEC4_CUDA<T_fp>(s);
		rot_m->w = new VEC4_CUDA<T_fp>(s);
		size = s;
	}
}
Quaternion::Quaternion(u32 s, int c_flag) : Quaternion((c_flag & 1)?0:s) {
	if (c_flag & 1) { 
		size = s;
		isCUDA = true;
		initialize_CUDA();
	}
}
int Quaternion::set_transformation_matrix_tran(VEC4<T_fp> delta) {
	for (T_uint i = 0; i < size; i++) {
		rot_m->x->w[i] += delta.x;
		rot_m->y->w[i] += delta.y;
		rot_m->z->w[i] += delta.z;
	}
	return 0;
};
int Quaternion::set_transformation_matrix_rot() {
	for (T_uint i = 0; i < size; i++) {

		rot_m->x->i[i] = 1 - 2 * _j(i) * _j(i) - 2 * _k(i) * _k(i);
		rot_m->x->j[i] = 2 * _i(i) * _j(i) - 2 * _k(i) * _w(i);
		rot_m->x->k[i] = 2 * _i(i) * _k(i) + 2 * _j(i) * _w(i);

		rot_m->y->i[i] = 2 * _i(i) * _j(i) + 2 * _k(i) * _w(i);
		rot_m->y->k[i] = 1 - 2 * _i(i) * _i(i) - 2 * _k(i) * _k(i);
		rot_m->y->j[i] = 2 * _j(i) * _k(i) - 2 * _i(i) * _w(i);

		rot_m->z->i[i] = 2 * _i(i) * _k(i) - 2 * _j(i) * _w(i);
		rot_m->z->j[i] = 2 * _j(i) * _k(i) + 2 * _i(i) * _w(i);
		rot_m->z->k[i] = 1 - 2 * _i(i) * _i(i) - 2 * _j(i) * _j(i);
		
	}
	return 0;
};

int Quaternion::_memset(VEC4<T_fp>* v) {

	memcpy(vec->complex.i, &(v->i), sizeof(T_fp));
	memcpy(vec->complex.j, &(v->j), sizeof(T_fp));
	memcpy(vec->complex.k, &(v->k), sizeof(T_fp));
	memcpy(vec->w, &(v->w), sizeof(T_fp));
	return 0;
}

int Quaternion::_memset(T_fp* _w, T_fp* _i, T_fp* _j, T_fp* _k) {
	memcpy(vec->complex.i, _i, sizeof(T_fp) * size);
	memcpy(vec->complex.j, _j, sizeof(T_fp) * size);
	memcpy(vec->complex.k, _k, sizeof(T_fp) * size);
	memcpy(vec->w, _w, sizeof(T_fp) * size);
	return 0;

}
int Quaternion::_memset(int val) {
	memset(vec->complex.i, val, sizeof(T_fp) * size);
	memset(vec->complex.j, val, sizeof(T_fp) * size);
	memset(vec->complex.k, val, sizeof(T_fp) * size);
	memset(vec->w, val, sizeof(T_fp) * size);
	return 0;

}

int Quaternion::_free(int flag) {
	switch (flag) {
	case 0:
		free(vec->complex.i); free(vec->complex.j);  free(vec->complex.k);  free(vec->w);
		break;
	case 1:
		cudaFree(vec->complex.i); cudaFree(vec->complex.j);  cudaFree(vec->complex.k);  cudaFree(vec->w);
		break;
	}
	return 0;
}

