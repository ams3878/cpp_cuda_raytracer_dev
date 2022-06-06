#include "framework.h"
Quaternion::Quaternion() {
	h_vec.i = (T_fp*)malloc(sizeof(T_fp));
	h_vec.j = (T_fp*)malloc(sizeof(T_fp));
	h_vec.k = (T_fp*)malloc(sizeof(T_fp));
	h_vec.w = (T_fp*)malloc(sizeof(T_fp));
	h_rot_m = (T_fp*)malloc(sizeof(T_fp) * 9);
	d_rot_m = NULL;
	size = 1;
}
Quaternion::Quaternion(u32 s) {
	h_vec.i = (T_fp*)malloc(sizeof(T_fp) * s);
	h_vec.j = (T_fp*)malloc(sizeof(T_fp) * s);
	h_vec.k = (T_fp*)malloc(sizeof(T_fp) * s);
	h_vec.w = (T_fp*)malloc(sizeof(T_fp) * s);
	h_rot_m = (T_fp*)malloc(sizeof(T_fp) * 9 * s);
	d_rot_m = NULL;
	size = s;
}
Quaternion::Quaternion(u32 s, int c_flag) : Quaternion(s){
	if (c_flag & 1) { initialize_CUDA(); }
	//if (c_flag & 2) { _memset(c_flag >> 16); }
}
int Quaternion::set_rot_matrix(int s_flag) {
	if (s_flag & 1) { set_rot_matrix(); }
	if (s_flag & 2) { set_rot_matrix_CUDA(); }
	return 0;
}

int Quaternion::set_rot_matrix() {
	for (int i = 0, ii = 0; i < size; i++, ii+=9) {		
		h_rot_m[ii] = 1 - 2 * h_vec.j[i] * h_vec.j[i] - 2 * h_vec.k[i] * h_vec.k[i];
		h_rot_m[ii + 1] = 2 * h_vec.i[i] * h_vec.j[i] - 2 * h_vec.k[i] * h_vec.w[i];
		h_rot_m[ii + 2] = 2 * h_vec.i[i] * h_vec.k[i] + 2 * h_vec.j[i] * h_vec.w[i];
		h_rot_m[ii + 3] = 2 * h_vec.i[i] * h_vec.j[i] + 2 * h_vec.k[i] * h_vec.w[i];
		h_rot_m[ii + 4] = 1 - 2 * h_vec.i[i] * h_vec.i[i] - 2 * h_vec.k[i] * h_vec.k[i];
		h_rot_m[ii + 5] = 2 * h_vec.j[i] * h_vec.k[i] - 2 * h_vec.i[i] * h_vec.w[i];
		h_rot_m[ii + 6] = 2 * h_vec.i[i] * h_vec.k[i] - 2 * h_vec.j[i] * h_vec.w[i];
		h_rot_m[ii + 7] = 2 * h_vec.j[i] * h_vec.k[i] + 2 * h_vec.i[i] * h_vec.w[i];
		h_rot_m[ii + 8] = 1 - 2 * h_vec.i[i] * h_vec.i[i] - 2 * h_vec.j[i] * h_vec.j[i];
	}
	return 0;
};

   int Quaternion::_memset(Quaternion::quaternion_vec v) {
//	memcpy(h_vec, v, sizeof(quaternion_vec) * size);
	memcpy(h_vec.i, v.i, sizeof(T_fp) * size);
	memcpy(h_vec.j, v.j, sizeof(T_fp) * size);
	memcpy(h_vec.k, v.k, sizeof(T_fp) * size);
	memcpy(h_vec.w, v.w, sizeof(T_fp) * size);
	return 0;

}
int Quaternion::_memset(T_fp* _w, T_fp* _i, T_fp* _j, T_fp* _k) {
	memcpy(h_vec.i, _i, sizeof(T_fp) * size);
	memcpy(h_vec.j, _j, sizeof(T_fp) * size);
	memcpy(h_vec.k, _k, sizeof(T_fp) * size);
	memcpy(h_vec.w, _w, sizeof(T_fp) * size);
	return 0;

}
int Quaternion::_memset(int val) {
	memset(h_vec.i, val, sizeof(T_fp) * size);
	memset(h_vec.j, val, sizeof(T_fp) * size);
	memset(h_vec.k, val, sizeof(T_fp) * size);
	memset(h_vec.w, val, sizeof(T_fp) * size);
	return 0;

}

int Quaternion::_free() {
	free(h_vec.i); free(h_vec.j);  free(h_vec.k);  free(h_vec.w); 
	cudaFree(d_vec.i); cudaFree(d_vec.j);  cudaFree(d_vec.k);  cudaFree(d_vec.w);
	return 0;
}

