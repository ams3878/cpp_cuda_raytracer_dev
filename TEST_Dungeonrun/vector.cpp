#include "Vector.h"

int vector_log2(u64 value) {
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    value |= value >> 32;
    return tab64[((u64)((value - (value >> 1)) * 0x07EDD5E59A4E28C2)) >> 58];
}

float vector_norm(float scalor){
    float invrs_sqrt = scalor;
    float invrs_sqrt_half = 0.5f * invrs_sqrt;
    union { float x; s64 i; } u;
    u.x = invrs_sqrt_half;
    // u.i = typeid(T) == typeid(float) ? 0x5f375a86 - (u.i >> 1) : 0x5FE6EB50C7B537A9 - (u.i >> 1);
    u.i = 0x5f375a86 - (u.i >> 1);
    /* The next line can be repeated any number of times to increase accuracy */
    for (int i = 0; i < 8; i++) {
        u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
    }
    //invrs_sqrt = 
    return u.x;
}
void VEC4<T_fp>::rotate(VEC4<VEC4<T_fp>*>*cur_rot, VEC4<T_fp>*cur_vec, VEC4<T_fp>*new_vec, int reverse) {
    //quaternion Multiply if we are doing new rotation
    if (new_vec) {
        T_fp t_i = cur_vec->i, t_j = cur_vec->j, t_k = cur_vec->k, t_w = cur_vec->w;
        cur_vec->i = t_j * new_vec->z - t_k * new_vec->y + t_i * new_vec->w + t_w * new_vec->x;
        cur_vec->j = t_k * new_vec->x - t_i * new_vec->z + t_j * new_vec->w + t_w * new_vec->y;
        cur_vec->k = t_i * new_vec->y - t_j * new_vec->x + t_k * new_vec->w + t_w * new_vec->z;
        cur_vec->w = t_w * new_vec->w - t_i * new_vec->x - t_j * new_vec->y - t_k * new_vec->z;
    
        //convert quaternion into rot matrix and store it
        cur_rot->x->i = (1 - 2 * cur_vec->j * cur_vec->j - 2 * cur_vec->k * cur_vec->k);
        cur_rot->x->j = (2 * cur_vec->i * cur_vec->j - 2 * cur_vec->k * cur_vec->w);
        cur_rot->x->k = (2 * cur_vec->i * cur_vec->k + 2 * cur_vec->j * cur_vec->w);

        cur_rot->y->i = (2 * cur_vec->i * cur_vec->j + 2 * cur_vec->k * cur_vec->w);
        cur_rot->y->j = (1 - 2 * cur_vec->i * cur_vec->i - 2 * cur_vec->k * cur_vec->k);
        cur_rot->y->k = (2 * cur_vec->j * cur_vec->k - 2 * cur_vec->i * cur_vec->w);

        cur_rot->z->i = (2 * cur_vec->i * cur_vec->k - 2 * cur_vec->j * cur_vec->w);
        cur_rot->z->j = (2 * cur_vec->j * cur_vec->k + 2 * cur_vec->i * cur_vec->w);
        cur_rot->z->k = (1 - 2 * cur_vec->i * cur_vec->i - 2 * cur_vec->j * cur_vec->j);
    }
    T_fp temp_x = x * reverse, temp_y = y * reverse, temp_z = z * reverse;
    //do the roation of the vector
    x = (temp_x * cur_rot->x->i + temp_y * cur_rot->x->j + temp_z * cur_rot->x->k);
    y = (temp_x * cur_rot->y->i + temp_y * cur_rot->y->j + temp_z * cur_rot->y->k);
    z = (temp_x * cur_rot->z->i + temp_y * cur_rot->z->j + temp_z * cur_rot->z->k);
}
