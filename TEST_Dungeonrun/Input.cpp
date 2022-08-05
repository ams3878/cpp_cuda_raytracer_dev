#include "framework.h"
Input::Input() {
    t_vec = new VEC4<T_fp>();
    t_quat = new Quaternion();
}
void Input::set_vec(T_fp _x, T_fp _y, T_fp _z, T_fp _w) {
    t_vec->x = _x; 
    t_vec->y = _y;
    t_vec->z = _z;
    t_vec->w = _w;
}
void Input::set_quat() {
     t_quat->vec = t_vec;
     t_quat->set_transformation_matrix_rot();
}
void Input::set_quat(T_fp _x, T_fp _y, T_fp _z, T_fp _w) {
    set_vec(_x, _y, _z, _w);
    set_quat();
}