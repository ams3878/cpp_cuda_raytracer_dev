#pragma once
#include "platform_common.h"
class Input {
public:
    Button buttons[BUTTON_COUNT];

    VEC4<T_fp>* t_vec;
    Quaternion* t_quat;

    int selected_triangle = -1;
    Input();
    void set_vec(T_fp _x, T_fp _y, T_fp _z, T_fp _w);
    void set_quat();
    void set_quat(T_fp _x, T_fp _y, T_fp _z, T_fp _w);
};