#pragma once
#ifndef PLATFORM_COMMON_H
#define PLATFORM_COMMON_H
#include  <windows.h>
#include "Vector.h"
#include "typedefs.h"
#ifndef GLOBALINPUT
#define GLOBALINPUT g_input
#endif
#ifndef EPSILON
#define EPSILON 0.0001f
#endif


//CUDA TRANSFORM DEFINES
constexpr auto TRANSLATE_XYZ = 30;
constexpr auto TRI_TAG_OFFSET = 10;
constexpr auto ROTATE_TRI_PY = 10;
constexpr auto ROTATE_TRI_NY = 11;


//The rotate tages are used for offset into rot matrix so they cannot be arbitray tags
constexpr auto ROTATE_CAM_PY = 0;
constexpr auto ROTATE_CAM_NY = 1;
constexpr auto ROTATE_CAM_PX = 2;
constexpr auto ROTATE_CAM_NX = 3;

class Button {
    HWND* wind = NULL;
public:
    bool is_down = false;;
    bool changed = false;
    s16 p_x = 0;
    s16 p_y = 0;//current point, updated every tick
    s16 p_select_x = 0;//point when selected, updated once per select
    s16 p_select_y = 0;
  };

enum {
    BUTTON_UP = VK_UP,
    BUTTON_DOWN = VK_DOWN,
    BUTTON_W = 'W',
    BUTTON_S = 'S',
    BUTTON_A = 'A',
    BUTTON_D = 'D',
    BUTTON_R = 'R',
    BUTTON_E = 'E',
    BUTTON_Q = 'Q',
    BUTTON_T = 'T',
    BUTTON_LEFT = VK_LEFT ,
    BUTTON_RIGHT = VK_RIGHT,
    BUTTON_ENTER = VK_RETURN,
    BUTTON_ESCAPE = VK_ESCAPE,
    BUTTON_F1 = VK_F1,
    BUTTON_LEFTMOUSE,
    BUTTON_RIGHTMOUSE,
    BUTTON_COUNT, // Should be the last item
  };
  #define NUM_MOUSE_BUTTONS 2
  //enum w_input_enum { BUTTON_UP, BUTTON_W  };
  class Input {
  public:
    Button buttons[BUTTON_COUNT];
    //Render_State* selected_hud_obj;
    //Render_Object_Container* selected_world_obj;
    VEC4<T_fp> *translate;
    int selected_triangle = -1;
    Input() {
        translate = new VEC4<T_fp>();
    }

  };
#endif
#ifndef BUTTON_PROCESS_MACROS
#define BUTTON_PROCESS_MACROS
#define process_button(b)\
case b: {\
  GLOBALINPUT->buttons[b].changed = is_down != GLOBALINPUT->buttons[b].is_down;\
  GLOBALINPUT->buttons[b].is_down = is_down;\
} break;
#define process_mouse_move(b, vk)\
case vk:{\
  GLOBALINPUT->buttons[b].p_x = x_pos;\
  GLOBALINPUT->buttons[b].p_y = y_pos;\
} break;
#define process_mouse_click(b, msg, d)\
case msg: {\
  GLOBALINPUT->buttons[b].changed = d != GLOBALINPUT->buttons[b].is_down;\
  GLOBALINPUT->buttons[b].is_down = d;\
  GLOBALINPUT->buttons[b].p_x = x_pos;\
  GLOBALINPUT->buttons[b].p_y = y_pos;\
} break;
#define is_button_release(b) !GLOBALINPUT->buttons[b].is_down && GLOBALINPUT->buttons[b].changed
#define is_click_select(b) GLOBALINPUT->buttons[b].is_down && GLOBALINPUT->buttons[b].changed
#define is_click_hold(b) GLOBALINPUT->buttons[b].is_down && !GLOBALINPUT->buttons[b].changed
#endif