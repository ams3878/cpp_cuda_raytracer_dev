#pragma once
#ifndef PLATFORM_COMMON_H
#define PLATFORM_COMMON_H
#include <windows.h>
#ifndef GLOBALINPUT
#define GLOBALINPUT g_input
#endif
#ifndef EPSILON
#define EPSILON 0.0001f
#endif
typedef char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;

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
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_W,
    BUTTON_S,
    BUTTON_A,
    BUTTON_D,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_ENTER,
    BUTTON_ESCAPE,
    BUTTON_LEFTMOUSE,
    BUTTON_RIGHTMOUSE,
    BUTTON_F1,
    BUTTON_COUNT, // Should be the last item
  };

  class Input {
  public:
    Button buttons[BUTTON_COUNT];
    //Render_State* selected_hud_obj;
    //Render_Object_Container* selected_world_obj;
    struct translate_vector { double dx; double dy; double dz; }translate;
    int selected_triangle = -1;
    Input() { translate.dx = 0;  translate.dy = 0;  translate.dz = 0;
    }

  };
#endif
#ifndef BUTTON_PROCESS_MACROS
#define BUTTON_PROCESS_MACROS
#define process_button(b, vk)\
case vk: {\
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