#pragma once
#ifndef WINDOW_FUNCTIONS_H
  #define WINDOW_FUNCTIONS_H
#include <windows.h>
#include <winuser.h>
#include <tchar.h>
#include "platform_common.h"
class Input;
#define MAIN_WIN_CLASS 0

  u32 initialize_hwnd_class(WNDCLASS* class_list, WNDPROC w_c);
  u32 initialize_hwnd(HWND* hwn, u8 wclass);
  u32 peek_message_wrapper(HWND* wind, Input** input_ptr);
#endif