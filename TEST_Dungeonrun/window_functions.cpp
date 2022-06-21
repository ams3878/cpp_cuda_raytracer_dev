#include "window_functions.h"
#include "framework.h"
u32 initialize_hwnd_class(WNDCLASS* wndclass,  WNDPROC w_c) {
  wndclass->lpszClassName = _T("Game Window Class");    
  wndclass->cbClsExtra = 0;
  wndclass->cbWndExtra = 0;
  wndclass->hInstance = NULL;
  wndclass->lpszMenuName = 0;
  wndclass->hIcon = 0;
  wndclass->hCursor = 0;
  wndclass->hbrBackground = 0;
  wndclass->style = CS_HREDRAW | CS_VREDRAW;
  wndclass->lpfnWndProc = w_c;
  RegisterClass(wndclass);  // Register Class
  return 0;
}

u32 initialize_hwnd(HWND* hwn, u8 wclass) {
  switch (wclass) {
  case MAIN_WIN_CLASS: {
    //Fullscreen
    SetWindowLong(*hwn, GWL_STYLE, GetWindowLong(*hwn, GWL_STYLE) & ~WS_OVERLAPPEDWINDOW);
    MONITORINFO mi = { sizeof(mi) };
    GetMonitorInfo(MonitorFromWindow(*hwn, MONITOR_DEFAULTTOPRIMARY), &mi);
    SetWindowPos(*hwn, HWND_TOP, (mi.rcMonitor.right - mi.rcMonitor.left) / 4, (mi.rcMonitor.bottom - mi.rcMonitor.top) / 4, (mi.rcMonitor.right - mi.rcMonitor.left)/2, (mi.rcMonitor.bottom - mi.rcMonitor.top)/2, SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
    //SetWindowPos(*hwn, HWND_TOP,0, 0, (mi.rcMonitor.right - mi.rcMonitor.left) , (mi.rcMonitor.bottom - mi.rcMonitor.top), SWP_NOOWNERZORDER | SWP_FRAMECHANGED);

  }break;
  }
  return 0;
}

u32 peek_message_wrapper(HWND* wind, Input** input_ptr) {
  MSG message;
  Input* GLOBALINPUT = *input_ptr;
  int count_msg = 0;
  while (PeekMessage(&message, *wind, 0, 0, PM_REMOVE)) {
    count_msg += 1;
    u32 vk_code = (u32)message.wParam;
    s16 x_pos = (s16)(message.lParam & (0x0000FFFF));
    s16 y_pos = (s16)((message.lParam & (0xFFFF0000)) >> 16);
    switch (message.message) {
    case WM_KEYUP:
    case WM_KEYDOWN: {
      bool is_down = ((message.lParam & (0x80000000)) == 0);
      switch (vk_code) {
        process_button(BUTTON_UP);
        process_button(BUTTON_DOWN);
        process_button(BUTTON_W);
        process_button(BUTTON_A);
        process_button(BUTTON_D);
        process_button(BUTTON_S);
        process_button(BUTTON_R);
        process_button(BUTTON_Q);
        process_button(BUTTON_E);
        process_button(BUTTON_T);
        process_button(BUTTON_LEFT);
        process_button(BUTTON_RIGHT);
        process_button(BUTTON_ENTER);
        process_button(BUTTON_ESCAPE);
        process_button(BUTTON_F1);
      }
    } break;
    case WM_MOUSEMOVE: {
      switch (vk_code) {
        process_mouse_move(BUTTON_LEFTMOUSE, MK_LBUTTON);
        process_mouse_move(BUTTON_RIGHTMOUSE, MK_RBUTTON);
      }
    }break;
    process_mouse_click(BUTTON_LEFTMOUSE, WM_LBUTTONUP, 0);
    process_mouse_click(BUTTON_RIGHTMOUSE, WM_RBUTTONUP, 0);
    process_mouse_click(BUTTON_LEFTMOUSE, WM_LBUTTONDOWN, 1);
    process_mouse_click(BUTTON_RIGHTMOUSE, WM_RBUTTONDOWN, 1);
    default: {
      TranslateMessage(&message);
      DispatchMessage(&message);
    }
    }//switch message
  }//while peekmessage
  return count_msg;
}

