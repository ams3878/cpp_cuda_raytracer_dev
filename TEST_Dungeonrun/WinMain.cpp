#include "window_functions.h"
#include "Camera.h"
#include "Trixel.h"
#ifndef CUDA_KERNEL
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#endif
static bool* g_running = (bool*)malloc(sizeof(bool));
static Input* g_input = new Input();
struct resolution { u32 h; u32 w; double ar; } g_window_res, g_render_res;
#define GLOBALINPUT g_input
static  BITMAPINFO g_bitmap_info;


LRESULT CALLBACK window_callback(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    LRESULT result = 0;
    switch (uMsg) {
    case WM_CLOSE:
    case WM_DESTROY: {
        *g_running = false;
    } break;
    case WM_SIZE: {
        RECT rect;
        GetClientRect(hwnd, &rect);
        g_window_res.w = rect.right - rect.left;
        g_window_res.h = rect.bottom - rect.top;
        g_window_res.ar = (double)g_window_res.w / g_window_res.h;
        g_bitmap_info.bmiHeader.biSize = sizeof(g_bitmap_info.bmiHeader);
        g_bitmap_info.bmiHeader.biWidth = g_window_res.w;
        g_bitmap_info.bmiHeader.biHeight = g_window_res.h;
        g_bitmap_info.bmiHeader.biPlanes = 1;
        g_bitmap_info.bmiHeader.biBitCount = 32;
        g_bitmap_info.bmiHeader.biCompression = BI_RGB;
    }break;
    default: {
        result = DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
    }
    return result;
}

int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
    WNDCLASS* main_window_class = (WNDCLASS*)malloc(sizeof(WNDCLASS));
    initialize_hwnd_class(main_window_class, window_callback);
    HWND window = CreateWindow(main_window_class->lpszClassName, _T("Dungeon_Run"), WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, 0, 0, hInstance, 0); // Create Window  
    initialize_hwnd(&window, MAIN_WIN_CLASS);
    HDC hdc = GetDC(window);
    AllocConsole();
    FILE* dummyFile;
    freopen_s(&dummyFile, "CONOUT$", "wb", stdout);
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
    //For some reason these have to match but they should be allowed to be different 
    //**TODO** solve
    g_render_res.w =  g_window_res.w;
    g_render_res.h = g_window_res.h;
    Camera* main_cam = new Camera(g_render_res.w, g_render_res.h, // res_w, res_h
        g_render_res.ar * .024, .024, .055, // f_w, f_h, focal_len
        0.0, 0.0, -40.0, // position
        0.0, 0.0, 0.0, // look at
        0.0, 1.0, 0.0// up
    );
    //TEMP TRIXEL LIST
    double points_for_trixels[9];
    u32 color_for_trixels[1];
    color_for_trixels[0] = 42300;
    points_for_trixels[0] = 0.0;    points_for_trixels[1] = 0.0;    points_for_trixels[2] = 0.0;
    points_for_trixels[3] = -20.0;    points_for_trixels[4] = -20.0;    points_for_trixels[5] = 0.0;
    points_for_trixels[6] = 20.0;    points_for_trixels[7] = -20.0;    points_for_trixels[8] = 0.0;

    Trixel* trixel_list = new Trixel(1, points_for_trixels, color_for_trixels);
    trixel_list->init_camera_trixel_data(main_cam);
    float delta_time = 0.016666f;
    LARGE_INTEGER frame_begin_time;
    QueryPerformanceCounter(&frame_begin_time);

    float performance_frequency;
    {
        LARGE_INTEGER perf;
        QueryPerformanceFrequency(&perf);
        performance_frequency = (float)perf.QuadPart;
    }
    while (*g_running) {
        for (int i = 0; i < BUTTON_COUNT; i++) {
            g_input->buttons[i].changed = false;
        }
        int msg_cnt = peek_message_wrapper(&window, &g_input);
        if (is_button_release(BUTTON_ESCAPE)) {
            window_callback(window, WM_DESTROY, 0, 0); continue;
            fclose(dummyFile);
            FreeConsole();
        }
        trixel_list->intersect_trixels(main_cam);
        StretchDIBits(hdc, 0, 0, g_render_res.w, g_render_res.h, 0, 0, g_render_res.w, g_render_res.h, (void*)main_cam->h_mem.h_color.c, &g_bitmap_info, DIB_RGB_COLORS, SRCCOPY);

        LARGE_INTEGER frame_end_time;
        QueryPerformanceCounter(&frame_end_time);
        delta_time = (float)(frame_end_time.QuadPart - frame_begin_time.QuadPart) / performance_frequency;
        frame_begin_time = frame_end_time;
        wprintf(L"\x1b[s");
        printf("Last Cuda Error: %s\n", cudaGetErrorString(cudaPeekAtLastError()));
        printf("FPS: %f \n", 1 / delta_time);
        printf("CamerPos [x:%f y:%f z:%f] \n", main_cam->o_prop.pos.x, main_cam->o_prop.pos.y, main_cam->o_prop.pos.z);
        wprintf(L"\x1b[u");
    } 

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cudaError_t cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        return 1;
    }
    return 0;
}