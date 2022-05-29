#include "window_functions.h"
#include "Camera.h"
#include "Trixel.h"
#include "sort.h"
#include "Vector.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>

static bool* g_running = (bool*)malloc(sizeof(bool));
static Input* g_input = new Input();
struct resolution { u32 h; u32 w; double ar; } g_window_res, g_render_res;
#define GLOBALINPUT g_input
static  BITMAPINFO g_bitmap_info;

void read_ply(const char* file_name, double** points_list, s64* num_tri, kd_leaf_sort** leaf_list, kd_vertex** vertex_list, s64* num_vert, u8 mode);


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
    float delta_time = 0.016666f;
    LARGE_INTEGER create_window_begin_time;
    QueryPerformanceCounter(&create_window_begin_time);

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
    g_render_res.ar = g_window_res.ar;

    Camera* main_cam = new Camera(g_render_res.w, g_render_res.h, // res_w, res_h
        g_render_res.ar * .024, .024, .055, // f_w, f_h, focal_len
        0.0, 0.750, -1.0, // position
        0.0, 0.0, 0.0, // look at
        0.0, 1.0, 0.0// up
    );
    double* points_for_trixels = 0;
    kd_leaf_sort* kd_leaf_list = 0;
    kd_vertex* vertices_for_trixels;
    color* color_for_trixels = NULL;
    s64 num_trixels = 0;
    s64 num_trixel_vert = 0;

    LARGE_INTEGER read_scene_begin_time;
    QueryPerformanceCounter(&read_scene_begin_time);
    float performance_frequency;
    {
        LARGE_INTEGER perf;
        QueryPerformanceFrequency(&perf);
        performance_frequency = (float)perf.QuadPart;
    }
    //read_ply("dump_test.ply", &points_for_trixels, &num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);

    //HUGE DRAGON BOI
    //read_ply("dragon_vrip_mod.ply", &points_for_trixels, &num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);

    //BABy TRIXEL LIST
    read_ply("dump.ply", &points_for_trixels, &num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert,1);

    //BIG BOY RABBIT
    //read_ply("dump23.ply", &points_for_trixels, &num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert,1);

    color_for_trixels = (color*)malloc(sizeof(color) * num_trixels);
    for (int trixel_index = 0; trixel_index < num_trixels; trixel_index++) {
        color_for_trixels->argb[trixel_index].r = 200;
        color_for_trixels->argb[trixel_index].g = 0;
        color_for_trixels->argb[trixel_index].b = 0;

    }
    LARGE_INTEGER read_scene_end_time;
    QueryPerformanceCounter(&read_scene_end_time);
    delta_time = (float)(read_scene_end_time.QuadPart - read_scene_begin_time.QuadPart) / performance_frequency;
    printf("Time to Read Tree: %f seconds\n", delta_time);
    printf("\t\t primitives: %lld\n\n", num_trixels);



    LARGE_INTEGER build_tree_start_time;
    QueryPerformanceCounter(&build_tree_start_time);

    Trixel* trixel_list = new Trixel(num_trixels, points_for_trixels, color_for_trixels);
    trixel_list->set_sorted_voxels(kd_leaf_list, num_trixels);
    trixel_list->create_kd();

    LARGE_INTEGER build_tree_end_time;
    QueryPerformanceCounter(&build_tree_end_time);

    delta_time = (float)(build_tree_end_time.QuadPart - build_tree_start_time.QuadPart) / performance_frequency;
    printf("Time to build tree: %f seconds\n", delta_time);

    main_cam->init_camera_trixel_data(trixel_list, trixel_list->num_trixels);
    main_cam->init_camera_voxel_data(trixel_list, trixel_list->num_voxels);




    LARGE_INTEGER frame_begin_time;
    LARGE_INTEGER frame_end_time;

    QueryPerformanceCounter(&frame_begin_time);

    u32 render_mode = 0;
    #define TICK_RATE 30
    float cur_tick = 0;
    float cam_speed = .05f;
    while (*g_running) {
        if (cur_tick >= 1 / (TICK_RATE * delta_time)) {
            for (int i = 0; i < BUTTON_COUNT; i++) {
                g_input->buttons[i].changed = false;
            }
            int msg_cnt = peek_message_wrapper(&window, &g_input);
            if (is_button_release(BUTTON_ESCAPE)) {
                window_callback(window, WM_DESTROY, 0, 0); continue;
                fclose(dummyFile);
                FreeConsole();
            }
            if (is_click_hold(BUTTON_W)) {
                g_input->translate.dx = main_cam->o_prop.n.x * cam_speed;
                g_input->translate.dy = main_cam->o_prop.n.y * cam_speed;
                g_input->translate.dz = main_cam->o_prop.n.z * cam_speed;
                main_cam->translate(g_input->translate);
            }
            if (is_click_hold(BUTTON_S)) {
                g_input->translate.dx = -main_cam->o_prop.n.x * cam_speed;
                g_input->translate.dy = -main_cam->o_prop.n.y * cam_speed;
                g_input->translate.dz = -main_cam->o_prop.n.z * cam_speed;
                main_cam->translate(g_input->translate);
            }
            if (is_click_hold(BUTTON_Q)) { // STRAFE LEFT
                g_input->translate.dx = -main_cam->o_prop.u.x * cam_speed;
                g_input->translate.dy = -main_cam->o_prop.u.y * cam_speed;
                g_input->translate.dz = -main_cam->o_prop.u.z * cam_speed;
                main_cam->translate(g_input->translate);
            }
            if (is_click_hold(BUTTON_E)) {// STRAFE RIGHT
                g_input->translate.dx = main_cam->o_prop.u.x * cam_speed;
                g_input->translate.dy = main_cam->o_prop.u.y * cam_speed;
                g_input->translate.dz = main_cam->o_prop.u.z * cam_speed;
                main_cam->translate(g_input->translate);
            }
            if (is_button_release(BUTTON_R)) {
                render_mode = (render_mode + 1) % 2;
            }
            cur_tick = 0;
        }
        trixel_list->intersect_trixels(main_cam, render_mode);
         main_cam->color_pixels();
        StretchDIBits(hdc, 0, 0, g_render_res.w, g_render_res.h, 0, 0, g_render_res.w, g_render_res.h, (void*)main_cam->h_mem.h_color.c, &g_bitmap_info, DIB_RGB_COLORS, SRCCOPY);

        QueryPerformanceCounter(&frame_end_time);
        delta_time = (float)(frame_end_time.QuadPart - frame_begin_time.QuadPart) / performance_frequency;
        frame_begin_time = frame_end_time;
        wprintf(L"\x1b[s");
        printf("Last Cuda Error: %s\n", cudaGetErrorString(cudaPeekAtLastError()));
        printf("FPS: %f \n", 1 / delta_time);
        printf("CamerPos [x:%f y:%f z:%f] \n", main_cam->o_prop.pos.x, main_cam->o_prop.pos.y, main_cam->o_prop.pos.z);
        wprintf(L"\x1b[u");
        cur_tick++;
    } 

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cudaError_t cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        exit(0);
    }
    exit(0);
}