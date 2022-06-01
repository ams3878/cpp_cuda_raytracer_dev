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
       -0.05, .10, .32, // position
        -0.05, 0.10, 0.0, // look at
        0.0, 1.0, 0.0// up
    );
    double* points_for_trixels = 0, *points_for_trixels_1 = 0, * points_for_trixels_2 = 0;
    kd_leaf_sort* kd_leaf_list = 0, *kd_leaf_list_1 = 0, * kd_leaf_list_2 = 0;
    kd_vertex* vertices_for_trixels;
    color color_for_trixels;
    s64 num_trixels_1 = 0, num_trixels_2 = 0, num_trixel_vert = 0, tot_num_trixels = 0;

    LARGE_INTEGER read_scene_begin_time;
    QueryPerformanceCounter(&read_scene_begin_time);
    float performance_frequency;
    {
        LARGE_INTEGER perf;
        QueryPerformanceFrequency(&perf);
        performance_frequency = (float)perf.QuadPart;
    }
    //read_ply("dump_test.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);

    //HUGE DRAGON BOI
    //read_ply("dragon_vrip_mod.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);
    //HUGE BUDDHA BOU
    //read_ply("happy_vrip_mod.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);
    //BABy TRIXEL LIST
    /*
    read_ply("dump.ply", &points_for_trixels_1, &num_trixels_1, &kd_leaf_list_1, &vertices_for_trixels, &num_trixel_vert, 1);
    tot_num_trixels += num_trixels_1;
    num_trixels_2 = tot_num_trixels;
    read_ply("dump.ply", &points_for_trixels_2, &num_trixels_2, &kd_leaf_list_2, &vertices_for_trixels, &num_trixel_vert, 1);
    tot_num_trixels += num_trixels_2;

    points_for_trixels = (double*)malloc(sizeof(double) * tot_num_trixels * 9);
    kd_leaf_list = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * tot_num_trixels);

    memcpy(points_for_trixels, points_for_trixels_1, sizeof(double) * num_trixels_1 * 9);
    memcpy(points_for_trixels + ((num_trixels_1 -1)* 9), points_for_trixels_2, sizeof(double) * num_trixels_2 * 9);

    memcpy(kd_leaf_list, kd_leaf_list_1, sizeof(kd_leaf_sort) * num_trixels_1);
    memcpy(kd_leaf_list + num_trixels_1 - 1, kd_leaf_list_2, sizeof(kd_leaf_sort) * num_trixels_2);
    free(points_for_trixels_1);    free(points_for_trixels_2);    free(kd_leaf_list_1);    free(kd_leaf_list_2);
    */
    //read_ply("dump.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 1);

    //BIG BOY RABBIT
    read_ply("dump23.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert,1);

    color_for_trixels.c = (u32*)malloc(sizeof(u32) * tot_num_trixels);
    color_for_trixels.rad = (color::radiance*)malloc(sizeof(color::radiance) * tot_num_trixels);

    for (int trixel_index = 0; trixel_index < tot_num_trixels; trixel_index++) {
        color_for_trixels.rad[trixel_index].r = .1;
        color_for_trixels.rad[trixel_index].g = .4;
        color_for_trixels.rad[trixel_index].b = .8;
    }
    //**TODO** make a timing macro
    LARGE_INTEGER read_scene_end_time;
    QueryPerformanceCounter(&read_scene_end_time);
    delta_time = (float)(read_scene_end_time.QuadPart - read_scene_begin_time.QuadPart) / performance_frequency;
    printf("Time to Read Tree: %f seconds\n", delta_time);
    printf("\t\t primitives: %lld\n\n", tot_num_trixels);


    printf("Begining Spatial Heiarchy build\n\tSorting voxels....", delta_time);
    LARGE_INTEGER sort_primitives_start_time;
    QueryPerformanceCounter(&sort_primitives_start_time);

    Trixel* trixel_list = new Trixel(tot_num_trixels, points_for_trixels, &color_for_trixels);
    trixel_list->set_sorted_voxels(kd_leaf_list, tot_num_trixels);

    LARGE_INTEGER sort_primitives_end_time;
    QueryPerformanceCounter(&sort_primitives_end_time);
    printf("complete. %f seconds\n\tPartioning voxels......", (float)(sort_primitives_end_time.QuadPart - sort_primitives_start_time.QuadPart) / performance_frequency);

    LARGE_INTEGER build_kdtree_start_time;
    QueryPerformanceCounter(&build_kdtree_start_time);

    trixel_list->create_kd();

    LARGE_INTEGER build_kdtree_end_time;
    QueryPerformanceCounter(&build_kdtree_end_time);
    printf("complete. %f seconds\n", (float)(build_kdtree_end_time.QuadPart - build_kdtree_start_time.QuadPart) / performance_frequency);


    printf("Total Time to build tree: %f seconds\n", (float)(build_kdtree_end_time.QuadPart - sort_primitives_start_time.QuadPart) / performance_frequency);


    main_cam->init_camera_trixel_data(trixel_list, trixel_list->num_trixels);
    main_cam->init_camera_voxel_data(trixel_list, trixel_list->num_voxels);
    cudaFree(trixel_list->h_tree.d_nodes);




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
                main_cam->transform(g_input->translate, TRANSLATE_XYZ);
            }
            if (is_click_hold(BUTTON_S)) {
                g_input->translate.dx = -main_cam->o_prop.n.x * cam_speed;
                g_input->translate.dy = -main_cam->o_prop.n.y * cam_speed;
                g_input->translate.dz = -main_cam->o_prop.n.z * cam_speed;
                main_cam->transform(g_input->translate, TRANSLATE_XYZ);
            }
            if (is_click_hold(BUTTON_Q)) { // STRAFE LEFT
                g_input->translate.dx = -main_cam->o_prop.u.x * cam_speed;
                g_input->translate.dy = -main_cam->o_prop.u.y * cam_speed;
                g_input->translate.dz = -main_cam->o_prop.u.z * cam_speed;
                main_cam->transform(g_input->translate, TRANSLATE_XYZ);
            }
            if (is_click_hold(BUTTON_E)) {// STRAFE RIGHT
                g_input->translate.dx = main_cam->o_prop.u.x * cam_speed;
                g_input->translate.dy = main_cam->o_prop.u.y * cam_speed;
                g_input->translate.dz = main_cam->o_prop.u.z * cam_speed;
                main_cam->transform(g_input->translate, TRANSLATE_XYZ);
            }
            if (is_button_release(BUTTON_R)) {
                g_input->translate.dx = .5;
                g_input->translate.dy = .5;
                g_input->translate.dz = .5;
                main_cam->transform(g_input->translate, SCALE_XYZ);
            }
            if (is_button_release(BUTTON_T)) {
                g_input->translate.dx = 2.0;
                g_input->translate.dy = 2.0;
                g_input->translate.dz = 2.0;
                main_cam->transform(g_input->translate, SCALE_XYZ);
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