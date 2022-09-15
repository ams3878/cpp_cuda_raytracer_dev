#include "window_functions.h"
#include "framework.h"
#include "sort.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>

static bool* g_running = (bool*)malloc(sizeof(bool));
static Input* g_input = new Input();
struct resolution { u32 h; u32 w; T_fp ar; } g_window_res, g_render_res;
#define GLOBALINPUT g_input
static  BITMAPINFO g_bitmap_info;

void read_ply(const char* file_name, T_fp** points_list, T_uint* num_tri, kd_leaf_sort** leaf_list, kd_vertex** vertex_list, T_uint* num_vert, u8 mode);


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
        g_window_res.ar = (T_fp)g_window_res.w / g_window_res.h;
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

int CALLBACK WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
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
        g_render_res.ar * (T_fp).024, (T_fp).024, (T_fp).055, // f_w, f_h, focal_len
        (T_fp)0.0, (T_fp)0.10, (T_fp)-1.0, // position
        (T_fp)0.00, (T_fp)0.100, (T_fp)0.00, // look at
        (T_fp)0.0, (T_fp)1.0, (T_fp)0.0// up
    );
    T_fp* points_for_trixels = NULL;
    kd_leaf_sort* kd_leaf_list = NULL;
    kd_vertex* vertices_for_trixels;
    Color color_for_trixels;
    T_uint num_trixels_1 = 0, num_trixels_2 = 0, num_trixel_vert = 0, tot_num_trixels = 0;

    LARGE_INTEGER read_scene_begin_time;
    QueryPerformanceCounter(&read_scene_begin_time);
    float performance_frequency;
    {
        LARGE_INTEGER perf;
        QueryPerformanceFrequency(&perf);
        performance_frequency = (float)perf.QuadPart;
    }
    int model_select = 0;
    switch(model_select){
    case 0:
        //BIG BOY RABBIT
        read_ply("rabbit_70k.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 1);
        break;
    case 1:
        //HUGE DRAGON BOI
        read_ply("dragon_vrip_mod.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);
        break;
    case 2:
        //HUGE BUDDHA BOU
        read_ply("happy_vrip_mod.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 0);
        break;
    case 3:
        //HUGE BUDDHA BOU
        read_ply("tester.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 2);
        break;
    case 4:
        //test ply read
        read_ply("3_walls.ply", &points_for_trixels, &tot_num_trixels, &kd_leaf_list, &vertices_for_trixels, &num_trixel_vert, 3);
        break;
    }   


    color_for_trixels.c = (u32*)malloc(sizeof(u32) * tot_num_trixels);
    color_for_trixels.rad = (Color::radiance*)malloc(sizeof(Color::radiance) * tot_num_trixels);
    if (!color_for_trixels.rad){ return NULL; }
    for (T_uint trixel_index = 0; trixel_index < tot_num_trixels; trixel_index++) {
        color_for_trixels.rad[trixel_index].r = (T_fp).1;
        color_for_trixels.rad[trixel_index].g = (T_fp).55;
        color_for_trixels.rad[trixel_index].b = (T_fp).20;
    }
    //**TODO** make a timing macro
    LARGE_INTEGER read_scene_end_time;
    QueryPerformanceCounter(&read_scene_end_time);
    delta_time = (float)(read_scene_end_time.QuadPart - read_scene_begin_time.QuadPart) / performance_frequency;
    printf("Time to Read Tree: %f seconds\n", delta_time);
    printf("\t\t primitives: %d\n\n", tot_num_trixels);


    printf("Begining Spatial Heiarchy build\n\tSorting voxels....");
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
    Object* obj1 = new Object(trixel_list);
    Object* obj2 = new Object(trixel_list);

    main_cam->add_object(obj1);
    main_cam->add_object(obj2);


    //Quaternion s = Quaternion(200000000);
    //s.initialize_CUDA();
    //s._free();

    LARGE_INTEGER frame_begin_time;
    LARGE_INTEGER frame_end_time;

    QueryPerformanceCounter(&frame_begin_time);

    u32 render_mode = 0;
    #define TICK_RATE 30
    float cur_tick = 0;
    float cam_speed = .005f;


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
            if (is_click_hold(BUTTON_R)) {
                (*g_input).set_quat((T_fp)0.0, (T_fp)0.09950371902099893, (T_fp)0.0, (T_fp)0.9950371902099893);
                obj1->transform(g_input, ROTATE_TRI_PY);
            }
            if (is_click_hold(BUTTON_W)) {
                (*g_input).set_quat(main_cam->o_prop.n.x,  main_cam->o_prop.n.y, main_cam->o_prop.n.z, cam_speed);
                obj1->transform(g_input, TRANSLATE_Z);
            }
            if (is_click_hold(BUTTON_S)) {
                (*g_input).set_quat(main_cam->o_prop.n.x, main_cam->o_prop.n.y, main_cam->o_prop.n.z, -cam_speed);
                obj1->transform(g_input, TRANSLATE_Z);
            }
            if (is_click_hold(BUTTON_Q)) { // STRAFE LEFT
                (*g_input).set_quat(main_cam->o_prop.u.x, main_cam->o_prop.u.y, main_cam->o_prop.u.z, cam_speed);
                obj1->transform(g_input, TRANSLATE_X);
            }
            if (is_click_hold(BUTTON_E)) {// STRAFE RIGHT
                (*g_input).set_quat(main_cam->o_prop.u.x, main_cam->o_prop.u.y, main_cam->o_prop.u.z, -cam_speed);
                obj1->transform(g_input, TRANSLATE_X);
            }
            if (is_click_hold(BUTTON_T)) {
                (*g_input).set_quat((T_fp)0.0, (T_fp)-0.09950371902099893, (T_fp)0.0, (T_fp)0.9950371902099893);
                obj1->transform(g_input, ROTATE_TRI_NY);
            }
            cur_tick = 0;
        }
        obj1->render(main_cam);
        main_cam->color_pixels(PHONG_COLOR_TAG);
       // obj2->render(main_cam);
        //main_cam->color_pixels(PHONG_COLOR_TAG);

        StretchDIBits(hdc, 0, 0, g_render_res.w, g_render_res.h, 0, 0, g_render_res.w, g_render_res.h, (void*)main_cam->h_mem.h_color.c, &g_bitmap_info, DIB_RGB_COLORS, SRCCOPY);

        QueryPerformanceCounter(&frame_end_time);
        delta_time = (float)(frame_end_time.QuadPart - frame_begin_time.QuadPart) / performance_frequency;
        frame_begin_time = frame_end_time;
        bool gogogo = true;
        //gogogo = false;
        if (gogogo) {
            wprintf(L"\x1b[s");
            printf("Resolution: %d x %d\n", g_render_res.w, g_render_res.h);
            printf("Last Cuda Error: %s\n", cudaGetErrorString(cudaPeekAtLastError()));
            printf("FPS: %f \n", 1 / delta_time);
            printf("CamerPos [x:%f y:%f z:%f] \n", main_cam->o_prop.pos.x, main_cam->o_prop.pos.y, main_cam->o_prop.pos.z);
            printf("Camera N [x:%f y:%f z:%f] \n", main_cam->o_prop.n.x, main_cam->o_prop.n.y, main_cam->o_prop.n.z);
            printf("Camera U [x:%f y:%f z:%f] \n", main_cam->o_prop.u.x, main_cam->o_prop.u.y, main_cam->o_prop.u.z);
            printf("Camera V [x:%f y:%f z:%f] \n", main_cam->o_prop.v.x, main_cam->o_prop.v.y, main_cam->o_prop.v.z);

            wprintf(L"\x1b[u");
        }
        cur_tick++;
        main_cam->color_pixels(SET_COLOR_TAG);

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