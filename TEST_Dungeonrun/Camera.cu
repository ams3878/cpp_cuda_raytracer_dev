#ifndef CUDA_KERNEL
#define CUDA_KERNEL
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#endif
#include "platform_common.h"
#include "Camera.h"
#include "Trixel.h"

#define BLOCK_SIZE 512

__global__ void init_cam_mem_cuda(Camera::pixel_memory* m, u64 res_w, u64 res_h, double draw_distance, double pix_w, double pix_h,
    Camera::orientation_properties::o_vector n,    Camera::orientation_properties::o_vector v,    Camera::orientation_properties::o_vector u){
    //MAYBE make o_vectors shared mem? if you do delete the o_vector mod memebers
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    u64 i_y = i / res_w;
    u64 i_x = i % res_w;
    u32 pattern = i % 8;

    m->rad.d_r[i] = 0.0;    m->rad.d_g[i] = 0.0;    m->rad.d_b[i] = 0.0;
    m->d_color.argb[i].r = (u8)240;  m->d_color.argb[i].b = (u8)0; m->d_color.argb[i].a = (u8)0;
    if(pattern < 4){ m->d_color.argb[i].g = (u8)240; m->d_color.argb[i].r = (u8)0;}
    m->norm.d_x[i] = 0.0;   m->norm.d_y[i] = 0.0;   m->norm.d_z[i] = 0.0;
    m->pnt.d_x[i] = 0.0;    m->pnt.d_y[i] = 0.0;    m->pnt.d_z[i] = 0.0;

    //**TODO** These all need normalized
    m->rmd.d_x[i] = n.x + u.x * i_x + v.x * i_y;
    m->rmd.d_y[i] = n.y + u.y * i_x + v.y * i_y;
    m->rmd.d_z[i] = n.z + u.z * i_x + v.z * i_y;

    m->d_dist.d[i] = draw_distance;
    m->d_rmi.index[i] = -1;    
}
cudaError_t init_camera_device_memory(Camera* c, Trixel* t) {
    cudaError_t cudaStatus = init_camera_device_memory(c);

    return cudaStatus;
};

cudaError_t init_camera_device_memory(Camera* c){
    cudaError_t cudaStatus;
    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    }  
    u64 bl_size = (c->f_prop.res.count < BLOCK_SIZE) ? c->f_prop.res.count : BLOCK_SIZE;
    init_cam_mem_cuda <<< c->f_prop.res.count / bl_size, bl_size >>> ((Camera::pixel_memory*)c->d_mem, c->f_prop.res.w, c->f_prop.res.h, c->r_prop.draw_distance,
        c->f_prop.pix.w, c->f_prop.pix.w, c->o_prop.n_mod, c->o_prop.v_mod, c->o_prop.u_mod);
    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        printf("init_cam_mem_cuda launch failed: %s\n", cudaGetErrorString(cudaStatus));
    }
    
    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        printf("cudaDeviceSynchronize returned error code %d after launching init_cam_mem_cuda!\n", cudaStatus);
    }
    cudaMemcpy(c->h_mem.h_rmi.index, c->h_mem.d_rmi.index, c->f_prop.res.count * sizeof(u64), cudaMemcpyDeviceToHost);
    cudaMemcpy(c->h_mem.h_color.c, c->h_mem.d_color.c, c->f_prop.res.count * sizeof(u32), cudaMemcpyDeviceToHost);
    return cudaStatus;
}
