#ifndef CUDA_KERNEL
#define CUDA_KERNEL
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#endif
#include "platform_common.h"
#include "Camera.h"
#include "Trixel.h"
#ifndef CUDA_VECTOR_H
#include "vector.cuh" 
#endif

__global__ void color_cam_cuda(Camera::pixel_memory* cm, u32* t, u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    u32 pattern = i % 16;
    double sdx, sdy, sdz, dot_r_n,rx,ry,rz;
    double phong_difuse_modifier, phong_spec_modifier;
    double tri_cr = (double)cm->d_color.rad[i].r, tri_cg = (double)cm->d_color.rad[i].g, tri_cb = (double)cm->d_color.rad[i].b;
    double point_rad_r = 0.0, point_rad_g = 0.0, point_rad_b = 0.0;
    if (cm->d_rmi.index[i] >= 0) {// if ray hit an object
        //Check Shadows
        //for (int i = 0; i < 1; i++) {
            //Get light to intersect point vector
        // just put light at 2,2,2 for now
            sdx = 2 - cm->pnt.d_x[i]; sdy = 2 - cm->pnt.d_y[i]; sdz = 2 - cm->pnt.d_z[i];
            //in_shadow = (-1 != device_moller_trumbore(e2x, e2y, e2z, e1x, e1y, e1z, qx, qy, qz, tx, ty, tz, num_tri, w_i, sdx, sdy, sdz, &temp_rmsd));
            if (1) {//if not in shadow do "phong"
                //**TODO** ADD LIGHT INTESITY
                device_normalize_vector(&sdx, &sdy, &sdz);
                //**CAN DO EXTRA PRECALC** nxx, nxy, nyy, nyz, nzz
                dot_r_n = device_dot(sdx, sdy, sdz, cm->norm.d_x[i], cm->norm.d_x[i], cm->norm.d_z[i]);
                rx = (sdx - (2 * dot_r_n * cm->norm.d_x[i])) * cm->rmd.d_x[i];
                ry = (sdy - (2 * dot_r_n * cm->norm.d_y[i])) * cm->rmd.d_y[i];
                rz = (sdz - (2 * dot_r_n * cm->norm.d_z[i])) * cm->rmd.d_z[i];

                //r = s.negate().reflect(surface_norm, hw = o_ref.blin)
                phong_difuse_modifier = .6 * fabs(dot_r_n);
                phong_spec_modifier = powf(fabs((rx + ry + rz)), 5) * .3;

                // d_s_color = object.color * o_ref.diffuse * abs(inter.dot(s.nv, surface_norm)
                //d_s_color += light.color * (abs(inter.dot(r.nv, v_n.nv)) **o_ref.exponent) * o_ref.specular

                point_rad_r += (tri_cr * phong_difuse_modifier) + (1 * phong_spec_modifier);
                point_rad_g += (tri_cg * phong_difuse_modifier) + (1 * phong_spec_modifier);
                point_rad_b += (tri_cb * phong_difuse_modifier) + (1 * phong_spec_modifier);
            }//END IF NOT IN SHADOW
        //}//FOR EACH LIGHT
       //**TODO**Basic Repro here, change later
            double max_rad = fmax(fmax(point_rad_r, point_rad_g), point_rad_b);
            cm->d_color.argb[i].r = (u8)((point_rad_r / max_rad) * 255);
            cm->d_color.argb[i].g = (u8)((point_rad_g / max_rad) * 255);
            cm->d_color.argb[i].b = (u8)((point_rad_b / max_rad) * 255);
            cm->d_color.argb[i].a = (u8)0;
    }
    else if(cm->d_rmi.index[i] == -2){
        double max_rad = fmax(fmax(tri_cr, tri_cg), tri_cb);
        cm->d_color.argb[i].r = (u8)((tri_cr ) * 255);
        cm->d_color.argb[i].g = (u8)((tri_cg ) * 255);
        cm->d_color.argb[i].b = (u8)((tri_cb ) * 255);
        cm->d_color.argb[i].a = (u8)0;
    }
    else {
        cm->d_color.argb[i].r = (u8)240;  cm->d_color.argb[i].b = (u8)240; cm->d_color.argb[i].g = (u8)0; cm->d_color.argb[i].a = (u8)0;
        if (pattern < 8) { cm->d_color.argb[i].g = (u8)240; cm->d_color.argb[i].r = (u8)0; }
    }
}
cudaError_t color_camera_device(Camera* c) {
    cudaError_t cudaStatus;
    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    }
    color_cam_cuda << < 1 + ((u32)c->f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >> > ((Camera::pixel_memory*)c->d_mem, c->trixels_list->h_mem.d_color.c, c->f_prop.res.count);
    status_lauch_and_sync(color_cam_cuda);
    cudaMemcpy(c->h_mem.h_color.c, c->h_mem.d_color.c, c->f_prop.res.count * sizeof(u32), cudaMemcpyDeviceToHost);

    return cudaStatus;
}


__global__ void init_cam_mem_cuda(Camera::pixel_memory* m, u64 res_w, u64 res_h, double draw_distance, double pix_w, double pix_h,
    Camera::orientation_properties::o_vector n,    Camera::orientation_properties::o_vector v,    Camera::orientation_properties::o_vector u){
    //MAYBE make o_vectors shared mem? if you do delete the o_vector mod memebers
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= (res_w * res_h)) { return; }
    u64 i_y = i / res_w;
    u64 i_x = i % res_w;
    u32 pattern = i % 8;

    m->rad.d_r[i] = 0.0;    m->rad.d_g[i] = 0.0;    m->rad.d_b[i] = 0.0;
   // m->d_color.argb[i].r = (u8)240;  m->d_color.argb[i].b = (u8)0; m->d_color.argb[i].g = (u8)0; m->d_color.argb[i].a = (u8)0;
    //if(pattern < 4){ m->d_color.argb[i].g = (u8)240; m->d_color.argb[i].r = (u8)0;}
    m->norm.d_x[i] = 0.0;   m->norm.d_y[i] = 0.0;   m->norm.d_z[i] = 0.0;
    m->pnt.d_x[i] = 0.0;    m->pnt.d_y[i] = 0.0;    m->pnt.d_z[i] = 0.0;


    m->rmd.d_x[i] = n.x + u.x * i_x + v.x * i_y;    m->rmd.d_y[i] = n.y + u.y * i_x + v.y * i_y;    m->rmd.d_z[i] = n.z + u.z * i_x + v.z * i_y;
    device_normalize_vector(&m->rmd.d_x[i], &m->rmd.d_y[i], &m->rmd.d_z[i]);

    m->inv_rmd.d_x[i] = 1/m->rmd.d_x[i];    m->inv_rmd.d_y[i] = 1/ m->rmd.d_y[i];    m->inv_rmd.d_z[i] = 1/ m->rmd.d_z[i];
    m->sign_rmd.d_x[i] = ((u64*)m->rmd.d_x)[i] >> 63;    m->sign_rmd.d_y[i] = ((u64*)m->rmd.d_y)[i] >> 63;    m->sign_rmd.d_z[i] = ((u64*)m->rmd.d_z)[i] >> 63;

    m->d_dist.d[i] = draw_distance;
    m->d_rmi.index[i] = -1;    
}

cudaError_t init_camera_device_memory(Camera* c){
    cudaError_t cudaStatus;
    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    }  
    init_cam_mem_cuda << < 1+((u32)c->f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >>> ((Camera::pixel_memory*)c->d_mem, c->f_prop.res.w, c->f_prop.res.h, c->r_prop.draw_distance,
        c->f_prop.pix.w, c->f_prop.pix.h, c->o_prop.n_mod, c->o_prop.v_mod, c->o_prop.u_mod);
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

__global__ void init_cam_voxel_mem_cuda(Camera::voxel_memory* vm, Trixel::kd_tree* kdm, vector_xyz co, u64 max_threads) {

    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    int cut_dir = kdm->d_nodes[i].cut_flag;
    vm->d_Bo[i].t0x = kdm->d_nodes[i].h_bound.x0 - co.x;
    vm->d_Bo[i].t1x = kdm->d_nodes[i].h_bound.x1 - co.x;
    vm->d_Bo[i].t0y = kdm->d_nodes[i].h_bound.y0 - co.y;
    vm->d_Bo[i].t1y = kdm->d_nodes[i].h_bound.y1 - co.y;
    vm->d_Bo[i].t0z = kdm->d_nodes[i].h_bound.z0 - co.z;
    vm->d_Bo[i].t1z = kdm->d_nodes[i].h_bound.z1 - co.z;
    vm->is_leaf[i] = kdm->d_nodes[i].is_leaf;
    vm->children[i].left = kdm->d_nodes[i].left_node;
    vm->children[i].right= kdm->d_nodes[i].right_node;
    vm->children[i].parent = kdm->d_nodes[i].parent;

    vm->children[i].triangle = vm->is_leaf[i] == 0 ? -1 : kdm->d_nodes[i].tri_index;

    vm->cut_flags[i].x = cut_dir == 0 || cut_dir == 3 ? 1 : 0;
    vm->cut_flags[i].y = cut_dir == 1 || cut_dir == 4 ? 1 : 0;
    vm->cut_flags[i].z = cut_dir == 2 || cut_dir == 5 ? 1 : 0;


    vm->s1[i] = kdm->d_nodes[i].s1 - ((co.x * (double)vm->cut_flags[i].x) + (co.y * (double)vm->cut_flags[i].y) + (co.z * (double)vm->cut_flags[i].z));
    vm->s2[i] = kdm->d_nodes[i].s2 - ((co.x * (double)vm->cut_flags[i].x) + (co.y * (double)vm->cut_flags[i].y) + (co.z * (double)vm->cut_flags[i].z));

}

cudaError_t init_camera_voxel_device_memory(Trixel* t, Camera* c) {
    cudaError_t cudaStatus;
    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    }
    init_cam_voxel_mem_cuda << < 1 + (u32)(t->num_voxels / BLOCK_SIZE)  , BLOCK_SIZE >> > (
        (Camera::voxel_memory*)c->d_voxels,
        (Trixel::kd_tree*)t->d_tree,
        c->o_prop.pos, t->num_voxels);
    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        printf("init_cam_voxel_cuda launch failed: %s\n", cudaGetErrorString(cudaStatus));
    }

    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        printf("cudaDeviceSynchronize returned error code %d after launching init_cam_mem_cuda!\n", cudaStatus);
    }
    return cudaStatus;
}


__global__ void translate_trixels_device_cuda(Trixel::trixel_memory* tm, Camera::trixel_memory* cm, Input::translate_vector a, u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    cm->d_t.x[i] += a.dx; cm->d_t.y[i] += a.dy;	cm->d_t.z[i] += a.dz;
    //Still pretty sure its move calculations to unroll and try to just =+ the detla.,...but maybe precomute e1 * e2 woudl speed it up? would have to unroll and look
    device_cross(&cm->d_q.x[i], &cm->d_q.y[i], &cm->d_q.z[i], cm->d_t.x[i], cm->d_t.y[i], cm->d_t.z[i], tm->d_edges.e1.x[i], tm->d_edges.e1.y[i], tm->d_edges.e1.z[i]);
    cm->d_w[i] = device_dot(cm->d_q.x[i], cm->d_q.y[i], cm->d_q.z[i], tm->d_edges.e2.x[i], tm->d_edges.e2.y[i], tm->d_edges.e2.z[i]);
}



__global__ void translate_cam_voxel_mem_cuda(Camera::voxel_memory* vm, Input::translate_vector tv, u64 max_threads) {

    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    //THESE VALUES ARE ALL CALCULATED AS SOME POINT IN SPACE minus THE CAMERA ORIGIN
    //So if you move the camera by  + dx...need to subtract dx from the voxel
    vm->d_Bo[i].t0x -= tv.dx;    vm->d_Bo[i].t1x -= tv.dx;
    vm->d_Bo[i].t0y -= tv.dy;    vm->d_Bo[i].t1y -= tv.dy;
    vm->d_Bo[i].t0z -= tv.dz;    vm->d_Bo[i].t1z -= tv.dz;

    vm->s1[i] -= (tv.dx * (double)vm->cut_flags[i].x) + (tv.dy * (double)vm->cut_flags[i].y) + (tv.dz * (double)vm->cut_flags[i].z);
    vm->s2[i] -= (tv.dx * (double)vm->cut_flags[i].x) + (tv.dy * (double)vm->cut_flags[i].y) + (tv.dz * (double)vm->cut_flags[i].z);

}
cudaError_t transform_trixels_device(Trixel* t, Camera* c, Input::translate_vector transform_vector, u8 transform_flag) {
    cudaError_t cudaStatus;
    switch (transform_flag) {
    case TRANSLATE_XYZ:
        translate_trixels_device_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > ((Trixel::trixel_memory*)t->d_mem, (Camera::trixel_memory*)c->d_trixels, transform_vector, t->num_trixels);
        break;
    }
    status_lauch_and_sync(scale_trixels_device_cuda);
    return cudaStatus;
}
cudaError_t transform_camera_voxel_device_memory(Camera* c, Input::translate_vector tv,  u8 transform_select) {
    cudaError_t cudaStatus;
    switch (transform_select) {
    case TRANSLATE_XYZ:
        translate_cam_voxel_mem_cuda << < 1 + (u32)(c->h_voxels.num_voxels / BLOCK_SIZE), BLOCK_SIZE >> > (
            (Camera::voxel_memory*)c->d_voxels,
            tv, c->h_voxels.num_voxels);
        break;
   }
    status_lauch_and_sync(translate_cam_voxel_mem_cuda);
    return cudaStatus;
}

