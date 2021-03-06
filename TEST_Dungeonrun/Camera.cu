#ifndef CUDA_KERNEL
#define CUDA_KERNEL
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <cuda_profiler_api.h>
#include "cuda_profiler_api.h"
#include <stdio.h>
#endif
#include "framework.h"
#include "framework.cuh"


__global__ void color_cam_cuda(Camera::pixel_memory* cm, u32* t, u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    u32 pattern = i % 16;
    T_fp sdx, sdy, sdz, dot_r_n,rx,ry,rz;
    T_fp phong_difuse_modifier, phong_spec_modifier;
    T_fp tri_cr = (T_fp)cm->d_color.rad[i].r, tri_cg = (T_fp)cm->d_color.rad[i].g, tri_cb = (T_fp)cm->d_color.rad[i].b;
    T_fp point_rad_r = 0.0, point_rad_g = 0.0, point_rad_b = 0.0;

    if (cm->d_rmi.index[i] >= 0) {// if ray hit an object
        //Check Shadows
        //for (int i = 0; i < 1; i++) {
            //Get light to intersect point vector
        // just put light at 2,2,2 for now
            sdx = 2 - cm->pnt.x[i]; sdy = 2 - cm->pnt.y[i]; sdz = 2 - cm->pnt.z[i];
            //in_shadow = (-1 != device_moller_trumbore(e2x, e2y, e2z, e1x, e1y, e1z, qx, qy, qz, tx, ty, tz, num_tri, w_i, sdx, sdy, sdz, &temp_rmsd));
            if (1) {//if not in shadow do "phong"
                //**TODO** ADD LIGHT INTESITY
                device_normalize_vector(&sdx, &sdy, &sdz);
                //**CAN DO EXTRA PRECALC** nxx, nxy, nyy, nyz, nzz
                dot_r_n = device_dot(sdx, sdy, sdz, cm->norm.x[i], cm->norm.x[i], cm->norm.z[i]);
                rx = (sdx - (2 * dot_r_n * cm->norm.x[i])) * cm->rmd.x[i];
                ry = (sdy - (2 * dot_r_n * cm->norm.y[i])) * cm->rmd.y[i];
                rz = (sdz - (2 * dot_r_n * cm->norm.z[i])) * cm->rmd.z[i];

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
            T_fp max_rad = fmaxf(fmaxf(point_rad_r, point_rad_g), point_rad_b);
            cm->d_color.argb[i].r = (u8)((point_rad_r / max_rad) * 255);
            cm->d_color.argb[i].g = (u8)((point_rad_g / max_rad) * 255);
            cm->d_color.argb[i].b = (u8)((point_rad_b / max_rad) * 255);
            cm->d_color.argb[i].a = (u8)0;
    }
    else if(cm->d_rmi.index[i] == -2){
        T_fp max_rad = fmaxf(fmaxf(tri_cr, tri_cg), tri_cb);
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
    color_cam_cuda << < 1 + ((u32)c->f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >> > (c->d_mem, c->trixels_list->h_mem.d_color.c, c->f_prop.res.count);
    status_lauch_and_sync(color_cam_cuda);
    cudaMemcpy(c->h_mem.h_color.c, c->h_mem.d_color.c, c->f_prop.res.count * sizeof(u32), cudaMemcpyDeviceToHost);

    return cudaStatus;
}

__global__ void init_cam_mem_cuda(Camera::pixel_memory* m, u64 res_w, u64 res_h, T_fp draw_distance, T_fp pix_w, T_fp pix_h,
    VEC3<T_fp> n, VEC3<T_fp> v, VEC3<T_fp> u){
    //MAYBE make o_vectors shared mem? if you do delete the o_vector mod memebers
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= (res_w * res_h)) { return; }
    u64 i_y = i / res_w;
    u64 i_x = i % res_w;

    m->rad.r[i] = 0.0;    m->rad.g[i] = 0.0;    m->rad.b[i] = 0.0;
    m->d_color.argb[i].r = (u8)0;  m->d_color.argb[i].b = (u8)0; m->d_color.argb[i].g = (u8)0; m->d_color.argb[i].a = (u8)0;
    m->norm.x[i] = 0.0;   m->norm.y[i] = 0.0;   m->norm.z[i] = 0.0;
    m->pnt.x[i] = 0.0;    m->pnt.y[i] = 0.0;    m->pnt.z[i] = 0.0;


    m->rmd.x[i] = n.x + u.x * i_x + v.x * i_y;    m->rmd.y[i] = n.y + u.y * i_x + v.y * i_y;    m->rmd.z[i] = n.z + u.z * i_x + v.z * i_y;
    device_normalize_vector(&m->rmd.x[i], &m->rmd.y[i], &m->rmd.z[i]);

    m->inv_rmd.x[i] = 1/m->rmd.x[i];    m->inv_rmd.y[i] = 1/ m->rmd.y[i];    m->inv_rmd.z[i] = 1/ m->rmd.z[i];
    m->sign_rmd.x[i] = ((T_uint*)m->rmd.x)[i] >> precision_shift;    m->sign_rmd.y[i] = ((T_uint*)m->rmd.y)[i] >> precision_shift;    m->sign_rmd.z[i] = ((T_uint*)m->rmd.z)[i] >> precision_shift;

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
    init_cam_mem_cuda << < 1+((u32)c->f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >>> (c->d_mem, c->f_prop.res.w, c->f_prop.res.h, c->r_prop.draw_distance,
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
__global__ void init_cam_voxel_mem_cuda(Camera::voxel_memory* vm, Trixel::kd_tree* kdm, VEC4_CUDA<VEC4_CUDA<T_fp>>* _rotM, VEC4_CUDA<T_fp>* _vec, VEC3<T_fp> co, u64 max_threads) {

    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    int cut_dir = kdm->d_nodes[i].cut_flag;
    vm->d_Bo[i].t0x = kdm->d_nodes[i].h_bound.x0 - co.x + vm->obj_center.x;
    vm->d_Bo[i].t1x = kdm->d_nodes[i].h_bound.x1 - co.x + vm->obj_center.x;
    vm->d_Bo[i].t0y = kdm->d_nodes[i].h_bound.y0 - co.y + vm->obj_center.y;
    vm->d_Bo[i].t1y = kdm->d_nodes[i].h_bound.y1 - co.y + vm->obj_center.y;
    vm->d_Bo[i].t0z = kdm->d_nodes[i].h_bound.z0 - co.z + vm->obj_center.z;
    vm->d_Bo[i].t1z = kdm->d_nodes[i].h_bound.z1 - co.z + vm->obj_center.z;
    vm->is_leaf[i] = kdm->d_nodes[i].is_leaf;
    vm->children[i].left = kdm->d_nodes[i].left_node;
    vm->children[i].right= kdm->d_nodes[i].right_node;
    vm->children[i].parent = kdm->d_nodes[i].parent;

    vm->children[i].triangle = vm->is_leaf[i] == 0 ? -1 : kdm->d_nodes[i].tri_index;

    vm->cut_flags[i].x = cut_dir == 0 || cut_dir == 3 ? 1 : 0;
    vm->cut_flags[i].y = cut_dir == 1 || cut_dir == 4 ? 1 : 0;
    vm->cut_flags[i].z = cut_dir == 2 || cut_dir == 5 ? 1 : 0;

    vm->d_q->d_rot_m = _rotM;
    vm->d_q->d_vec = _vec;

    vm->s1[i] = kdm->d_nodes[i].s1 - (((co.x + vm->obj_center.x) * (T_fp)vm->cut_flags[i].x) + ((co.y + vm->obj_center.y) * (T_fp)vm->cut_flags[i].y) + ((co.z + vm->obj_center.z) * (T_fp)vm->cut_flags[i].z));
    vm->s2[i] = kdm->d_nodes[i].s2 - (((co.x + vm->obj_center.x) * (T_fp)vm->cut_flags[i].x) + ((co.y + vm->obj_center.y) * (T_fp)vm->cut_flags[i].y) + ((co.z + vm->obj_center.x) * (T_fp)vm->cut_flags[i].z));

}
cudaError_t init_camera_voxel_device_memory(Trixel* t, Camera* c) {
    cudaError_t cudaStatus;
    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        printf("cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    }
    init_cam_voxel_mem_cuda << < 1 + (u32)(t->num_voxels / BLOCK_SIZE), BLOCK_SIZE >> > (
        c->d_voxels,
        (Trixel::kd_tree*)t->d_tree, c->h_voxels.q->d_rot_m, c->h_voxels.q->d_vec,
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
__global__ void update_voxel_transform_m_translate_cuda(Camera::voxel_memory* dvm, u64 max_threads) {
    u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (voxel_index >= max_threads) { return; }
    VEC4_CUDA<VEC4_CUDA<T_fp>>* cur_rot = dvm->d_q->d_rot_m;

    cur_rot->x->w[voxel_index] += dvm->cur_transform.x * dvm->cur_transform.d;
    cur_rot->y->w[voxel_index] += dvm->cur_transform.y * dvm->cur_transform.d;
    cur_rot->z->w[voxel_index] += dvm->cur_transform.z * dvm->cur_transform.d;

}

__global__ void rotate_cam_voxel_mem_cuda(Camera::voxel_memory* dvm, VEC4_CUDA<T_fp>* new_vec, u64 max_threads) {
    u64 voxel_index = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (voxel_index >= max_threads) { return; }
    VEC4_CUDA<T_fp>* cur_vec = dvm->d_q->d_vec;
    VEC4_CUDA<VEC4_CUDA<T_fp>>* cur_rot = dvm->d_q->d_rot_m;

    if (new_vec) { quaternion_mul(cur_vec, new_vec, voxel_index); }

    cur_rot->x->i[voxel_index] = (1 - 2 * cur_vec->j[voxel_index] * cur_vec->j[voxel_index] - 2 * cur_vec->k[voxel_index] * cur_vec->k[voxel_index]);
    cur_rot->x->j[voxel_index] = (2 * cur_vec->i[voxel_index] * cur_vec->j[voxel_index] - 2 * cur_vec->k[voxel_index] * cur_vec->w[voxel_index]);
    cur_rot->x->k[voxel_index] = (2 * cur_vec->i[voxel_index] * cur_vec->k[voxel_index] + 2 * cur_vec->j[voxel_index] * cur_vec->w[voxel_index]);

    cur_rot->y->i[voxel_index] = (2 * cur_vec->i[voxel_index] * cur_vec->j[voxel_index] + 2 * cur_vec->k[voxel_index] * cur_vec->w[voxel_index]);
    cur_rot->y->j[voxel_index] = (1 - 2 * cur_vec->i[voxel_index] * cur_vec->i[voxel_index] - 2 * cur_vec->k[voxel_index] * cur_vec->k[voxel_index]);
    cur_rot->y->k[voxel_index] = (2 * cur_vec->j[voxel_index] * cur_vec->k[voxel_index] - 2 * cur_vec->i[voxel_index] * cur_vec->w[voxel_index]);

    cur_rot->z->i[voxel_index] = (2 * cur_vec->i[voxel_index] * cur_vec->k[voxel_index] - 2 * cur_vec->j[voxel_index] * cur_vec->w[voxel_index]);
    cur_rot->z->j[voxel_index] = (2 * cur_vec->j[voxel_index] * cur_vec->k[voxel_index] + 2 * cur_vec->i[voxel_index] * cur_vec->w[voxel_index]);
    cur_rot->z->k[voxel_index] = (1 - 2 * cur_vec->i[voxel_index] * cur_vec->i[voxel_index] - 2 * cur_vec->j[voxel_index] * cur_vec->j[voxel_index]);

    (dvm->cur_transform).rotate(cur_rot->x, cur_rot->y, cur_rot->z, voxel_index);
}

__global__ void update_trixels_device_cuda(Trixel::trixel_memory* tm, Camera::trixel_memory* cm, u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    device_cross(&cm->d_q.x[i], &cm->d_q.y[i], &cm->d_q.z[i], cm->d_t.x[i], cm->d_t.y[i], cm->d_t.z[i], tm->d_edges.e1.x[i], tm->d_edges.e1.y[i], tm->d_edges.e1.z[i]);
    cm->d_w[i] = device_dot(cm->d_q.x[i], cm->d_q.y[i], cm->d_q.z[i], tm->d_edges.e2.x[i], tm->d_edges.e2.y[i], tm->d_edges.e2.z[i]);
}

//Translate in the direction the object is "facing" by the amount (d) VEC4 (x,y,z,d)
__global__ void translate_cam_voxel_mem_cuda(Camera::voxel_memory* vm, Camera::trixel_memory* cm, bool reverse,u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }

    //THESE VALUES ARE ALL CALCULATED AS SOME POINT IN SPACE minus THE CAMERA ORIGIN
    //So if you move the camera by  + dx...need to subtract dx from the voxel
    T_fp _dx = (vm->cur_transform.d * vm->cur_transform.x);
    T_fp _dy = (vm->cur_transform.d * vm->cur_transform.y);
    T_fp _dz = (vm->cur_transform.d * vm->cur_transform.z);

    if (i == 0) { printf("Translate: %f %f %f\n", _dx, _dy, _dz); }

    _dx = reverse ? -_dx : _dx;
    _dy = reverse ? -_dy : _dy;
    _dz = reverse ? -_dz : _dz;

    vm->d_Bo[i].t0x += _dx;    vm->d_Bo[i].t1x += _dx;
    vm->d_Bo[i].t0y += _dy;    vm->d_Bo[i].t1y += _dy;
    vm->d_Bo[i].t0z += _dz;    vm->d_Bo[i].t1z += _dz;

    vm->s1[i] += (_dx * (T_fp)vm->cut_flags[i].x) + (_dy * (T_fp)vm->cut_flags[i].y) + (_dz * (T_fp)vm->cut_flags[i].z);
    vm->s2[i] += (_dx * (T_fp)vm->cut_flags[i].x) + (_dy * (T_fp)vm->cut_flags[i].y) + (_dz * (T_fp)vm->cut_flags[i].z);
    if (vm->is_leaf[i]) {
        T_uint tri_i = vm->children[i].triangle;
        cm->d_t.x[tri_i] -= _dx; cm->d_t.y[tri_i] -= _dy;	cm->d_t.z[tri_i] -= _dz;
    }
}

cudaError_t transform_camera_voxel_device_memory(Camera* c, Trixel* t, VEC4<T_fp>* tv, Quaternion* q, u8 transform_select) {
    cudaError_t cudaStatus;
    switch (transform_select) {
    case TRANSLATE_XYZ:
    case TRANSLATE_X:
    case TRANSLATE_Z:
        printf("N : %f %f %f %f\n", c->h_voxels.n.x, c->h_voxels.n.y, c->h_voxels.n.z, c->h_voxels.n.w);
        printf("N : %f %f %f %f\n", c->h_voxels.init_face.x, c->h_voxels.init_face.y, c->h_voxels.init_face.z, c->h_voxels.init_face.w);
        cudaMemcpy(&c->h_voxels.cur_transform, &-*tv, sizeof(VEC4<T_fp>), cudaMemcpyHostToHost);
        cudaMemcpy(c->d_voxels, &c->h_voxels, sizeof(Camera::voxel_memory), cudaMemcpyHostToDevice);
        rotate_cam_voxel_mem_cuda << < 1, 1 >> > (
            c->d_voxels, NULL, 1);
        translate_cam_voxel_mem_cuda << < 1 + (u32)(c->h_voxels.num_voxels / BLOCK_SIZE), BLOCK_SIZE >> > (
            c->d_voxels, c->d_trixels, false, c->h_voxels.num_voxels);
        update_trixels_device_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > (
            (Trixel::trixel_memory*)t->d_mem, c->d_trixels, t->num_trixels);

        //c->h_voxels.n -= c->h_voxels.cur_transform;
        //normalize_Vector(&c->h_voxels.n);
        c->h_voxels.init_face += c->h_voxels.cur_transform;
        normalize_Vector(&c->h_voxels.init_face);
        cudaMemcpy(&c->h_voxels.cur_transform, &-c->h_voxels.init_face, sizeof(VEC4<T_fp>), cudaMemcpyHostToHost);
        cudaMemcpy(c->d_voxels, &c->h_voxels, sizeof(Camera::voxel_memory), cudaMemcpyHostToDevice);
        rotate_cam_voxel_mem_cuda << < 1, 1 >> > (
         c->d_voxels, NULL, 1);
        cudaMemcpy(&c->h_voxels, c->d_voxels, sizeof(Camera::voxel_memory), cudaMemcpyDeviceToHost);
        c->h_voxels.n = -c->h_voxels.cur_transform;

        printf("N : %f %f %f %f\n", c->h_voxels.n.x, c->h_voxels.n.y, c->h_voxels.n.z, c->h_voxels.n.w);
        printf("N : %f %f %f %f\n", c->h_voxels.init_face.x, c->h_voxels.init_face.y, c->h_voxels.init_face.z, c->h_voxels.init_face.w);

        printf("\n");
        break;
    case ROTATE_TRI_PY:
    case ROTATE_TRI_NY:
        cudaMemcpy(c->h_voxels.next_rotation->vec->i, q->vec->i, sizeof(T_fp), cudaMemcpyHostToDevice);
        cudaMemcpy(c->h_voxels.next_rotation->vec->j, q->vec->j, sizeof(T_fp), cudaMemcpyHostToDevice);
        cudaMemcpy(c->h_voxels.next_rotation->vec->k, q->vec->k, sizeof(T_fp), cudaMemcpyHostToDevice);
        cudaMemcpy(c->h_voxels.next_rotation->vec->w, q->vec->w, sizeof(T_fp), cudaMemcpyHostToDevice);

        cudaMemcpy(&c->h_voxels.cur_transform, &-c->h_voxels.init_face, sizeof(VEC4<T_fp>), cudaMemcpyHostToHost);
        cudaMemcpy(c->d_voxels, &c->h_voxels, sizeof(Camera::voxel_memory), cudaMemcpyHostToDevice);
        rotate_cam_voxel_mem_cuda << < 1, 1 >> > (
            c->d_voxels, c->h_voxels.next_rotation->d_vec, 1);
        cudaMemcpy(&c->h_voxels, c->d_voxels, sizeof(Camera::voxel_memory), cudaMemcpyDeviceToHost);

        c->h_voxels.cur_transform += c->h_voxels.n;
        //c->h_voxels.cur_transform = -c->h_voxels.cur_transform;


        cudaMemcpy(c->d_voxels, &c->h_voxels, sizeof(Camera::voxel_memory), cudaMemcpyHostToDevice);
        translate_cam_voxel_mem_cuda << < 1 + (u32)(c->h_voxels.num_voxels / BLOCK_SIZE), BLOCK_SIZE >> > (
            c->d_voxels, c->d_trixels, true, c->h_voxels.num_voxels);
        update_trixels_device_cuda << < 1 + (u32)(t->num_trixels / BLOCK_SIZE), BLOCK_SIZE >> > (
            (Trixel::trixel_memory*)t->d_mem, c->d_trixels, t->num_trixels);
        c->h_voxels.cur_transform -= c->h_voxels.n;

        normalize_Vector(&c->h_voxels.cur_transform);

        cudaMemcpy(&c->h_voxels.n, &-c->h_voxels.cur_transform, sizeof(VEC4<T_fp>), cudaMemcpyHostToHost);

        printf("N : %f %f %f %f\n", c->h_voxels.n.x, c->h_voxels.n.y, c->h_voxels.n.z, c->h_voxels.n.w);
        printf("N : %f %f %f %f\n", c->h_voxels.init_face.x, c->h_voxels.init_face.y, c->h_voxels.init_face.z, c->h_voxels.init_face.w);

        break;
    }


    status_lauch_and_sync(translate_cam_voxel_mem_cuda);
    return cudaStatus;
}

__global__ void rotate_cam_mem_cuda(Camera::pixel_memory* m, VEC4_CUDA<VEC4_CUDA<T_fp>>* rot_m, int offset, u64 max_threads) {
    u64 i = (u64)threadIdx.x + ((u64)blockIdx.x * blockDim.x);
    if (i >= max_threads) { return; }
    T_fp temp_x = m->rmd.x[i];	T_fp temp_y = m->rmd.y[i];	T_fp temp_z = m->rmd.z[i];

   // m->rmd.x[i] = rot_m[0 + offset] * temp_x + rot_m[1 + offset] * temp_y + rot_m[2 + offset] * temp_z;
   // m->rmd.y[i] = rot_m[3 + offset] * temp_x + rot_m[4 + offset] * temp_y + rot_m[5 + offset] * temp_z;
  //  m->rmd.z[i] = rot_m[6 + offset] * temp_x + rot_m[7 + offset] * temp_y + rot_m[8 + offset] * temp_z;


    m->inv_rmd.x[i] = 1 / m->rmd.x[i];    m->inv_rmd.y[i] = 1 / m->rmd.y[i];    m->inv_rmd.z[i] = 1 / m->rmd.z[i];
    m->sign_rmd.x[i] = ((T_uint*)m->rmd.x)[i] >> precision_shift;
    m->sign_rmd.y[i] = ((T_uint*)m->rmd.y)[i] >> precision_shift; 
    m->sign_rmd.z[i] = ((T_uint*)m->rmd.z)[i] >> precision_shift;
}
cudaError_t Camera::rotate(Quaternion* q, int offset)
{
    cudaError_t cudaStatus;
    o_prop.rotation_help_array[0] = &o_prop.n;
    o_prop.rotation_help_array[1] = &o_prop.u;
    o_prop.rotation_help_array[2] = &o_prop.v;
    T_fp temp_x , temp_y ,temp_z;
    for (int i = 0; i < 3; i++) {
        temp_x = o_prop.rotation_help_array[i]->x;	 temp_y = o_prop.rotation_help_array[i]->y;	 temp_z = o_prop.rotation_help_array[i]->z;

        //o_prop.rotation_help_array[i]->x = q->rot_m[0 + offset] * temp_x + q->rot_m[1 + offset] * temp_y + q->rot_m[2 + offset] * temp_z;
       // o_prop.rotation_help_array[i]->y = q->rot_m[3 + offset] * temp_x + q->rot_m[4 + offset] * temp_y + q->rot_m[5 + offset] * temp_z;
       // o_prop.rotation_help_array[i]->z = q->rot_m[6 + offset] * temp_x + q->rot_m[7 + offset] * temp_y + q->rot_m[8 + offset] * temp_z;
    }

    
    rotate_cam_mem_cuda << < 1 + ((u32)f_prop.res.count / BLOCK_SIZE), BLOCK_SIZE >> > (
        d_mem, q->rot_m, offset, f_prop.res.count);
    status_lauch_and_sync(rotate_cam_mem_cuda);

    return cudaStatus;
}

