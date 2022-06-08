#pragma once
#ifndef VECTOR_H
  #define VECTOR_H
  #include "platform_common.h"
#define HOST_EPSILON_SINGLE 1e-35
#define VECTOR_NO_TAG 0
#define VECTOR_X_TAG 1
#define VECTOR_Y_TAG 2
#define VECTOR_Z_TAG 3
#include <typeinfo>
//PPP_TAG pre proccesor precison tag. 0 for float 1 for double
#define PPP_TAG 0


#if PPP_TAG == 0
#define T_fp float
#define T_uint u32
#define precision_shift 31
#elif PPP_TAG == 1
#define T_fp double
#define T_uint u64
#define precision_shift 63
#endif

  template <typename T>  T* normalize_Vector(T vx, T vy, T vz);
#define _normalize_Vector(v) normalize_Vector<T_fp>(v[0],v[1],v[2]);
  template <typename T> T dot_Vector(T* vec_a, T* vec_b);
  template <typename T> int cross_Vector(T* vec_a, T* vec_b);
  template <typename T> T vector_normsq(T* vec);
  int vector_log2(u64 val);

  const int tab64[64] = {//log2 64 bit index list
63,  0, 58,  1, 59, 47, 53,  2,
60, 39, 48, 27, 54, 33, 42,  3,
61, 51, 37, 40, 49, 18, 28, 20,
55, 30, 34, 11, 43, 14, 22,  4,
62, 57, 46, 52, 38, 26, 32, 41,
50, 36, 17, 19, 29, 10, 13, 21,
56, 45, 25, 31, 35, 16,  9, 12,
44, 24, 15,  8, 23,  7,  6,  5 };

  template <typename T>
  struct VEC3_CUDA {
      union { T* x; T* r; T* i; }; union { T* y; T* g; T* j; }; union { T* z; T* b; T* k; };
  VEC3_CUDA() :x(), y(), z() {};
  VEC3_CUDA(s64 s) { 
      x = (T*)malloc(sizeof(T) * s); 
      y = (T*)malloc(sizeof(T) * s);
      z = (T*)malloc(sizeof(T) * s);  } 
  };

  template <typename T>
  struct VEC3 {
      union { T x; T r; T i; }; union { T y; T g; T j; }; union { T z; T b; T k; };
      VEC3() : x(), y(), z() {};
      VEC3(T _x, T _y, T _z) { x = _x; y = _y; z = _z; };
      template <typename T2>
      void rotate(T2 qx, T2 qy, T2 qz) {
          T temp_x = x, temp_y = y, temp_z = z;
          x = temp_x * qx.i + temp_y * qx.j + temp_z * qx.k;
          y = temp_x * qy.i + temp_y * qy.j + temp_z * qy.k;
          z = temp_x * qz.i + temp_y * qz.j + temp_z * qz.k;
      };
  };

  template <typename T>
  T vector_norm(T scalor) {
      T invrs_sqrt = scalor;
      T invrs_sqrt_half = 0.5 * invrs_sqrt;
      union { T x; s64 i; } u;
      u.x = invrs_sqrt_half;
     // u.i = typeid(T) == typeid(float) ? 0x5f375a86 - (u.i >> 1) : 0x5FE6EB50C7B537A9 - (u.i >> 1);
      u.i = 0x5f375a86 - (u.i >> 1);
      /* The next line can be repeated any number of times to increase accuracy */
      for (int i = 0; i < 8; i++) {
          u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
      }
      //invrs_sqrt = 
      return u.x;
  }
  template <typename T>
  T* normalize_Vector(T vx, T vy, T vz, T v4) {//both v and nv must already be intialized, if both same in place normalize
      T s = vx * vx + vy * vy + vz * vz + v4 * v4;//this cant be negative     
      s = vector_norm(s);
      T* temp = (T*)malloc(sizeof(T) * 4);
      temp[0] = vx * s;
      temp[1] = vy * s;
      temp[2] = vz * s;
      temp[3] = v4 * s;
      return temp;
  }
  template <typename T>
  T* normalize_Vector(T vx, T vy, T vz) {//both v and nv must already be intialized, if both same in place normalize
      T s = vx * vx + vy * vy + vz * vz ;//this cant be negative     
      s = vector_norm(s);
      T* temp = (T*)malloc(sizeof(T) * 3);
      temp[0] = vx * s;
      temp[1] = vy * s;
      temp[2] = vz * s;
      return temp;
  }

  template <typename T>
  T dot_Vector(T* vec_a, T* vec_b) {
      return vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1] + vec_a[2] * vec_b[2];
  }

  template <typename T>
  int cross_Vector(T* vec_a, T* vec_b) {//not defined if DEFAULT_SPACE is not 3
      T t0 = vec_a[1] * vec_b[2] - vec_a[2] * vec_b[1];
      T t1 = vec_a[2] * vec_b[0] - vec_a[0] * vec_b[2];
      T t2 = vec_a[0] * vec_b[1] - vec_a[1] * vec_b[0];
      vec_a[0] = t0; vec_a[1] = t1; vec_a[2] = t2;
      return 0;
  }

  template <typename T>
  T vector_normsq(T* vec) {
      T rv_normsq = 0.0;  T* temp_v = vec;
      for (int i = 0; i < 3; i++, temp_v++) { rv_normsq += *temp_v * *temp_v; }
      return rv_normsq;
  }
#endif