#pragma once
#ifndef VECTOR_H
  #define VECTOR_H
  #include "platform_common.h"
#define HOST_EPSILON_SINGLE 1e-30
#define VECTOR_NO_TAG 0
#define VECTOR_X_TAG 1
#define VECTOR_Y_TAG 2
#define VECTOR_Z_TAG 3

struct vector_xyz { double x; double y; double z; };

  template <typename T>  T* normalize_Vector(T vx, T vy, T vz);
#define _normalize_Vector(v) normalize_Vector(v[0],v[1],v[2]);
  template <typename T> T dot_Vector(T* vec_a, T* vec_b);
  template <typename T> int cross_Vector(T* vec_a, T* vec_b);
  template <typename T> T vector_norm(T* vec);
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
  T* normalize_Vector(T vx, T vy, T vz) {//both v and nv must already be intialized, if both same in place normalize
      union { T x; s64 i; } u;
      u.x = vx * vx + vy * vy + vz * vz;//this cant be negative
      if (u.x < EPSILON) { return NULL; } //not sure if i need to do anything, if errors change this
      else {
          T invrs_sqrt_half = 0.5f * u.x;
          u.x = invrs_sqrt_half;
          //Im just goin to use doubles for now
          //if (typeid(T) == typeid(float) { u.i = 0x5f375a86 - (u.i >> 1); }//single
          //if (typeid(T) == typeid(double) { u.i = 0x5FE6EB50C7B537A9 - (u.i >> 1); }//double
          u.i = 0x5FE6EB50C7B537A9 - (u.i >> 1);
          /* The next line can be repeated any number of times to increase accuracy */
          double old = u.x;
          u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
          while (old - u.x > HOST_EPSILON_SINGLE) {
              old = u.x;
              u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
          }
      }
      T* temp = (T*)malloc(sizeof(T) * 3);
      temp[0] = vx * u.x;
      temp[1] = vy * u.x;
      temp[2] = vz * u.x;

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
  T vector_norm(T* vec) {
      T invrs_sqrt = vector_normsq(vec);
      T invrs_sqrt_half = 0.5 * invrs_sqrt;
      union { T x; int i; } u;
      u.x = invrs_sqrt_half;
      u.i = 0x5f375a86 - (u.i >> 1);
      /* The next line can be repeated any number of times to increase accuracy */
      u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
      invrs_sqrt = u.x;
      return 1 / invrs_sqrt;
  }
  template <typename T>
  T vector_normsq(T* vec) {
      T rv_normsq = 0.0;  T* temp_v = vec;
      for (int i = 0; i < 3; i++, temp_v++) { rv_normsq += *temp_v * *temp_v; }
      return rv_normsq;
  }
#endif