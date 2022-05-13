#pragma once
#ifndef VECTOR_H
  #define VECTOR_H
  #include "platform_common.h"
  struct Vector {
    //This struct is used for points, rays, and true  vectors
    // Which it is will be determined by how it is intializd/read
    u8 space : 4; //The number of dimensions typical 3
    u8 type : 2;//0-Ray, 1-Vector, 2-point
    float* v; // The vector
    float* p; // First point\origin
    float* q; // Second point\dest
    float l2; //euclidean distance
    float* nv; //normazlied vector  
  };
  struct default_Vector{
    float* p;
    float* q;
    float* v;
    float* nv;
  };
  struct default_Ray {
    float* o;
    float* d;
  };
#define default_Point float
#define short_Vector float//use this to save memory. Assume default-space. Assume origin(0,0,0). Direction Vector/Ray only
#define DEFAULT_SPACE 3


  template <typename T>  T* normalize_Vector(T vx, T vy, T vz);
#define _normalize_Vector(v) normalize_Vector(v[0],v[1],v[2]);
  template <typename T> T dot_Vector(T* vec_a, T* vec_b);
  template <typename T> int cross_Vector(T* vec_a, T* vec_b);
  template <typename T> T vector_norm(T* vec);
  template <typename T> T vector_normsq(T* vec);


 
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
      int count = 10;
      while (count-- > 0) {
        u.x = u.x * (1.5f - invrs_sqrt_half * u.x * u.x);
      }
    }
    T* temp = (T*)malloc(sizeof(T)*3);
    temp[0] = vx * u.x;
    temp[1] = vy * u.x;
    temp[2] = vz * u.x;

    return temp;
  }



  template <typename T>
  T dot_Vector(T* vec_a, T* vec_b) {//i assume if used right away this is fine. but if problems happen with stack variable change to reference
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
    for (int i = 0; i < DEFAULT_SPACE; i++, temp_v++) { rv_normsq += *temp_v * *temp_v; }
    return rv_normsq;
  }

#endif