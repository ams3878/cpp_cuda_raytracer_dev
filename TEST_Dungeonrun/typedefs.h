#pragma once
typedef char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;

#define PPP_TAG 0
#define NULL 0

#if PPP_TAG == 0
typedef float T_fp;
typedef u32 T_uint;
typedef u64 T_luint;
#define precision_shift 31
#define HOST_EPSILON 1e-15
#elif PPP_TAG == 1
//If using this, there will be "potential" buffer over runs because if you read in max of u64 primitives then the space required is 9 times that
//Seeing as there is likely no machine on earth that has that much memory you shold be safe, also i dont think you could read that many primitives in
//one at a time in yoru lifetime? could be wrong thoiugh.
typedef double T_fp;
typedef u64 T_uint;
typedef u64 T_luint;
#define precision_shift 63
#define HOST_EPSILON 1e-15
#endif