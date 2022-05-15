#include "Vector.h"

int vector_log2(u64 value) {
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    value |= value >> 32;
    return tab64[((u64)((value - (value >> 1)) * 0x07EDD5E59A4E28C2)) >> 58];
}
