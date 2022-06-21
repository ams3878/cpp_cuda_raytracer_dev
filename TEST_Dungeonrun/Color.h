#pragma once
#include "typedefs.h"

class Color
{
public:
	union { struct { u8 b = 3; u8 g = 3; u8 r = 6; u8 a = 0; }*argb; u32* c; };
	struct radiance { T_fp r, g, b; } *rad;

	Color();
	Color(u64 size);

};

