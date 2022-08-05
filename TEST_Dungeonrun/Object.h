#pragma once
#include "Trixel.h"
#define TRIXEL_OBJECT_TAG 0
class Object
{
	T_uint object_id;
	u8 object_tag;
public:
	u8 getTag() { return object_tag; }
	Trixel* trixel_list;
	Quaternion* quat;
	Object(Trixel* x);
	Object(Trixel* x, Quaternion* q) :Object(x) { quat = new Quaternion(*q); }
};

