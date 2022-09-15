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
	VEC4<T_fp>* init_face, *cur_face; //trying to get rid of these. atm would need one per camera.
	Object(Trixel* x);
	Object(Trixel* x, Quaternion* q) :Object(x) { quat = new Quaternion(*q); }
	void transform(Input* dq, u8 transform_select);
	void render(Camera* c);
};

