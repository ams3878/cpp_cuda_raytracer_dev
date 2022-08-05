#include "framework.h"

Object::Object(Trixel* new_object) {
	object_tag = new_object->object_tag;
	if (object_tag == TRIXEL_OBJECT_TAG) {
		trixel_list = new_object;
	}
}
