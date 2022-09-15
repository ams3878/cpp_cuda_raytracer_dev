#include "framework.h"

Object::Object(Trixel* new_object) {
	object_tag = new_object->object_tag;
	if (object_tag == TRIXEL_OBJECT_TAG) {
		trixel_list = new_object;
	}
}

void Object::render(Camera* c) {
	trixel_list->intersect_trixels(c, quat, 0);
}

void Object::transform(Input* dq, u8 transform_select) {
	cudaError_t cuda_err = cudaPeekAtLastError();
	transform_camera_voxel_device_memory(this, dq->t_vec, dq->t_quat, transform_select);
}
