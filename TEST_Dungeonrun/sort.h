#pragma once
template <typename T> int merge_sort(T* old_list, T* new_list, u64 size, u8 component_select);
#define SORT_NO_TAG 0
#define SORT_X_TAG 1
#define SORT_Y_TAG 2
#define SORT_Z_TAG 3

/* oops started doing quick sort_save for later::
	u64 end_index = *(&old_list + 1) - old_list - 1;
	T median_value = old_list[0] < old_list[end_index / 2] ? old_list[0] > old_list[end_index] ? old_list[0]
		: old_list[end_index / 2] < old_list[end_index] ? old_list[end_index / 2] : old_list[end_index]
		: old_list[0] < old_list[end_index] ? old_list[0] : old_list[end_index / 2] > old_list[end_index] ? old_list[end_index / 2] : old_list[end_index];
*/
template <typename T> int merge_sort(T* old_list, T* new_list, u64 size, u8 component_select) {
	u64 swap_counter = -1;
	merge_recurse(old_list, 0, size - 1, new_list, component_select);
	return 0;
}

template <typename T> void merge_recurse(T* r_list, u64 l, u64 r, T* w_list, u8 component_select) {
	if (l >= r) { return; }
	u64 m = l + (r - l) / 2;
	merge_recurse(r_list, l, m, w_list, component_select);
	merge_recurse(r_list, m+1, r, w_list, component_select);
	_merge(r_list, l, m, r, w_list, component_select);
	return;
}

template <typename T> void _merge(T* r_list, u64 l, u64 m, u64 r, T* w_list, u8 component_select) {
	int i = l;
	int size = r - l + 1;
	int start = l;
	int m_start = m;
	while(l <= m_start && r > m){
		//**TODO** make comparitor passed as funciton to generalize
		if (((r_list[l].x < r_list[m+1].x) && component_select == SORT_X_TAG) ||\
			((r_list[l].y < r_list[m + 1].y) && component_select == SORT_Y_TAG) ||
			((r_list[l].z < r_list[m + 1].z) && component_select == SORT_Z_TAG)
			) { w_list[i++] = r_list[l++]; }
		else { w_list[i++] = r_list[++m]; }
	}
	while (l <= m_start) { w_list[i++] = r_list[l++]; }
	while (r > m) { w_list[i++] = r_list[++m]; }

	cudaMemcpy(r_list + start, w_list + start, size * sizeof(T), cudaMemcpyHostToHost);
}
