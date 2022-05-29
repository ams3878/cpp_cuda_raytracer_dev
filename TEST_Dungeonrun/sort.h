#pragma once
template <typename T> int merge_sort(T* old_list, T* new_list, s64 size, u8 component_select);
#define SORT_NO_TAG 0
#define SORT_X0_TAG 1
#define SORT_Y0_TAG 2
#define SORT_Z0_TAG 3
#define SORT_X1_TAG 4
#define SORT_Y1_TAG 5
#define SORT_Z1_TAG 6

template <typename T> int merge_sort(T* old_list, T* new_list, s64 size, u8 component_select) {
	merge_recurse(old_list, 0, size - 1, new_list, component_select);
	return 0;
}

template <typename T> void merge_recurse(T* r_list, s64 l, s64 r, T* w_list, u8 component_select) {
	if (l >= r) { return; }
	u64 m = l + (r - l) / 2;
	merge_recurse(r_list, l, m, w_list, component_select);
	merge_recurse(r_list, m+1, r, w_list, component_select);
	_merge(r_list, l, m, r, w_list, component_select);
	return;
}

template <typename T> void _merge(T* r_list, s64 l, s64 m, s64 r, T* w_list, u8 component_select) {
	u64 i = l;
	u64 size = r - l + 1;
	u64 start = l;
	u64 m_start = m;
	bool comparator = false;
	while(l <= m_start && r > m){
		//**TODO** make comparitor passed as funciton to generalize		
		switch (component_select) {
		case SORT_X0_TAG:
			comparator = r_list[l].x0 < r_list[m + 1].x0;
			break;
		case SORT_Y0_TAG:
			comparator = r_list[l].y0 < r_list[m + 1].y0;
			break;
		case SORT_Z0_TAG:
			comparator = r_list[l].z0 < r_list[m + 1].z0;
			break;
		case SORT_X1_TAG:
			comparator = r_list[l].x1 < r_list[m + 1].x1;
			break;
		case SORT_Y1_TAG:
			comparator = r_list[l].y1 < r_list[m + 1].y1;
			break;
		case SORT_Z1_TAG:
			comparator = r_list[l].z1 < r_list[m + 1].z1;
			break;
		}
		w_list[i++] = (comparator) ? r_list[l++] : r_list[++m];

	}
	while (l <= m_start) { w_list[i++] = r_list[l++]; }
	while (r > m) { w_list[i++] = r_list[++m]; }

	cudaMemcpy(r_list + start, w_list + start, size * sizeof(T), cudaMemcpyHostToHost);
}
