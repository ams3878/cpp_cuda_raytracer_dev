#include <fstream>
#include <sstream>
#include <string>
#include "framework.h"


#define XYZ_MASK 0
#define XYZCI_MASK 1
void read_ply(const char* file_name, T_fp** points_list, T_uint* num_tri, kd_leaf_sort** kd_leafs, kd_vertex** vertex_list, T_uint* num_vert, u8 mode) {
    std::ifstream input(file_name);
    std::string line;
    s64 triangle_index_offset = *num_tri;
    input >> *num_vert;
    input >> *num_tri;
    *vertex_list = (kd_vertex*)malloc(sizeof(kd_vertex) * *num_vert );
    *kd_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * *num_tri);

    struct point_vector { T_fp x; T_fp y; T_fp z; };
    *points_list = (T_fp*)malloc(sizeof(T_fp) * *num_tri * 3 * 3);
    T_fp confidence, intesity;
    for (T_luint i = 0; i < *num_vert; i++) {
        switch(mode){
        case XYZ_MASK:
            input >> (*vertex_list)[i].x >> (*vertex_list)[i].y >> (*vertex_list)[i].z;
            break;
        case XYZCI_MASK:
            input >> (*vertex_list)[i].x >> (*vertex_list)[i].y >> (*vertex_list)[i].z >> confidence >> intesity;
            break;
        }
    } 
    int c, p1, p2, p3;
    for (T_luint i = 0,leaf_index=0; i < (* num_tri) * 3;) {
        input >> c >> p1 >> p2 >> p3;
        if (c == 3) {
            (*kd_leafs)[leaf_index].x0 = min((*vertex_list)[p1].x, min((*vertex_list)[p2].x, (*vertex_list)[p3].x));
            (*kd_leafs)[leaf_index].x1 = max((*vertex_list)[p1].x, max((*vertex_list)[p2].x, (*vertex_list)[p3].x));

            (*kd_leafs)[leaf_index].y0 = min((*vertex_list)[p1].y, min((*vertex_list)[p2].y, (*vertex_list)[p3].y));
            (*kd_leafs)[leaf_index].y1 = max((*vertex_list)[p1].y, max((*vertex_list)[p2].y, (*vertex_list)[p3].y));

            (*kd_leafs)[leaf_index].z0 = min((*vertex_list)[p1].z, min((*vertex_list)[p2].z, (*vertex_list)[p3].z));
            (*kd_leafs)[leaf_index].z1 = max((*vertex_list)[p1].z, max((*vertex_list)[p2].z, (*vertex_list)[p3].z));
            (*kd_leafs)[leaf_index].tri_list_index = triangle_index_offset + leaf_index++;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p1].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p1].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p1].z;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p2].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p2].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p2].z;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p3].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p3].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p3].z;
        }
        else { (*num_tri)--; }//Maybe something in the list wasnt a triangle
    }
    free(*vertex_list);
}