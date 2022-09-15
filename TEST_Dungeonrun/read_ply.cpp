#include <fstream>
#include <sstream>
#include <string>
#include "framework.h"


constexpr auto XYZ_MASK = 0;
constexpr auto XYZCI_MASK = 1;
constexpr auto XYZN_MASK = 2;
constexpr auto FORMAT_ASCII = 1;
constexpr auto FORMAT_BLE = 2;
//void add_point(kd_leaf_sort* kdl, VEC3<T_fp>* pl) {}
void read_ply(const char* file_name, T_fp** points_list, T_uint* num_tri, kd_leaf_sort** kd_leafs, kd_vertex** vertex_list, T_uint* num_vert, u8 mode) {
    std::ifstream input(file_name);
    std::string line, tag, name, val;
    std::stringstream ss_line, ss_tag, ss_name, ss_val;
    T_uint format_flag = 0;
    s64 triangle_index_offset = *num_tri;
    while (line != "end_header") {
        std::getline(input, line);
        ss_line = std::stringstream(line);
        std::getline(ss_line, tag, ' ');
        if (tag.compare("format") == 0) {
            std::getline(ss_line, name, ' ');
            if (name.compare("float") == 0) {
                format_flag = FORMAT_ASCII;
            }
            if (name.compare("binary_little_endian ") == 0) {
                format_flag = FORMAT_BLE;
            }
        }
        if(tag.compare("element") == 0) {
            std::getline(ss_line, name , ' ');
            if (name.compare("vertex") == 0) {
                std::getline(ss_line, val,' ');
                *num_vert = std::stoi(val);
            }
            if (name.compare("face") == 0) {
                std::getline(ss_line, val, ' ');
                *num_tri = std::stoi(val);
            }
        }
        continue;
    }
    *vertex_list = (kd_vertex*)malloc(sizeof(kd_vertex) * *num_vert );
    *kd_leafs = (kd_leaf_sort*)malloc(sizeof(kd_leaf_sort) * *num_tri * 2); //make extra space in case all are rectangles that need split

    struct point_vector { T_fp x; T_fp y; T_fp z; };
    *points_list = (T_fp*)malloc(sizeof(T_fp) * *num_tri * 3 * 3 * 2);//make extra space in case all are rectangles that need split
    T_fp confidence, intesity, arg3;
    for (T_luint i = 0; i < *num_vert; i++) {
        switch (mode) {
        case XYZ_MASK:
            input >> (*vertex_list)[i].x >> (*vertex_list)[i].y >> (*vertex_list)[i].z;
            break;
        case XYZCI_MASK:
            input >> (*vertex_list)[i].x >> (*vertex_list)[i].y >> (*vertex_list)[i].z >> confidence >> intesity;
            break;
        case XYZN_MASK:
            input >> (*vertex_list)[i].x >> (*vertex_list)[i].y >> (*vertex_list)[i].z >> confidence >> intesity >> arg3;
            break;
        case 3:
            input >> confidence;
            break;
        }
    } 
    int c, p1, p2, p3, p4;
    for (T_luint i = 0,leaf_index=0; i < (* num_tri) * 3;) {
        input >> c;
        if (c == 4) {
            input >> p1 >> p2 >> p3 >> p4;
            (*num_tri)++;
            VEC3<T_fp> A, B, C, D, t_C, t_D;
            A = VEC3<T_fp>((*vertex_list)[p1].x, (*vertex_list)[p1].y, (*vertex_list)[p1].z);
            B = VEC3<T_fp>((*vertex_list)[p2].x, (*vertex_list)[p2].y, (*vertex_list)[p2].z);
            C = VEC3<T_fp>((*vertex_list)[p3].x, (*vertex_list)[p3].y, (*vertex_list)[p3].z);
            D = VEC3<T_fp>((*vertex_list)[p4].x, (*vertex_list)[p4].y, (*vertex_list)[p4].z);  
            //if (((B - A).dot(C - B) < 0) || ((B - A).dot(D - B) < 0) ){ t_C = C; C = B; B = t_C; }
            //if (((B - A).dot(C - B) < 0) || ((B - A).dot(D - B) < 0)) { t_D = D; D = B; B = t_D; }
           // if ((B - A).dot(C - B) < (B - A).dot(D - B)){ t_D = D; D = C; C = t_D; }

            (*kd_leafs)[leaf_index].x0 = min(A.x, min(B.x, C.x));
            (*kd_leafs)[leaf_index].x1 = max(A.x, max(B.x, C.x));

            (*kd_leafs)[leaf_index].y0 = min(A.y, min(B.y, C.y));
            (*kd_leafs)[leaf_index].y1 = max(A.y, max(B.y, C.y));

            (*kd_leafs)[leaf_index].z0 = min(A.z, min(B.z, C.z));
            (*kd_leafs)[leaf_index].z1 = max(A.z, max(B.z, C.z));
            (*kd_leafs)[leaf_index].tri_list_index = triangle_index_offset + leaf_index++;

            ((point_vector*)(*points_list))[i].x = A.x;
            ((point_vector*)(*points_list))[i].y = A.y;
            ((point_vector*)(*points_list))[i++].z = A.z;

            ((point_vector*)(*points_list))[i].x = B.x;
            ((point_vector*)(*points_list))[i].y = B.y;
            ((point_vector*)(*points_list))[i++].z = B.z;

            ((point_vector*)(*points_list))[i].x = C.x;
            ((point_vector*)(*points_list))[i].y = C.y;
            ((point_vector*)(*points_list))[i++].z = C.z;

            (*kd_leafs)[leaf_index].x0 = min(A.x, min(D.x, C.x));
            (*kd_leafs)[leaf_index].x1 = max(A.x, max(D.x, C.x));

            (*kd_leafs)[leaf_index].y0 = min(A.y, min(D.y, C.y));
            (*kd_leafs)[leaf_index].y1 = max(A.y, max(D.y, C.y));

            (*kd_leafs)[leaf_index].z0 = min(A.z, min(D.z, C.z));
            (*kd_leafs)[leaf_index].z1 = max(A.z, max(D.z, C.z));
            (*kd_leafs)[leaf_index].tri_list_index = triangle_index_offset + leaf_index++;

            ((point_vector*)(*points_list))[i].x =A.x;
            ((point_vector*)(*points_list))[i].y = A.y;
            ((point_vector*)(*points_list))[i++].z = A.z;

            ((point_vector*)(*points_list))[i].x = C.x;
            ((point_vector*)(*points_list))[i].y = C.y;
            ((point_vector*)(*points_list))[i++].z = C.z;

            ((point_vector*)(*points_list))[i].x = D.x;
            ((point_vector*)(*points_list))[i].y = D.y;
            ((point_vector*)(*points_list))[i++].z = D.z;
        }

        if (c == 3) { input >> p1 >> p2 >> p3; 
            (*kd_leafs)[leaf_index].x0 = min((*vertex_list)[p1].x, min((*vertex_list)[p2].x, (*vertex_list)[p3].x));
            (*kd_leafs)[leaf_index].x1 = max((*vertex_list)[p1].x, max((*vertex_list)[p2].x, (*vertex_list)[p3].x));

            (*kd_leafs)[leaf_index].y0 = min((*vertex_list)[p1].y, min((*vertex_list)[p2].y, (*vertex_list)[p3].y));
            (*kd_leafs)[leaf_index].y1 = max((*vertex_list)[p1].y, max((*vertex_list)[p2].y, (*vertex_list)[p3].y));

            (*kd_leafs)[leaf_index].z0 = min((*vertex_list)[p1].z, min((*vertex_list)[p2].z, (*vertex_list)[p3].z));
            (*kd_leafs)[leaf_index].z1 = max((*vertex_list)[p1].z, max((*vertex_list)[p2].z, (*vertex_list)[p3].z));
            (*kd_leafs)[leaf_index].tri_list_index = triangle_index_offset + leaf_index++;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p3].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p3].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p3].z;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p1].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p1].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p1].z;

            ((point_vector*)(*points_list))[i].x = (*vertex_list)[p2].x;
            ((point_vector*)(*points_list))[i].y = (*vertex_list)[p2].y;
            ((point_vector*)(*points_list))[i++].z = (*vertex_list)[p2].z;
        }
    }
    free(*vertex_list);
}