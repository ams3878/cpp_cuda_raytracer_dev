#include <fstream>
#include <sstream>
#include <string>
#include "platform_common.h"

void read_ply(const char* file_name, double** points_list, u64* num_tri, double** vertex_list, u64* num_vert) {
    std::ifstream input(file_name);
    std::string line;
    input >> *num_vert;
    input >> *num_tri;
    *vertex_list = (double*)malloc(sizeof(double) * *num_vert * 3);
    *points_list = (double*)malloc(sizeof(double) * *num_tri * 3 * 3);
    printf("File %s\t verts: %d, faces: %d\n", file_name, num_vert, *num_tri);
    double confidence, intesity;
    for (u64 i = 0; i < *num_vert*3;) {
        input >> (*vertex_list)[i++] >> (*vertex_list)[i++] >> (*vertex_list)[i++] >> confidence >> intesity;
    } 
    int c, p1, p2, p3;
    for (u64 i = 0; i < *num_tri * 9;) {
        input >> c >> p1 >> p2 >> p3;
        p1 *= 3; p2 *= 3; p3 *= 3;
        (*points_list)[i++] = (*vertex_list)[p1]; (*points_list)[i++] = (*vertex_list)[p1 + 1]; (*points_list)[i++] = (*vertex_list)[p1 + 2];
        (*points_list)[i++] = (*vertex_list)[p2]; (*points_list)[i++] = (*vertex_list)[p2 + 1]; (*points_list)[i++] = (*vertex_list)[p2 + 2];
        (*points_list)[i++] = (*vertex_list)[p3]; (*points_list)[i++] = (*vertex_list)[p3 + 1]; (*points_list)[i++] = (*vertex_list)[p3 + 2];
    }
}