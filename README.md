# cpp_cuda_raytracer_dev
Render / Animation Engine

Functionality / Algorithms:

  - Triangle Intersect : moller-trumbore
  - Spacial Hieararchy : KDtree
    - nlogn build 
    - root(n) avg voxel checks per ray
    
 - BRDF : Not implemented

Metrics
  - Standfor Dragon (800k) ~ 150 FPS
