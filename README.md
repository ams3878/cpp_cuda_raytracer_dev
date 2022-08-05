# cpp_cuda_raytracer_dev
Render / Animation Engine

Functionality / Algorithms:

  - Triangle Intersect : moller-trumbore
  - Spacial Hieararchy : KDtree
    - nlogn build 
    - root(n) avg voxel checks per ray
    
 - BRDF : The most basic phong you could imagine
 - Animation : Rotation or Translation or Translation Then Rotation. but not rotation first(who knows...)

Metrics
  - Standfor Dragon (800k) ~ 150 FPS
