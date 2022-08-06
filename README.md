# cpp_cuda_raytracer_dev
Render / Animation Engine

Functionality / Algorithms:

  - Triangle Intersect : moller-trumbore
  - Spacial Hieararchy : KDtree
    - nlogn build 
    - root(n) avg voxel checks per ray
    
 - BRDF : The most basic phong you could imagine
 - Animation : 
   - Quaternion Rotation via keypress (current 6 directions) 
   - Linear / No interpolation since each frame is rendered in real time and incremented is fixed

Metrics
  - Standford Dragon (800k primitives) ~ 100 (25 at 90%+ pixel coverage) FPS  960 x 540


https://user-images.githubusercontent.com/22137697/179344041-96da1a67-7e1a-4c4a-8792-e9d53a0468eb.mp4
