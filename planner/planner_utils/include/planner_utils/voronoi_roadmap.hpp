#ifndef _VORONOI_ROADMAP_H_
#define _VORONOI_ROADMAP_H_

#include <dynamic_voronoi/dynamic_voronoi.hpp>

/* 3D voronoi roadmap*/
class VoronoiRoadmap {
public:
  VoronoiRoadmap() {}

  // void setParams(){

  // }

  // void addLayer(const int& z_cm, ){
  //   dyn_voro_map_[z_cm] = dynamic_voronoi::DynamicVoronoi();

  // }

  std::map<int, dynamic_voronoi::DynamicVoronoi> dyn_voro_map_; // array of voronoi objects with key of height (cm)
}; // class VoronoiRoadmap

#endif // _VORONOI_ROADMAP_H_
