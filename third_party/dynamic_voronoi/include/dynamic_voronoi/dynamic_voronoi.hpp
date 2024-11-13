#ifndef _DYNAMIC_VORONOI_H_
#define _DYNAMIC_VORONOI_H_

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include <memory>

#include <math.h>
#include <iostream>

#include <dynamic_voronoi/bucketed_queue.hpp>

namespace dynamic_voronoi{

struct DynamicVoronoiParams{
  double res{0.1};

  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};

  int origin_z_cm{0};

  double z_sep_cm{0.0};
}; // struct DynamicVoronoiParams


//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

public:
  
  DynamicVoronoi();

  ~DynamicVoronoi();

  // Set parameters
  void setParams(const DynamicVoronoiParams& params);

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, const std::vector<bool>& bool_map_1d_arr) ;

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true) ;
  //! prune the Voronoi diagram
  void prune() ;
  //! prune the Voronoi diagram by globally revisiting all Voronoi nodes. 
  // Takes more time but gives a more sparsely pruned Voronoi graph. 
  // You need to call this after every call to update()
  void updateAlternativePrunedDiagram();

  //! returns whether the specified cell is part of the alternatively pruned diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative( const int& x, const int& y ) const;

  //! check if cell is a voronoi vertex (has at least 3 voronoi neighbours)
  bool isVoronoiVertex(int x, int y) const;
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(const int& x, const int& y ) const;
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");

  /* Getter methods */

  //! retrieve the number of neighbors that are Voronoi cells (4-connected)
  int getNumVoronoiNeighbors(int x, int y) const;

  //! retrieve the number of neighbors that are Voronoi cells (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y);

  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );

  //! returns the squared obstacle distance at the specified location
  int getSqrDistToObs( int x, int y ) {
    if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)){
      return data_[x][y].sqdist; 
    } 
    else {
      return -1;
    }
  }

private:  
  struct dataCell {
    float dist; // Position to nearest obstacle
    int sqdist; // Position to nearest obstacle
    char voronoi;   // voronoi status
    char queueing;  
    int obstX;  // Position of nearest obstacle
    int obstY;  // Position of nearest obstacle
    bool needsRaise;
  };

  typedef std::shared_ptr<dataCell[]> dataCellArr;

  typedef enum {
    voronoiKeep=-4, 
    freeQueued = -3, 
    voronoiRetry=-2, 
    voronoiPrune=-1, 
    free=0, 
    occupied=1} State;
  typedef enum {
    fwNotQueued=1, 
    fwQueued=2, 
    fwProcessed=3, 
    bwQueued=4, 
    bwProcessed=1} QueueingState;
  typedef enum {
    invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {
    pruned, 
    keep, 
    retry} 
    markerMatchResult;

  // methods
  void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist=true);
  void reviveVoroNeighbors(int &x, int &y);

  bool isOccupied(int &x, int &y, dataCell &c);
  markerMatchResult markerMatch(int x, int y);
  bool markerMatchAlternative(int x, int y);
  int getVoronoiPruneValence(int x, int y);

/* Exposed methods used to interface with external planners */
public:

  /* Mapping methods */

  // Set obstacle at position (x,y)
  void setObstacle(int x, int y);
  // Remove obstacle at position (x,y)
  void removeObstacle(int x, int y);

  /* Checking methods */
  // If cell is in map
  bool isInMap(int x, int y) const;
  //! checks whether the specficied location is occupied
  bool isOccupied(const IntPoint& grid_pos) const;
  //! checks whether the location is occupied
  bool isOccupied(const size_t& x, const size_t& y) const ;

  // Convert from position to index
  bool posToIdx(const DblPoint& map_pos, IntPoint& grid_pos);

  // Convert from position to index
  void idxToPos(const IntPoint& grid_pos, DblPoint& map_pos);

  // // Get x origin of map
  // double getOriginX() const {
  //   return params_.origin_x;
  // }

  // // Get y origin of map
  // double getOriginY() const {
  //   return params_.origin_y;
  // }

  // Get z origin of map
  double getOriginZ() const {
    return params_.origin_z;
  }

  // Get map resolution
  double getRes() const {
    return params_.res;
  }

  //! returns the horizontal size of the workspace/map
  int getSizeX() const {
    return sizeX;
  }
  //! returns the vertical size of the workspace/map
  int getSizeY() const{
    return sizeY;
  }

  // std::unique_ptr<dataCellArr[]> getData() const {
  //   return data_;
  // }

  //! retrieve the alternatively pruned diagram. see updateAlternativePrunedDiagram()
  int** alternativePrunedDiagram() {
    return alternativeDiagram;
  };

public:
  DynamicVoronoiParams params_;

private:

  // queues
  BucketPrioQueue<IntPoint> open_;
  std::queue<IntPoint> pruneQueue;
  BucketPrioQueue<IntPoint> sortedPruneQueue;

  std::vector<IntPoint> removeList;
  std::vector<IntPoint> addList;
  std::vector<IntPoint> lastObstacles;

  // maps
  // bool flip_y_{false}; // Whether to flip the input map about y-axis
  int sizeY;  // Size of map along y
  int sizeX;  // Size of map along x
  std::shared_ptr<dataCellArr[]> data_{nullptr};

  int** alternativeDiagram{NULL};

}; // end class DynamicVoronoi

} // namespace dynamic_voronoi

#endif // _DYNAMIC_VORONOI_H_

