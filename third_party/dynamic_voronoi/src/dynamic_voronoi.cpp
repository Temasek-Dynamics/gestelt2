#include "dynamic_voronoi/dynamic_voronoi.hpp"
namespace dynamic_voronoi{

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  alternativeDiagram = NULL;
  allocatedGridMap = false;
}

DynamicVoronoi::DynamicVoronoi(const DynamicVoronoiParams& params): params_(params)
{
  sqrt2 = sqrt(2.0);
  data = NULL;
  alternativeDiagram = NULL;
  allocatedGridMap = false; 

}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++){
      delete[] data[x];
    }
    delete[] data;
  }
  if(alternativeDiagram){
    for (int x=0; x<sizeX; x++) {
      delete[] alternativeDiagram[x];
    }
    delete[] alternativeDiagram;    
    alternativeDiagram = NULL;
  }

  // Release weak reference to top and bottom layers
  top_voro_.reset();
  bottom_voro_.reset();
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY) {
  if (data) {
    for (int x=0; x<sizeX; x++) {
      delete[] data[x];
    }
    delete[] data;
    data = NULL;
  }
  if(alternativeDiagram){
    for (int x=0; x<sizeX; x++) delete[] alternativeDiagram[x];
    delete[] alternativeDiagram;    
    alternativeDiagram = NULL;
  }

  sizeX = _sizeX;
  sizeY = _sizeY;
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) {
    data[x] = new dataCell[sizeY];
  }

  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      data[x][y] = c;
    }
  }

}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, 
                                    const std::vector<bool>& bool_map_1d_arr) {
  
  initializeEmpty(_sizeX, _sizeY);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (bool_map_1d_arr[x + y * _sizeX]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) {
              continue;
            }
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) {
                continue;
              }
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1){
                continue;
              }
              if (!bool_map_1d_arr[nx + ny * _sizeX]) { 
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } 
          else {
            setObstacle(x,y);
          }
        }
      }
    }
  }

}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(IntPoint(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(IntPoint(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    IntPoint p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, IntPoint(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, IntPoint(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, IntPoint(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) {
    return data[x][y].dist;
  } 
  else return -1;
}

bool DynamicVoronoi::isVoronoiVertex(int x, int y) {
  if (isVoronoi(x,y)){
    if (getNumVoronoiNeighbors(x, y) >= 3){
      return true;
    }
  }
  return false;
}

bool DynamicVoronoi::isVoronoi(const int& x, const int& y ) const {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}

bool DynamicVoronoi::isVoronoiAlternative(const int& x, const int& y ) const{
  int v = alternativeDiagram[x][y];
  return (v == free || v == voronoiKeep);
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    IntPoint p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, IntPoint(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    IntPoint p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, IntPoint(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}

void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(IntPoint(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(IntPoint(nx,ny));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(IntPoint(nx,ny));
      }
    }
  }
}


void DynamicVoronoi::visualize(const char *filename) {
  // write ppm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n#\n");
  fprintf(F, "%d %d\n255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (alternativeDiagram!=NULL && (alternativeDiagram[x][y] == free || alternativeDiagram[x][y]==voronoiKeep)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if(isVoronoi(x,y)){
        fputc( 0, F );
        fputc( 0, F );
        fputc( 255, F );
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(sqrt(data[x][y].sqdist)*10);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    IntPoint p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied){
      continue;
    }
    if (data[x][y].voronoi==freeQueued) 
    {
      continue;
    }

    data[x][y].voronoi = freeQueued;
    sortedPruneQueue.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        sortedPruneQueue.push(r.sqdist, IntPoint(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        sortedPruneQueue.push(l.sqdist, IntPoint(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        sortedPruneQueue.push(t.sqdist, IntPoint(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        sortedPruneQueue.push(b.sqdist, IntPoint(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!sortedPruneQueue.empty()) {
    IntPoint p = sortedPruneQueue.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (sortedPruneQueue.empty()) {
      while (!pruneQueue.empty()) {
        IntPoint p = pruneQueue.front();
        pruneQueue.pop();
        sortedPruneQueue.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

void DynamicVoronoi::updateAlternativePrunedDiagram() {

  if(alternativeDiagram==NULL){
    alternativeDiagram = new int*[sizeX];
    for(int x=0; x<sizeX; x++){
      alternativeDiagram[x] = new int[sizeY];
    }
  }


  std::queue<IntPoint> end_cells;
  BucketPrioQueue<IntPoint> sortedPruneQueue;
  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      dataCell& c = data[x][y];
      alternativeDiagram[x][y] = c.voronoi;
      if(c.voronoi <=free){
        sortedPruneQueue.push(c.sqdist, IntPoint(x,y));
        end_cells.push(IntPoint(x, y));
      }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
        alternativeDiagram[x][y] = voronoiKeep;
        sortedPruneQueue.push(data[x][y].sqdist, IntPoint(x,y));
        end_cells.push(IntPoint(x, y));
      }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
        alternativeDiagram[x][y] = voronoiKeep;
        sortedPruneQueue.push(data[x][y].sqdist, IntPoint(x,y));
        end_cells.push(IntPoint(x, y));
      }
    }
  }


  while (!sortedPruneQueue.empty()) {
    IntPoint p = sortedPruneQueue.pop();

    if (markerMatchAlternative(p.x, p.y)) {
      alternativeDiagram[p.x][p.y]=voronoiPrune;
    } else {
  alternativeDiagram[p.x][p.y]=voronoiKeep;
    }
  }

  // //delete worms
  while (!end_cells.empty()) {
    IntPoint p = end_cells.front();
    end_cells.pop();

    if (isVoronoiAlternative(p.x,p.y) && getNumVoronoiNeighborsAlternative(p.x, p.y) == 1) {
      alternativeDiagram[p.x][p.y] = voronoiPrune;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (!(dx || dy) || (dx && dy)) {
            continue;
          }
          int nx = p.x + dx;
          int ny = p.y + dy;
          if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
            continue;
          }
          if (isVoronoiAlternative(nx,ny)) {
            if (getNumVoronoiNeighborsAlternative(nx, ny) == 1) {
              end_cells.push(IntPoint(nx, ny));
            }
          }
        }
      }
    }
  }
}

bool DynamicVoronoi::markerMatchAlternative(int x, int y) {
// prune if this returns true

  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
//  int obstacleCount=0;
  int voroCount = 0;
  for (dy = 1; dy >= -1; dy--) {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++) {
      if (dx || dy) {
        nx = x + dx;
        int v = alternativeDiagram[nx][ny];
        bool b = (v <= free && v != voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (v <= free && !(dx && dy))
          voroCount++;
        i++;
      }
    }
  }

  /*
   * 5 6 7
   * 3   4
   * 0 1 2
   */

  {
    //connected horizontal or vertically to only one cell
    if (voroCount == 1 && (f[1] || f[3] || f[4] || f[6])) {
      return false;
    }

    // 4-connected
    if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
      return false;

    if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
      return false;

  }
  return true;
}


int DynamicVoronoi::getNumVoronoiNeighbors(int x, int y) {
  int count = 0;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) { // 4 Connected
        continue;
      }

      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
        continue;
      }
      if (data[nx][ny].voronoi==free || data[nx][ny].voronoi==voronoiKeep) {
        count++;
      }
    }
  }
  return count;
}

int DynamicVoronoi::getNumVoronoiNeighborsAlternative(int x, int y) {
  int count = 0;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) { // 4 Connected
        continue;
      }

      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
        continue;
      }
      if (alternativeDiagram[nx][ny]==free || alternativeDiagram[nx][ny]==voronoiKeep) {
        count++;
      }
    }
  }
  return count;
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;

  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}

// Convert from index to position
bool DynamicVoronoi::posToIdx(const DblPoint& map_pos, IntPoint& grid_pos) {
  
  grid_pos.x = (map_pos.x - params_.origin_x) / params_.res;

  if (flip_y_){
    grid_pos.y = sizeY - (map_pos.y - params_.origin_y) / params_.res;
  }
  else {
    grid_pos.y = (map_pos.y - params_.origin_y) / params_.res;
  }

  if (!isInMap(grid_pos.x, grid_pos.y)){
    std::cout << "[DynamicVoronoi::posToIdx] (" << grid_pos.x << "," << grid_pos.y << ") not in map" << std::endl;
    return false;
  }
  return true;
}

// Convert from position to index
void DynamicVoronoi::idxToPos(const IntPoint& grid_pos, DblPoint& map_pos) {
  map_pos.x = grid_pos.x * params_.res + params_.origin_x;
  if (flip_y_){
    map_pos.y = (- grid_pos.y + sizeY) * params_.res + params_.origin_y;
  }
  else {
    map_pos.y = grid_pos.y * params_.res + params_.origin_y;
  }
}

/* Checking methods */

bool DynamicVoronoi::isInMap(int x, int y) {
  return !(x < 0 || x >= sizeX || y < 0 || y >= sizeY);
}

bool DynamicVoronoi::isOccupied(const IntPoint& grid_pos) const {
  // std::cout << "DynamicVoronoi::isOccupied(" << grid_pos.x << ", " << grid_pos.y << ")" << std::endl;
  dataCell c = data[grid_pos.x][grid_pos.y];
  return (c.obstX==grid_pos.x && c.obstY==grid_pos.y);
}

bool DynamicVoronoi::isOccupied(const size_t& x, const size_t& y) const {
  // std::cout << "DynamicVoronoi::isOccupied(" << x << ", " << y << ")" << std::endl;
  dataCell c = data[x][y];
  return (c.obstX == (int) x && c.obstY == (int) y);
}

bool DynamicVoronoi::isOccupied(int x, int y) {
  if (!isInMap(x, y)){
    return true;
  }
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  if (!isInMap(x, y)){
    return true;
  }
  return (c.obstX==x && c.obstY==y);
}

} // namespace dynamic_voronoi
