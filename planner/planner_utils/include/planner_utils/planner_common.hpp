/****************************************************************************
 * MIT License
 *  
 *	Copyright (c) 2024 John Tan. All rights reserved.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 *
 ****************************************************************************/

#ifndef _PLANNER_COMMON_HPP_
#define _PLANNER_COMMON_HPP_

/**
 * @file planner_common.hpp
 * @author John Tan
 * @brief Common helper methods and data structures for planners
 * @version 0.1
 * @date 2024-09-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <limits>
#include <queue>

#include <Eigen/Eigen>

#include <planner_utils/int_point.hpp>

inline int SQRT2 = 1.4142135623;

/**
 * @brief Priority queue used for open list in A* search
 * 
 * @tparam T 
 * @tparam priority_t 
 */
template<typename T, typename priority_t>
struct PriorityQueue {
  using PQElement = std::pair<priority_t, T>;
  struct PQComp {
      constexpr bool operator()(
          PQElement const& a,
          PQElement const& b)
          const noexcept
      {
          return a.first > b.first;
      }
  };

  std::priority_queue<PQElement, std::vector<PQElement>, PQComp > elements;


  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }

  void clear() {
    elements = std::priority_queue<PQElement, std::vector<PQElement>, PQComp>();
  }
};

/**
 * VCell: A cell used by voronoi graph search 
 * It has 3 elements: (x, y, z_cm). Where z is the height but in unit of centimeters.
 */
struct VCell {
  VCell() {}

  VCell(const int& x, const int& y, const int& z_cm)
    : x(x), y(y), z_cm(z_cm)
  {
    z_m = ((double)z_cm)/100.0;
    z = (int)(z_m/0.05); // 0.05 is the resolution
  }

  // Equality
  bool operator == (const VCell& pos) const
  {
    return (this->x == pos.x && this->y == pos.y && this->z_cm == pos.z_cm);
  }

  int x, y, z;  // Index of cell
  double z_m; // [THIS IS NOT THE INDEX] z in meters
  int z_cm; // [THIS IS NOT THE INDEX] z in centimeters
}; // struct VCell

// template <> 
// struct std::hash<VCell> {
//   /* implement hash function so we can put VCell into an unordered_set */
//   std::size_t operator()(const VCell& pos) const noexcept {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, pos.x);
//     boost::hash_combine(seed, pos.y);
//     boost::hash_combine(seed, pos.z);
//     return seed;
//   }
// };

template <> 
struct std::hash<VCell> {
  /* implement hash function so we can put VCell into an unordered_set */
  std::size_t operator()(const VCell& pos) const noexcept {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(pos.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};



/**
 * VCell_T: A space-time voronoi graph cell
 * It has 4 elements: (x, y, z, t). Where x,y,z are the index and t is the time at which the cell is occupied
 */
struct VCell_T {
  VCell_T() {}

  /**
   * @brief Construct a new VCell_T object
   * 
   * @param x index x coordinate
   * @param y index y coordinate
   * @param z_cm Height of voronoi map in centimeters 
   * @param t Time at which cell is occupied
   */
  VCell_T(const int& x, const int& y, const int& z_cm, const int& t)
    : x(x), y(y), z_cm(z_cm), t(t)
  {
    z_m = ((double)z_cm)/100.0;
    z = (int)(z_m/0.05); // 0.05 is the resolution
  }

  // Check if position is the same
  bool isSamePositionAs(const VCell_T& cell){
    return (this->x == cell.x && this->y == cell.y && this->z_cm == cell.z_cm );
  } 

  // Equality
  bool operator == (const VCell_T& cell) const
  {
    return (this->x == cell.x && this->y == cell.y && this->z_cm == cell.z_cm && this->t == cell.t);
  }

  int x, y, z;  // Index of cell
  double z_m; // [THIS IS NOT THE INDEX] z in meters
  int z_cm; // [THIS IS NOT THE INDEX] z in centimeters

  int t;  // Time at which cell is occupied
}; // struct VCell_T

// template <> 
// struct std::hash<VCell_T> {
//   /* implement hash function so we can put VCell_T into an unordered_set */
//   std::size_t operator()(const VCell_T& pos) const noexcept {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, pos.x);
//     boost::hash_combine(seed, pos.y);
//     boost::hash_combine(seed, pos.z);
//     boost::hash_combine(seed, pos.t);
//     return seed;
//   }
// };

template <> 
struct std::hash<VCell_T> {
  /* implement hash function so we can put VCell_T into an unordered_set */
  std::size_t operator()(const VCell_T& pos) const noexcept {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(pos.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.t) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};


/**
 * Cost operations for space-time Voronoi A* search
 */

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
static inline double getL1NormVT(const VCell_T& a, const VCell_T& b) {
  return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
static inline double getL2NormVT(const VCell_T& a, const VCell_T& b) {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return sqrt(dx*dx + dy*dy + dz*dz);
}

// Get octile distance
static inline double getChebyshevDistVT(const VCell_T& a, const VCell_T& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) - std::min(dx, std::min(dy, dz)); 
}

// // Get chebyshev distance
static inline double getOctileDistVT(const VCell_T& a, const VCell_T& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) + (SQRT2 - 2) * std::min(dx, std::min(dy, dz)); 
}

// template <> 
// struct std::hash<Eigen::Vector4d> {
//   /* implement hash function so we can put Eigen::Vector4d into an unordered_set */
//   std::size_t operator()(const Eigen::Vector4d& pos) const noexcept {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, pos(0));
//     boost::hash_combine(seed, pos(1));
//     boost::hash_combine(seed, pos(2));
//     boost::hash_combine(seed, pos(3));
//     return seed;
//   }
// };

// template <> 
// struct std::hash<Eigen::Vector4i> {
//   /* implement hash function so we can put Eigen::Vector4i into an unordered_set */
//   std::size_t operator()(const Eigen::Vector4i& pos) const noexcept {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, pos(0));
//     boost::hash_combine(seed, pos(1));
//     boost::hash_combine(seed, pos(2));
//     boost::hash_combine(seed, pos(3));
//     return seed;
//   }
// };

template <> 
struct std::hash<Eigen::Vector4d> {
  /* implement hash function so we can put Eigen::Vector4d into an unordered_set */
  std::size_t operator()(const Eigen::Vector4d& pos) const noexcept {
    std::size_t seed = 0;
    seed ^= std::hash<double>()(pos(0)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<double>()(pos(1)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<double>()(pos(2)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<double>()(pos(3)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <> 
struct std::hash<Eigen::Vector4i> {
  /* implement hash function so we can put Eigen::Vector4i into an unordered_set */
  std::size_t operator()(const Eigen::Vector4i& pos) const noexcept {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(pos(0)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos(1)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos(2)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos(3)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

/* Reservation table for space-time A*
*/
struct RsvnTbl{
  RsvnTbl(){}
  
  RsvnTbl(const double& t_plan_start): t_plan_start(t_plan_start)
  {}

  /**
   * @brief Check is position and time is in reservation table
   * 
   * @param pos_4d (x,y, z_cm, t) position to check
   * @param e_t [Space-time units] Elapsed time since plan started 
   */
  bool isReserved(const Eigen::Vector4i& pos_4d) const {
    return table.find(pos_4d) != table.end();
  }

  double t_plan_start; 
  std::unordered_set<Eigen::Vector4i> table;
};  // struct RsvnTbl

/* Helper */

// Convert from meters to centimeters
static inline int mToCm(const double& val_m){
  return (int) (val_m * 100.0);
}

// Convert from centimeters to meters
static inline double cmToM(const int& val_cm) {
  return ((double) val_cm)/100.0;  
}

/**
 * @brief Round to nearest multiple 
 * 
 * @param num Number to be rounded
 * @param mult Multiple
 * @return int 
 */
static inline int roundToMultInt(const int& num, const int& mult, const int& min, const int& max)
{
  if (mult == 0){
    return num;
  }

  if (num > max){
    return max;
  }

  if (num < min){
    return min;
  }

  int rem = (int)num % mult;
  if (rem == 0){
    return num;
  }

  return rem < (mult/2) ? (num-rem) : (num-rem) + mult;
}


#endif // _PLANNER_COMMON_HPP_

