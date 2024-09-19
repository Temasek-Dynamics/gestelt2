#ifndef _INT_POINT_H_
#define _INT_POINT_H_

#include <unordered_set>

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}

  void set(int _x, int _y){
    x = _x;
    y = _y;
  }

  // Equality
  bool operator == (const IntPoint& pos) const
  {
    return (this->x == pos.x && this->y == pos.y);
  }

  int x, y;
};

/*! A light-weight double point with fields x,y */
class DblPoint {
public:
  DblPoint() : x(0), y(0) {}
  DblPoint(double _x, double _y) : x(_x), y(_y) {}

  void set(double _x, double _y){
    x = _x;
    y = _y;
  }

  double x, y;
};

template <> 
struct std::hash<IntPoint> {
  /* implement hash function so we can put IntPoint into an unordered_set */
  std::size_t operator()(const IntPoint& pos) const noexcept {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(pos.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(pos.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};


// // Hash function for Eigen matrix and vector.
// // The code is from `hash_combine` function of the Boost library. See
// // http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
// template<typename T>
// struct matrix_hash : std::unary_function<T, size_t> {
//   std::size_t operator()(T const& matrix) const {
//     // Note that it is oblivious to the storage order of Eigen matrix (column- or
//     // row-major). It will give you the same hash value for two different matrices if they
//     // are the transpose of each other in different storage order.
//     size_t seed = 0;
//     for (size_t i = 0; i < matrix.size(); ++i) {
//       auto elem = *(matrix.data() + i);
//       seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//     }
//     return seed;
//   }
// };

#endif // _INT_POINT_H_
