#ifndef _BUCKETED_QUEUE_HPP_
#define _BUCKETED_QUEUE_HPP_

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include <map>

#include <planner_utils/int_point.hpp>

//! Priority queue for integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these groups are large.
 *  The elements are assumed to be integer coordinates, and the priorities are assumed
 *  to be squared Euclidean distances (integers).
 */

template <typename T>
class BucketPrioQueue {

public:
  //! Standard constructor
  /** Standard constructor. When called for the first time it creates a look up table 
   *  that maps square distances to bucket numbers, which might take some time...
   */
  BucketPrioQueue(); 


  void clear() { 
    buckets.clear();
    count = 0;
    nextPop = buckets.end();
  }

  //! Checks whether the Queue is empty
  bool empty();
  //! push an element
  void push(int prio, T t);
  //! return and pop the element with the lowest squared distance */
  T pop();
  
  int size() { return count; }
  int getNumBuckets() { return buckets.size(); }

  int getTopPriority(){
    return nextPop->first;
  }

private:
  
  int count;
  
  typedef std::map< int, std::queue<T> > BucketType;
  BucketType buckets;
  typename BucketType::iterator nextPop;
};

template <class T>
BucketPrioQueue<T>::BucketPrioQueue() {
  clear();
}

template <class T>
bool BucketPrioQueue<T>::empty() {
  return (count==0);
}

template <class T>
void BucketPrioQueue<T>::push(int prio, T t) {
  buckets[prio].push(t);
  if (nextPop == buckets.end() || prio < nextPop->first) nextPop = buckets.find(prio);
  count++;
}

template <class T>
T BucketPrioQueue<T>::pop() {
  while (nextPop!=buckets.end() && nextPop->second.empty()) ++nextPop;

  T p = nextPop->second.front();
  nextPop->second.pop();
  if (nextPop->second.empty()) {
    typename BucketType::iterator it = nextPop;
    nextPop++;
    buckets.erase(it);
  }
  count--;
  return p;
}

#endif // _BUCKETED_QUEUE_HPP_
