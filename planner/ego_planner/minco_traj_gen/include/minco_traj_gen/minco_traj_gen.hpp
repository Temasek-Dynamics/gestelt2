#ifndef MINCO_TRAJ_GEN__MINCO_TRAJ_GEN_HPP_
#define MINCO_TRAJ_GEN__MINCO_TRAJ_GEN_HPP_

#include "minco_traj_gen/visibility_control.h"

#include "root_finder.hpp"

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

// #include <boost/math/special_functions/factorials.hpp>

namespace poly_traj
{

  // using namespace boost::math;

  // Polynomial order and trajectory dimension are fixed here
  typedef Eigen::Matrix<double, 3, 6> CoefficientMat;
  typedef Eigen::Matrix<double, 3, 5> VelCoefficientMat;
  typedef Eigen::Matrix<double, 3, 4> AccCoefficientMat;

  class Piece
  {
  private:
      double duration;
      CoefficientMat coeffMat;

  public:
      Piece() = default;

      Piece(double dur, const CoefficientMat &cMat)
          : duration(dur), coeffMat(cMat) {}

      int getDim() const
      {
          return 3;
      }

      int getOrder() const
      {
          return 5;
      }

      double getDuration() const
      {
          return duration;
      }

      const CoefficientMat &getCoeffMat() const
      {
          return coeffMat;
      }

      VelCoefficientMat getVelCoeffMat() const
      {
          VelCoefficientMat velCoeffMat;
          int n = 1;
          for (int i = 4; i >= 0; i--)
          {
              velCoeffMat.col(i) = n * coeffMat.col(i);
              n++;
          }
          return velCoeffMat;
      }

      Eigen::Vector3d getPos(const double &t) const
      {
          Eigen::Vector3d pos(0.0, 0.0, 0.0);
          double tn = 1.0;

          for (int i = 5; i >= 0; i--)
          {
              pos += tn * coeffMat.col(i);
              tn *= t;
          }
          return pos;
      }

      Eigen::Vector3d getVel(const double &t) const
      {
          Eigen::Vector3d vel(0.0, 0.0, 0.0);
          double tn = 1.0;
          int n = 1;
          for (int i = 4; i >= 0; i--)
          {
              vel += n * tn * coeffMat.col(i);
              tn *= t;
              n++;
          }
          return vel;
      }

      Eigen::Vector3d getAcc(const double &t) const
      {
          Eigen::Vector3d acc(0.0, 0.0, 0.0);
          double tn = 1.0;
          int m = 1;
          int n = 2;
          for (int i = 3; i >= 0; i--)
          {
              acc += m * n * tn * coeffMat.col(i);
              tn *= t;
              m++;
              n++;
          }
          return acc;
      }

      Eigen::Vector3d getJer(const double &t) const
      {
          Eigen::Vector3d jer(0.0, 0.0, 0.0);
          double tn = 1.0;
          int l = 1;
          int m = 2;
          int n = 3;
          for (int i = 2; i >= 0; i--)
          {
              jer += l * m * n * tn * coeffMat.col(i);
              tn *= t;
              l++;
              m++;
              n++;
          }
          return jer;
      }

      CoefficientMat normalizePosCoeffMat() const
      {
          CoefficientMat nPosCoeffsMat;
          double t = 1.0;
          for (int i = 5; i >= 0; i--)
          {
              nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
              t *= duration;
          }
          return nPosCoeffsMat;
      }

      VelCoefficientMat normalizeVelCoeffMat() const
      {
          VelCoefficientMat nVelCoeffMat;
          int n = 1;
          double t = duration;
          for (int i = 4; i >= 0; i--)
          {
              nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
              t *= duration;
              n++;
          }
          return nVelCoeffMat;
      }

      AccCoefficientMat normalizeAccCoeffMat() const
      {
          AccCoefficientMat nAccCoeffMat;
          int n = 2;
          int m = 1;
          double t = duration * duration;
          for (int i = 3; i >= 0; i--)
          {
              nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
              n++;
              m++;
              t *= duration;
          }
          return nAccCoeffMat;
      }

      double getMaxVelRate() const
      {
          Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
          Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                  RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                  RootFinder::polySqr(nVelCoeffMat.row(2));
          int N = coeff.size();
          int n = N - 1;
          for (int i = 0; i < N; i++)
          {
              coeff(i) *= n;
              n--;
          }
          if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
          {
              return 0.0;
          }
          else
          {
              double l = -0.0625;
              double r = 1.0625;
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
              {
                  l = 0.5 * l;
              }
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
              {
                  r = 0.5 * (r + 1.0);
              }
              std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
              candidates.insert(0.0);
              candidates.insert(1.0);
              double maxVelRateSqr = -INFINITY;
              double tempNormSqr;
              for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
              {
                  if (0.0 <= *it && 1.0 >= *it)
                  {
                      tempNormSqr = getVel((*it) * duration).squaredNorm();
                      maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                  }
              }
              return sqrt(maxVelRateSqr);
          }
      }

      double getMaxAccRate() const
      {
          Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
          Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                  RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                  RootFinder::polySqr(nAccCoeffMat.row(2));
          int N = coeff.size();
          int n = N - 1;
          for (int i = 0; i < N; i++)
          {
              coeff(i) *= n;
              n--;
          }
          if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
          {
              return 0.0;
          }
          else
          {
              double l = -0.0625;
              double r = 1.0625;
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
              {
                  l = 0.5 * l;
              }
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
              {
                  r = 0.5 * (r + 1.0);
              }
              std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
              candidates.insert(0.0);
              candidates.insert(1.0);
              double maxAccRateSqr = -INFINITY;
              double tempNormSqr;
              for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
              {
                  if (0.0 <= *it && 1.0 >= *it)
                  {
                      tempNormSqr = getAcc((*it) * duration).squaredNorm();
                      maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                  }
              }
              return sqrt(maxAccRateSqr);
          }
      }

      bool checkMaxVelRate(const double &maxVelRate) const
      {
          double sqrMaxVelRate = maxVelRate * maxVelRate;
          if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
              getVel(duration).squaredNorm() >= sqrMaxVelRate)
          {
              return false;
          }
          else
          {
              Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
              Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                      RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                      RootFinder::polySqr(nVelCoeffMat.row(2));
              double t2 = duration * duration;
              coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
              return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
          }
      }

      bool checkMaxAccRate(const double &maxAccRate) const
      {
          double sqrMaxAccRate = maxAccRate * maxAccRate;
          if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
              getAcc(duration).squaredNorm() >= sqrMaxAccRate)
          {
              return false;
          }
          else
          {
              Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
              Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                      RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                      RootFinder::polySqr(nAccCoeffMat.row(2));
              double t2 = duration * duration;
              double t4 = t2 * t2;
              coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
              return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
          }
      }

      // GaaiLam
      bool project_pt(const Eigen::Vector3d &pt,
                              double &tt, Eigen::Vector3d &pro_pt)
      {
          // 2*(p-p0)^T * \dot{p} = 0
          auto l_coeff = getCoeffMat();
          l_coeff.col(5) = l_coeff.col(5) - pt;
          auto r_coeff = getVelCoeffMat();
          Eigen::VectorXd eq = Eigen::VectorXd::Zero(2 * 5);
          for (int j = 0; j < l_coeff.rows(); ++j)
          {
              eq = eq + RootFinder::polyConv(l_coeff.row(j), r_coeff.row(j));
          }
          double l = -0.0625;
          double r = duration + 0.0625;
          while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
          {
              l = 0.5 * l;
          }
          while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
          {
              r = 0.5 * (duration + 1.0);
          }
          std::set<double> roots =
              RootFinder::solvePolynomial(eq, l, r, 1e-6);
          // std::cout << "# roots: " << roots.size() << std::endl;
          double min_dist = -1;
          for (const auto &root : roots)
          {
              // std::cout << "root: " << root << std::endl;
              if (root < 0 || root > duration)
              {
                  continue;
              }
              if (getVel(root).norm() < 1e-6)
              { // velocity == 0, ignore it
                  continue;
              }
              // std::cout << "find min!" << std::endl;
              Eigen::Vector3d p = getPos(root);
              // std::cout << "p: " << p.transpose() << std::endl;
              double distance = (p - pt).norm();
              if (distance < min_dist || min_dist < 0)
              {
                  min_dist = distance;
                  tt = root;
                  pro_pt = p;
              }
          }
          return min_dist > 0;
      }

      bool intersection_plane(const Eigen::Vector3d p,
                                      const Eigen::Vector3d v,
                                      double &tt, Eigen::Vector3d &pt) const
      {
          // (pt - p)^T * v = 0
          auto coeff = getCoeffMat();
          coeff.col(5) = coeff.col(5) - p;
          Eigen::VectorXd eq = coeff.transpose() * v;
          double l = -0.0625;
          double r = duration + 0.0625;
          while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
          {
              l = 0.5 * l;
          }
          while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
          {
              r = 0.5 * (duration + 1.0);
          }
          std::set<double> roots =
              RootFinder::solvePolynomial(eq, l, r, 1e-6);
          for (const auto &root : roots)
          {
              tt = root;
              pt = getPos(root);
              return true;
          }
          return false;
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class Trajectory
  {
  private:
      typedef std::vector<Piece> Pieces;
      Pieces pieces;
      double global_start_time_{-1}; // Global start time in seconds

  public:
      Trajectory() = default;

      Trajectory(const std::vector<double> &durs,
                  const std::vector<CoefficientMat> &cMats)
      {
          int N = std::min(durs.size(), cMats.size());
          pieces.reserve(N);
          for (int i = 0; i < N; i++)
          {
              pieces.emplace_back(durs[i], cMats[i]);
          }
      }

      void setGlobalStartTime(const double& global_start_time)
      {
          global_start_time_ = global_start_time;
      }

      double getGlobalStartTime() const
      {
          return global_start_time_;
      }

      /**
       * @brief Get the size of each piece i.e. the number of points in a piece
       * 
       * @return int 
       */
      int getPieceSize() const
      {
          return pieces.size();
      }
      
      /**
       * @brief Get a vector of duration of every point
       * 
       * @return Eigen::VectorXd 
       */
      Eigen::VectorXd getDurations() const
      {
          int N = getPieceSize();
          Eigen::VectorXd durations(N);
          for (int i = 0; i < N; i++)
          {
              durations(i) = pieces[i].getDuration();
          }
          return durations;
      }

      /**
       * @brief Get a vector of duration of every point
       * 
       * @return Eigen::VectorXd 
       */
      std::vector<double> getSegDurationsDouble() const
      {
          int N = getPieceSize();
          std::vector<double> durations(N);
          // Eigen::VectorXd durations(N);
          for (int i = 0; i < N; i++)
          {
              // durations(i) = pieces[i].getDuration();
              durations[i] = pieces[i].getDuration();
          }

          // return std::vector<double>(durations.data(), durations.data() + durations.cols());
          return durations;
      }


      /**
       * @brief Get the total duration of the trajectory
       * 
       * @return double 
       */
      double getTotalDuration() const
      {
          int N = getPieceSize();
          double totalDuration = 0.0;
          for (int i = 0; i < N; i++)
          {
              totalDuration += pieces[i].getDuration();
          }
          return totalDuration;
      }

      Eigen::MatrixXd getPositions() const
      {
          int N = getPieceSize();
          Eigen::MatrixXd positions(3, N + 1);
          for (int i = 0; i < N; i++)
          {
              positions.col(i) = pieces[i].getCoeffMat().col(5);
          }
          positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
          return positions;
      }

      const Piece &operator[](int i) const
      {
          return pieces[i];
      }

      Piece &operator[](int i)
      {
          return pieces[i];
      }

      void clear(void)
      {
          pieces.clear();
          return;
      }

      Pieces::const_iterator begin() const
      {
          return pieces.begin();
      }

      Pieces::const_iterator end() const
      {
          return pieces.end();
      }

      Pieces::iterator begin()
      {
          return pieces.begin();
      }

      Pieces::iterator end()
      {
          return pieces.end();
      }

      void reserve(const int &n)
      {
          pieces.reserve(n);
          return;
      }

      void emplace_back(const Piece &piece)
      {
          pieces.emplace_back(piece);
          return;
      }

      void emplace_back(const double &dur,
                                const CoefficientMat &cMat)
      {
          pieces.emplace_back(dur, cMat);
          return;
      }

      void append(const Trajectory &traj)
      {
          pieces.insert(pieces.end(), traj.begin(), traj.end());
          return;
      }

      int locatePieceIdx(double &t) const
      {
          int N = getPieceSize();
          int idx;
          double dur;
          for (idx = 0;
                idx < N &&
                t > (dur = pieces[idx].getDuration());
                idx++)
          {
              t -= dur;
          }
          if (idx == N)
          {
              idx--;
              t += pieces[idx].getDuration();
          }
          return idx;
      }

      Eigen::Vector3d getPos(double t) const
      {
          // locate the piece that time t belongs to
          int pieceIdx = locatePieceIdx(t);
          // within that piece, get the postion at time t
          return pieces[pieceIdx].getPos(t);
      }

      Eigen::Vector3d getVel(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getVel(t);
      }

      Eigen::Vector3d getAcc(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getAcc(t);
      }

      Eigen::Vector3d getJer(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getJer(t);
      }
      
      /**
       * @brief Get position at the start of the indexed piece/segment
       * 
       * @param pieceIdx  Segment index
       * @return Eigen::Vector3d 
       */
      Eigen::Vector3d getJuncPos(int pieceIdx) const
      {
          if (pieceIdx != getPieceSize())
          {
              return pieces[pieceIdx].getCoeffMat().col(5);
          }
          else
          {
              return pieces[pieceIdx - 1].getPos(pieces[pieceIdx - 1].getDuration());
          }
      }

      /**
       * @brief Get velocity at the start of the indexed piece/segment
       * 
       * @param pieceIdx  Segment index
       * @return Eigen::Vector3d 
       */
      Eigen::Vector3d getJuncVel(int pieceIdx) const
      {
          if (pieceIdx != getPieceSize())
          {
              return pieces[pieceIdx].getCoeffMat().col(4);
          }
          else
          {
              return pieces[pieceIdx - 1].getVel(pieces[pieceIdx - 1].getDuration());
          }
      }

      /**
       * @brief Get acceleration at the start of the indexed piece/segment
       * 
       * @param pieceIdx  Segment index
       * @return Eigen::Vector3d 
       */
      Eigen::Vector3d getJuncAcc(int pieceIdx) const
      {
          if (pieceIdx != getPieceSize())
          {
              return pieces[pieceIdx].getCoeffMat().col(3) * 2.0;
          }
          else
          {
              return pieces[pieceIdx - 1].getAcc(pieces[pieceIdx - 1].getDuration());
          }
      }
      
      /**
       * @brief Get the Max velocity along the piece
       * 
       * @return double 
       */
      double getMaxVelRate() const
      {
          int N = getPieceSize();
          double maxVelRate = -INFINITY;
          double tempNorm;
          for (int i = 0; i < N; i++)
          {
              tempNorm = pieces[i].getMaxVelRate();
              maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
          }
          return maxVelRate;
      }

      double getMaxAccRate() const
      {
          int N = getPieceSize();
          double maxAccRate = -INFINITY;
          double tempNorm;
          for (int i = 0; i < N; i++)
          {
              tempNorm = pieces[i].getMaxAccRate();
              maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
          }
          return maxAccRate;
      }

      bool checkMaxVelRate(const double &maxVelRate) const
      {
          int N = getPieceSize();
          bool feasible = true;
          for (int i = 0; i < N && feasible; i++)
          {
              feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
          }
          return feasible;
      }

      bool checkMaxAccRate(const double &maxAccRate) const
      {
          int N = getPieceSize();
          bool feasible = true;
          for (int i = 0; i < N && feasible; i++)
          {
              feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
          }
          return feasible;
      }

      // GaaiLam
      Piece getPiece(int i) const
      {
          return pieces[i];
      }

      bool project_pt(const Eigen::Vector3d &pt,
                              int &ii, double &tt, Eigen::Vector3d &pro_pt)
      {
          bool find_project_pt = false;
          for (int i = 0; i < getPieceSize(); ++i)
          {
              auto piece = pieces[i];
              if (piece.project_pt(pt, tt, pro_pt))
              {
                  ii = i;
                  find_project_pt = true;
                  break;
              }
          }
          if (!find_project_pt)
          {
              // std::cout << "\033[32m" << "cannot project pt to traj" << "\033[0m" << std::endl;
              // std::cout << "pt: " << pt.transpose() << std::endl;
              // assert(false);
          }
          return find_project_pt;
      }
      bool intersection_plane(const Eigen::Vector3d p,
                                      const Eigen::Vector3d v,
                                      int &ii, double &tt, Eigen::Vector3d &pt)
      {
          for (int i = 0; i < getPieceSize(); ++i)
          {
              const auto &piece = pieces[i];
              if (piece.intersection_plane(p, v, tt, pt))
              {
                  ii = i;
                  return true;
              }
          }
          return false;
      }

      std::vector<Eigen::Vector3d> way_points()
      {
          std::vector<Eigen::Vector3d> pts;
          for (int i = 0; i < getPieceSize(); ++i)
          {
              pts.push_back(pieces[i].getPos(0));
          }
          return pts;
      }

      // zxzx
      /**
       * @brief Locate the (seg_index, t_in_segment / dur_segment) pair given the time
       * 
       * @param t Current time 
       * @return std::pair<int, double> Returns a pair of 
       * (segment_index, ratio of current time in segment to the total segment duration) 
       */
      std::pair<int, double> getIdxTimeRatioAtTime(double &t) const
      {
          int N = getPieceSize(); // Number of segments
          int seg_idx;    // Index of the segment
          double seg_dur; // Duration of the current segment
          double t_in_segment = t; // Time locally within the segment
          // Given the point at desired time (t), get the index (seg_idx) of the segment which corresponds to it
          for (seg_idx = 0;
                seg_idx < N && t_in_segment > (seg_dur = pieces[seg_idx].getDuration());
                seg_idx++)
          {
              t_in_segment -= seg_dur;
          }
          if (seg_idx == N) // IF final segment
          {
              seg_idx--;
              t_in_segment += pieces[seg_idx].getDuration();
          }
          std::pair<int, double> idx_time_ratio_pair{
              seg_idx, (t_in_segment / seg_dur)
          };
          return idx_time_ratio_pair;
      }

      Eigen::Vector3d getPoswithIdxRatio(double t, std::pair<int, double> &idx_ratio) const
      {
          idx_ratio = getIdxTimeRatioAtTime(t);
          return pieces[idx_ratio.first].getPos(t);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  // The banded system class is used for solving
  // banded linear system Ax=b efficiently.
  // A is an N*N band matrix with lower band width lowerBw
  // and upper band width upperBw.
  // Banded LU factorization has O(N) time complexity.
  class BandedSystem
  {
  public:
      // The size of A, as well as the lower/upper
      // banded width p/q are needed
      void create(const int &n, const int &p, const int &q)
      {
          // In case of re-creating before destroying
          destroy();
          N = n;
          lowerBw = p;
          upperBw = q;
          int actualSize = N * (lowerBw + upperBw + 1);
          ptrData = new double[actualSize];
          std::fill_n(ptrData, actualSize, 0.0);
          return;
      }

      void destroy()
      {
          if (ptrData != nullptr)
          {
              delete[] ptrData;
              ptrData = nullptr;
          }
          return;
      }

      void operator=(const BandedSystem &bs)
      {
          ptrData = nullptr;
          create(bs.N, bs.lowerBw, bs.upperBw);
          memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
      }

  private:
      int N;
      int lowerBw;
      int upperBw;
      double *ptrData = nullptr;

  public:
      // Reset the matrix to zero
      void reset(void)
      {
          std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
          return;
      }

      // The band matrix is stored as suggested in "Matrix Computation"
      const double &operator()(const int &i, const int &j) const
      {
          return ptrData[(i - j + upperBw) * N + j];
      }

      double &operator()(const int &i, const int &j)
      {
          return ptrData[(i - j + upperBw) * N + j];
      }

      // This function conducts banded LU factorization in place
      // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
      void factorizeLU()
      {
          int iM, jM;
          double cVl;
          for (int k = 0; k <= N - 2; k++)
          {
              iM = std::min(k + lowerBw, N - 1);
              cVl = operator()(k, k);
              for (int i = k + 1; i <= iM; i++)
              {
                  if (operator()(i, k) != 0.0)
                  {
                      operator()(i, k) /= cVl;
                  }
              }
              jM = std::min(k + upperBw, N - 1);
              for (int j = k + 1; j <= jM; j++)
              {
                  cVl = operator()(k, j);
                  if (cVl != 0.0)
                  {
                      for (int i = k + 1; i <= iM; i++)
                      {
                          if (operator()(i, k) != 0.0)
                          {
                              operator()(i, j) -= operator()(i, k) * cVl;
                          }
                      }
                  }
              }
          }
          return;
      }

      // This function solves Ax=b, then stores x in b
      // The input b is required to be N*m, i.e.,
      // m vectors to be solved.
      void solve(Eigen::MatrixXd &b) const
      {
          int iM;
          for (int j = 0; j <= N - 1; j++)
          {
              iM = std::min(j + lowerBw, N - 1);
              for (int i = j + 1; i <= iM; i++)
              {
                  if (operator()(i, j) != 0.0)
                  {
                      b.row(i) -= operator()(i, j) * b.row(j);
                  }
              }
          }
          for (int j = N - 1; j >= 0; j--)
          {
              b.row(j) /= operator()(j, j);
              iM = std::max(0, j - upperBw);
              for (int i = iM; i <= j - 1; i++)
              {
                  if (operator()(i, j) != 0.0)
                  {
                      b.row(i) -= operator()(i, j) * b.row(j);
                  }
              }
          }
          return;
      }

      // This function solves A.T x = b, then stores x in b
      // The input b is required to be N*m, i.e.,
      // m vectors to be solved.
      void solveAdj(Eigen::MatrixXd &b) const
      {
          int iM;
          for (int j = 0; j <= N - 1; j++)
          {
              b.row(j) /= operator()(j, j);
              iM = std::min(j + upperBw, N - 1);
              for (int i = j + 1; i <= iM; i++)
              {
                  if (operator()(j, i) != 0.0)
                  {
                      b.row(i) -= operator()(j, i) * b.row(j);
                  }
              }
          }
          for (int j = N - 1; j >= 0; j--)
          {
              iM = std::max(0, j - lowerBw);
              for (int i = iM; i <= j - 1; i++)
              {
                  if (operator()(j, i) != 0.0)
                  {
                      b.row(i) -= operator()(j, i) * b.row(j);
                  }
              }
          }
          return;
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class MinJerkOpt
  {
  public:
      void operator=(const MinJerkOpt &mjo)
      {
          N = mjo.N;
          headPVA = mjo.headPVA;
          tailPVA = mjo.tailPVA;
          T1 = mjo.T1;
          A = mjo.A;
          b = mjo.b;
          T2 = mjo.T2;
          T3 = mjo.T3;
          T4 = mjo.T4;
          T5 = mjo.T5;
      }
      ~MinJerkOpt() { A.destroy(); }

  private:
      int N; // Number of pieces
      Eigen::Matrix3d headPVA; // Start PVA
      Eigen::Matrix3d tailPVA; // Goal PVA
      Eigen::VectorXd T1; // shape (N, 1) Vector of time duration of each piece

      // A : shape (2*M*s, 2*M*s) = (6*M, 6*M)
      BandedSystem A; // M banded matrix in the literature

      //  b: shape (6 * N, 3)
      // 
      //     x-axis, y-axis, z-axis  
      //   [ cy_0_0  cy_0_0  cz_0_0;  // Start of 1st segment
      //     cy_1_0  cy_1_0  cz_1_0;      
      //     cy_2_0  cy_2_0  cz_2_0;      
      //     cy_3_0  cy_3_0  cz_3_0;
      //     cy_4_0  cy_4_0  cz_4_0;      
      //     cy_5_0  cy_5_0  cz_5_0;      
      //     ...     ...      ... ;      
      //     cy_0_N  cy_0_N  cz_0_N;  // Start of n-th segment
      //     cy_1_N  cy_1_N  cz_1_N;      
      //     cy_2_N  cy_2_N  cz_2_N;      
      //     cy_3_N  cy_3_N  cz_3_N;
      //     cy_4_N  cy_4_N  cz_4_N;      
      //     cy_5_N  cy_5_N  cz_5_N;] 
      Eigen::MatrixXd b;  // shape (6 * N, 3) Polynomial coefficients

      // Temp variables
      Eigen::VectorXd T2; // T1**2
      Eigen::VectorXd T3; // T1**3
      Eigen::VectorXd T4; // T1**4
      Eigen::VectorXd T5; // T1**5

  public:
    //   /**
    //    * @brief Set the boundary states and resize 
    //    * 
    //    * @param startPVA  Boundary start condition: Matrix consisting of 3d (position, velocity acceleration) 
    //    * @param endPVA    Boundary end condition: Matrix consisting of 3d (position, velocity acceleration) 
    //    * @param pieceNum  Number of path segments
    //    */
    //   void reset( const Eigen::Matrix3d &startPVA,
    //               const Eigen::Matrix3d &endPVA,
    //               const int &pieceNum)
    //   {
    //       N = pieceNum; // Number of segments
    //       headPVA = startPVA;
    //       tailPVA = endPVA;
    //       T1.resize(N);
    //       A.create(6 * N, 6, 6);
    //       b.resize(6 * N, 3);
    //       return;
    //   }

      /**
       * @brief 
       * 
       * @param inPs inner waypoints
       * @param ts time vector
       */
      void generate(const Eigen::Matrix3d& startPVA, 
                    const Eigen::Matrix3d& endPVA,
                    const Eigen::MatrixXd &inPs,
                    const Eigen::VectorXd &ts)
      {
          headPVA = startPVA;   // start PVA
          tailPVA = endPVA;     // end PVA
          N = 1 + inPs.cols(); // Number of segments
          T1.resize(N);
          A.create(6 * N, 6, 6);
          b.resize(6 * N, 3);

          // Default case: Single goal waypoint
          if (inPs.cols() == 0)
          {
              T1(0) = ts(0); // time duration 
              double t1_inv = 1.0 / T1(0);      // 1/t
              double t2_inv = t1_inv * t1_inv;  // 1/(t^2)
              double t3_inv = t2_inv * t1_inv;  // 1/(t^3)
              double t4_inv = t2_inv * t2_inv;  // 1/(t^4)
              double t5_inv = t4_inv * t1_inv;  // 1/(t^5)
              // CoefficientMat coeffMatReversed; // Of type Eigen::Matrix<double, 3, 6>

              Eigen::Matrix<double, 6, 3> coeffMat;

              // headPVA.col(0) = start pos (3x1), col(1) = start vel (3x1), col(2) = start acc (3x1)
              // tailPVA.col(0) = end pos (3x1), col(1) = end vel (3x1), col(2) = end acc (3x1)
              
              // Condition for a minimum jerk trajectory according to the Euler-Lagrange equation is that
              // 6th derivative of position P must be 0
              // Hence the trajectory follows a 5-th order polynomial of the form 
              // c_5*(t^5) + c_4*(t^4) + c_3*(t^3) + c_2*(t^2) + c_1*(t) + c_0
              
              /* The 5-th order polynomial equation is given in b = Ax as follows: */
              // [P_i; V_i; A_i; P_f; V_f; A_f] = [
              //      [0,        0,          0,      0,      0, 1],
              //      [0,        0,          0,      0,      1, 0],
              //      [0,        0,          0,      2,      0, 0],
              //      [t**5,     t**4,       t**3,   t**2,   t, 1],
              //      [5*t**4,   4*t**3,     3*t**2, 2*t,    1, 0],
              //      [20*t**3,  12*t**2,    6*t,    2,      0, 0], 
              // ] 
              // * [c_5, c_4, c_3, c_2, c_1, c_0]

              /* We the invert the A, and get the equation as x = inv(A) * b as follows: */
              // [c_5, c_4, c_3, c_2, c_1, c_0] = [
              //      [-6/t**5, -3/t**4, -1/(2*t**3), 6/t**5, -3/t**4, 1/(2*t**3)], 
              //      [15/t**4, 8/t**3, 3/(2*t**2), -15/t**4, 7/t**3, -1/t**2], 
              //      [-10/t**3, -6/t**2, -3/(2*t), 10/t**3, -4/t**2, 1/(2*t)], 
              //      [0, 0, 1/2, 0, 0, 0], 
              //      [0, 1, 0, 0, 0, 0], 
              //      [1, 0, 0, 0, 0, 0]]
              // ] 
              // * [P_i; V_i; A_i; P_f; V_f; A_f]
              // = [
              //     [A_f/(2*t**3) - A_i/(2*t**3) + 6*P_f/t**5 - 6*P_i/t**5 - 3*V_f/t**4 - 3*V_i/t**4], 
              //     [-A_f/t**2 + 3*A_i/(2*t**2) - 15*P_f/t**4 + 15*P_i/t**4 + 7*V_f/t**3 + 8*V_i/t**3], 
              //     [A_f/(2*t) - 3*A_i/(2*t) + 10*P_f/t**3 - 10*P_i/t**3 - 4*V_f/t**2 - 6*V_i/t**2], 
              //     [A_i/2], 
              //     [V_i], 
              //     [P_i]
              // ]

              // Note that in coefficients [c_5, c_4, c_3, c_2, c_1, c_0], each coefficient is a 3x1 vector

              // coeffMatReversed.col(5) = 0.5 * (tailPVA.col(2) - headPVA.col(2)) * t3_inv -
              //                           3.0 * (headPVA.col(1) + tailPVA.col(1)) * t4_inv +
              //                           6.0 * (tailPVA.col(0) - headPVA.col(0)) * t5_inv;
              // coeffMatReversed.col(4) = (-tailPVA.col(2) + 1.5 * headPVA.col(2)) * t2_inv +
              //                           (8.0 * headPVA.col(1) + 7.0 * tailPVA.col(1)) * t3_inv +
              //                           15.0 * (-tailPVA.col(0) + headPVA.col(0)) * t4_inv;
              // coeffMatReversed.col(3) = (0.5 * tailPVA.col(2) - 1.5 * headPVA.col(2)) * t1_inv -
              //                           (6.0 * headPVA.col(1) + 4.0 * tailPVA.col(1)) * t2_inv +
              //                           10.0 * (tailPVA.col(0) - headPVA.col(0)) * t3_inv;
              // coeffMatReversed.col(2) = 0.5 * headPVA.col(2);
              // coeffMatReversed.col(1) = headPVA.col(1);
              // coeffMatReversed.col(0) = headPVA.col(0);
              // b = coeffMatReversed.transpose();  

              coeffMat.row(0) = (headPVA.col(0)).transpose();
              coeffMat.row(1) = (headPVA.col(1));
              coeffMat.row(2) = (0.5 * headPVA.col(2)).transpose();
              coeffMat.row(3) = ((0.5 * tailPVA.col(2) - 1.5 * headPVA.col(2)) * t1_inv -
                                        (6.0 * headPVA.col(1) + 4.0 * tailPVA.col(1)) * t2_inv +
                                        10.0 * (tailPVA.col(0) - headPVA.col(0)) * t3_inv).transpose();
              coeffMat.row(4) = ((-tailPVA.col(2) + 1.5 * headPVA.col(2)) * t2_inv +
                                        (8.0 * headPVA.col(1) + 7.0 * tailPVA.col(1)) * t3_inv +
                                        15.0 * (-tailPVA.col(0) + headPVA.col(0)) * t4_inv).transpose();
              coeffMat.row(5) = (0.5 * (tailPVA.col(2) - headPVA.col(2)) * t3_inv -
                                        3.0 * (headPVA.col(1) + tailPVA.col(1)) * t4_inv +
                                        6.0 * (tailPVA.col(0) - headPVA.col(0)) * t5_inv).transpose();
              b = coeffMat;
          }
          
          // Multiple waypoints
          else
          {
              T1 = ts; // Time duration vector
              // cwiseProduct: coefficient-wise product
              T2 = T1.cwiseProduct(T1); // t^2
              T3 = T2.cwiseProduct(T1); // t^3
              T4 = T2.cwiseProduct(T2); // t^4
              T5 = T4.cwiseProduct(T1); // t^5

              A.reset();
              b.setZero();

              A(0, 0) = 1.0;
              A(1, 1) = 1.0;
              A(2, 2) = 2.0;
              b.row(0) = headPVA.col(0).transpose();
              b.row(1) = headPVA.col(1).transpose();
              b.row(2) = headPVA.col(2).transpose();

              for (int i = 0; i < N - 1; i++) // For each segment
              {
                  A(6 * i + 3, 6 * i + 3) = 6.0;
                  A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                  A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                  A(6 * i + 3, 6 * i + 9) = -6.0;
                  A(6 * i + 4, 6 * i + 4) = 24.0;
                  A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                  A(6 * i + 4, 6 * i + 10) = -24.0;
                  A(6 * i + 5, 6 * i) = 1.0;
                  A(6 * i + 5, 6 * i + 1) = T1(i);
                  A(6 * i + 5, 6 * i + 2) = T2(i);
                  A(6 * i + 5, 6 * i + 3) = T3(i);
                  A(6 * i + 5, 6 * i + 4) = T4(i);
                  A(6 * i + 5, 6 * i + 5) = T5(i);
                  A(6 * i + 6, 6 * i) = 1.0;
                  A(6 * i + 6, 6 * i + 1) = T1(i);
                  A(6 * i + 6, 6 * i + 2) = T2(i);
                  A(6 * i + 6, 6 * i + 3) = T3(i);
                  A(6 * i + 6, 6 * i + 4) = T4(i);
                  A(6 * i + 6, 6 * i + 5) = T5(i);
                  A(6 * i + 6, 6 * i + 6) = -1.0;
                  A(6 * i + 7, 6 * i + 1) = 1.0;
                  A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                  A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                  A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                  A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                  A(6 * i + 7, 6 * i + 7) = -1.0;
                  A(6 * i + 8, 6 * i + 2) = 2.0;
                  A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                  A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                  A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                  A(6 * i + 8, 6 * i + 8) = -2.0;

                  b.row(6 * i + 5) = inPs.col(i).transpose();
              }

              // Final segment
              A(6 * N - 3, 6 * N - 6) = 1.0;
              A(6 * N - 3, 6 * N - 5) = T1(N - 1);
              A(6 * N - 3, 6 * N - 4) = T2(N - 1);
              A(6 * N - 3, 6 * N - 3) = T3(N - 1);
              A(6 * N - 3, 6 * N - 2) = T4(N - 1);
              A(6 * N - 3, 6 * N - 1) = T5(N - 1);
              A(6 * N - 2, 6 * N - 5) = 1.0;
              A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
              A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
              A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
              A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
              A(6 * N - 1, 6 * N - 4) = 2;
              A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
              A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
              A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

              b.row(6 * N - 3) = tailPVA.col(0).transpose();
              b.row(6 * N - 2) = tailPVA.col(1).transpose();
              b.row(6 * N - 1) = tailPVA.col(2).transpose();

              A.factorizeLU(); // banded LU factorization in place
              A.solve(b);

              return;
          }
      }

      const Eigen::MatrixXd &get_b() const
      {
        return b;
      }

      /**
       * @brief Get number of trajectory segments
       * 
       * @return size_t 
       */
      size_t getNumSegs(){
          return N;
      }

      /**
       * @brief Return a vector of time durations of each piece/segment
       * 
       * @return const Eigen::VectorXd& 
       */
      const Eigen::VectorXd &getSegDurations() const
      {
        return T1;
      }

      /**
       * @brief Return a vector of time durations of each piece/segment
       * 
       * @return const Eigen::VectorXd& 
       */
      std::vector<double> getSegDurationsDouble() const
      {
          // std::vector<double> seg_durs(T1.cols());

          // for (int i = 0; i < T1.cols(); i++) // For each polynomial segment
          // {
          //     seg_durs[i] = T1(i);
          // }

          // return seg_durs;

          return std::vector<double>(T1.data(), T1.data() + T1.cols());
      }

      /**
       * @brief Original code: Get the cost of jerk for the entire trajectory 
       * 
       * @return double 
       */
      double getTrajJerkCost() const
      {
          double objective = 0.0;
          for (int i = 0; i < N; i++) // for each segment
          {

              objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +          // 36  * c3^2     * t
                            144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) + // 144 * c3 * c4  * t^2
                            192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +         // 192 * c4^2     * t^3 
                            240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) + // 240 * c3 * c5  * t^3  
                            720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) + // 720 * c4 * c5  * t^4 
                            720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);          // 720 * c5^2     * t^5 

          }

          return objective;
      }

      Trajectory getTraj(void) const
      {
          Trajectory traj;
          traj.reserve(N);
          for (int i = 0; i < N; i++) // For each polynomial segment
          {
              // Block of size (6,3) at (0,0), (6,0), ... (6*(N-1), 0)
              // Reverse rowwise (i.e. left to right -> right to left)

              // To trajectory, append (time duration of each piece, 5th order polynomial coefficients of 3 axes (x,y,z))
              traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
          }
          return traj;
      }

      /**
       * @brief Get Trajectory and set starting time
       * 
       * @param traj_start_t 
       * @return Trajectory 
       */
      Trajectory getTraj(const double& traj_start_t) const
      {
          Trajectory traj;
          traj.reserve(N);
          for (int i = 0; i < N; i++) // For each polynomial segment
          {
              // Block of size (6,3) at (0,0), (6,0), ... (6*(N-1), 0)
              // Reverse rowwise (i.e. left to right -> right to left)

              // To trajectory, append (time duration of each piece, 5th order polynomial coefficients of 3 axes (x,y,z))
              traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
          }

          traj.setGlobalStartTime(traj_start_t);
          
          return traj;
      }

      
      /**
       * @brief Given desired number of constraint points per segment, get all constraint points 
       * along trajectory. This is done by stepping through time and sampling the polynomial segment
       * 
       * @param num_cp Number of constraint points per piece
       * @return Eigen::MatrixXd Matrix of points of size (3, N*num_cp+1)
       */
      std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> getTimePositionPairs(const int num_cp) const
      {   
          std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_check;
          pts_check.resize(N);

          Eigen::Vector3d pos;
          Eigen::Matrix<double, 6, 1> beta0; //Time values
          double s1, s2, s3, s4, s5; //Time basis
          double traj_t = 0.0; // Time from start of trajectory

          for (int i = 0; i < N; ++i) // For each trajectory segment/piece
          {
              const auto &c = b.block<6, 3>(i * 6, 0);
              double step = T1(i) / num_cp; // step is time duration / number of constraint points
              s1 = 0.0; // Time stamp

              std::vector<std::pair<double, Eigen::Vector3d>> segment;
              for (int j = 0; j <= num_cp; ++j) // For each constraint point
              {
                  s2 = s1 * s1; // t^2
                  s3 = s2 * s1; // t^3
                  s4 = s2 * s2; // t^4
                  s5 = s4 * s1; // t^5
                  beta0 << 1.0, s1, s2, s3, s4, s5;
                  
                  
                  pos = c.transpose() * beta0; // pos = c_5*(t**5) + c_4*(t**4) + c_3*(t**3) + c_2*(t**2) + c_1*(t) + c_0
                  segment.push_back(std::make_pair(traj_t, pos));

                  // step through time
                  s1 += step; 
                  traj_t += step;
              }
              // add entire segment 
              pts_check.push_back(segment);
          }

          return pts_check;
      }

      /**
       * @brief Given desired number of constraint points per segment, get all constraint points 
       * along trajectory. This is done by stepping through time and sampling the polynomial segment
       * 
       * @param num_cp Number of constraint points per piece
       * @return Eigen::MatrixXd Matrix of points of size (3, N*num_cp+1)
       */
      Eigen::MatrixXd getInitConstraintPoints(const int num_cp) const
      {   
          Eigen::MatrixXd pts(3, N * num_cp + 1); // Each column of pts is a 3d position value
          Eigen::Vector3d pos;
          Eigen::Matrix<double, 6, 1> beta0; //Time values
          double s1, s2, s3, s4, s5; //Time values
          double step;
          int i_dp = 0;

          for (int i = 0; i < N; ++i) // For each trajectory segment/piece
          {
              const auto &c = b.block<6, 3>(i * 6, 0);
              step = T1(i) / num_cp; // step is time duration / number of constraint points
              s1 = 0.0; // Time stamp
              double t = 0;
              // innerLoop = num_cp;

              for (int j = 0; j <= num_cp; ++j) // For each constraint point
              {
                  s2 = s1 * s1; // t^2
                  s3 = s2 * s1; // t^3
                  s4 = s2 * s2; // t^4
                  s5 = s4 * s1; // t^5
                  beta0 << 1.0, s1, s2, s3, s4, s5;
                  
                  // pos = c_5*(t**5) + c_4*(t**4) + c_3*(t**3) + c_2*(t**2) + c_1*(t) + c_0
                  pos = c.transpose() * beta0;
                  pts.col(i_dp) = pos; 

                  s1 += step;

                  // Next point IF not last point in segment
                  //            OR is last point in segment AND LAST segment

                  // Don't move on IF last point in segment and not LAST segment
                  if (j != num_cp || (j == num_cp && i == N - 1))
                  {
                      ++i_dp;
                  }
              }
          }

          return pts;
      }



      // /**
      //  * @brief Get the cost of jerk for the entire trajectory 
      //  * 
      //  * @return double 
      //  */
      // double getTrajJerkCostNew() const
      // {
      //     double objective = 0.0;
      //     for (int i = 0; i < N; i++) // for each segment
      //     {

      //         // objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +          // 36  * c3^2     * t
      //         //              144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) + // 144 * c3 * c4  * t^2
      //         //              192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +         // 192 * c4^2     * t^3 
      //         //              240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) + // 240 * c3 * c5  * t^3  
      //         //              720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) + // 720 * c4 * c5  * t^4 
      //         //              720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);          // 720 * c5^2     * t^5 

      //         Eigen::MatrixXd b_i = b.block<6, 3>(6 * i, 0); // Coefficient of 3 axes (x,y,z) at segment i
              
      //         // Cost matrix Q is:
      //         //      0   0   0   0   0   0
      //         //      0   0   0   0   0   0
      //         //      0   0   0   0   0   0
      //         //      0   0   0  36  72 120
      //         //      0   0   0  72 192 360
      //         //      0   0   0 120 360 720

      //         objective += ( b_i.transpose() * constructQ(5, 3, T1(i)) * b_i ).trace(); 
      //     }

      //     return objective;
      // }

      // /**
      //  * @brief Construct the cost matrix for minimizing the k-th derivative. Used as part of 
      //  * J_m = integral of f'(t)^2 over t(m) to t(m-1) = C_m.T * Q_m * C_m
      //  * 
      //  * @param n Polynomial order (Snap uses 7th order polynomial, jerk uses 5th order polynomial)
      //  * @param k Derivative to construct cost for (snap is 4, jerk is 3)
      //  * @param T Time duration
      //  * @return Eigen::MatrixXd 
      //  */
      // Eigen::MatrixXd constructQ(const int& n, const int& k, const double& T) const 
      // {
      //     Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n+1, n+1);

      //     for (int i = k; i < n+1; i++){ // for each derivative of order ith 
      //         for (int j = i; j < n+1; j++){ // for each derivative of other kth
      //             int c = i + j - (2*k - 1);
      //             Q(i, j) = (factorial<float>(i) / factorial<float>(i-k)) * (factorial<float>(j) / factorial<float>(j-k)) * std::pow(T,c) / c ;
      //             Q(j, i) = Q(i, j);
      //         }
      //     }

      //     return Q;
      // }



      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

}  // namespace traj_opt

#endif  // MINCO_TRAJ_GEN__MINCO_TRAJ_GEN_HPP_
