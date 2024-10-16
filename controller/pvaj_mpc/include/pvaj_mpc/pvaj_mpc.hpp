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

#ifndef PVAJ_MPC_HPP_
#define PVAJ_MPC_HPP_

#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"

namespace pvaj_mpc
{

	/* QP formulation:
		min 1/2* x^T H x + f^T x   subject to
		b <= Ax <= b (Ax = b),  d <= Ax <= f,  l <= x <= u
	*/
	struct mpc_osqp_t
	{
		Eigen::MatrixXd Ax; // State transition matrix
		Eigen::MatrixXd Bx; // Control matrix

		/**
		 * Matrix M
		 * 		A
		 * 		A^2
		 * 		A^3
		 * 		...
		 * 		A^(N-1)
		 */
		Eigen::MatrixXd M;	// State transition matrix across all iterations

		/**
		 * Matrix C
		 *
		 * 		B
		 * 		AB		B
		 * 		A^2B	AB	 B
		 * 		...				...
		 * 		A^(N-1) ...			 B
		 */
		Eigen::MatrixXd C;				  // State transition matrix across all iterations
		Eigen::MatrixXd Q_bar;			  // Weight matrix across all iterations
		Eigen::MatrixXd R_bar, R_con_bar; // Weight matrix across all iterations

		Eigen::VectorXd u_low, u_upp, a_low, a_upp, v_low, v_upp;
		Eigen::VectorXd B_a, B_v, B_p;
		Eigen::MatrixXd A_a, A_v, A_p;
		Eigen::MatrixXd M_a, M_v, M_p;

		Eigen::MatrixXd T;
		Eigen::VectorXd D_T;

		Eigen::MatrixXd A_sys;
		Eigen::VectorXd A_sys_low, A_sys_upp;
		Eigen::MatrixXd A_sfc;
		Eigen::VectorXd A_sfc_low, A_sfc_upp;

		Eigen::MatrixXd H;
		Eigen::VectorXd f;
		Eigen::MatrixXd A;
		Eigen::VectorXd Alow, Aupp;
		Eigen::SparseMatrix<double> H_sparse;
		Eigen::SparseMatrix<double> A_sparse;

		Eigen::VectorXd u_optimal;
	};

	struct MPCControllerParams
	{
		// Controller params
		int MPC_HORIZON{5};
		double MPC_STEP{0.1};

		// Dynamical parameters
		Eigen::Matrix3d Drag;

		// Objective Weights
		double R_p{1000.0};	 // Intermediate Position
		double R_v{0.0};	 // Intermediate velocity
		double R_a{0.0};	 // Intermediate acceleration
		double R_u{0.0};	 // Controls
		double R_u_con{0.2}; // Change in controls
		double R_pN{2000.0}; // Terminal position
		double R_vN{1000.0}; // Terminal velocity
		double R_aN{1000.0}; // Terminal acceleration

		// State bounds
		Eigen::Vector3d v_min{-10.0, -10.0, -10.0};
		Eigen::Vector3d v_max{10.0, 10.0, 10.0};
		Eigen::Vector3d a_min{-20.0, -20.0, -20.0};
		Eigen::Vector3d a_max{20.0, 20.0, 20.0};
		// Control bounds
		Eigen::Vector3d u_min{-50.0, -50.0, -50.0};
		Eigen::Vector3d u_max{50.0, 50.0, 50.0};
	}

	class MPCController
	{
	public:
		MPCController(const MPCControllerParams &params)
		{
			initParams(params);

			ProblemFormation();
			X_0_.resize(mpc_.M.cols(), 1);
			X_r_.resize(mpc_.M.rows(), 1);
			planes_.resize(MPC_HORIZON);
		}

		void ProblemFormation(void)
		{
			/* 	states: {p1, v1, a1, p2, v2, a2, ... , pN, vN, aN}
				input: {u0, u1, u2, ... , u(N-1)}
			*/

			// system model
			SystemModel(mpc_.Ax, mpc_.Bx, MPC_STEP);
			MPCModel(mpc_.Ax, mpc_.Bx, mpc_.M, mpc_.C);

			/**
			 * (x_N - x_r).T * F * (x_N - x_r)
			 * 		+ sum_{k=0}^{N-1} (x_k - x_r).T * Q * (x_k - x_r) + u_k.T * R * u_k
			 */

			// cost function (Quadratic term and Linear term)
			Eigen::MatrixXd Q = Eigen::MatrixXd(9, 9).setZero();
			Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * R_p_; // Intermediate Position
			Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * R_v_; // Intermediate Velocity
			Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * R_a_; // Intermediate Acceleration

			Eigen::MatrixXd R = Eigen::MatrixXd(3, 3).setIdentity() * R_u_;			// Controls
			Eigen::MatrixXd R_con = Eigen::MatrixXd(3, 3).setIdentity() * R_u_con_; // Change in controls

			// Terminal state weights
			Eigen::MatrixXd F = Eigen::MatrixXd(9, 9).setZero();
			F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * R_pN_; // Terminal Position
			F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * R_vN_;
			F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * R_aN_;

			QuadraticTerm(mpc_, Q, R, R_con, F);
			LinearTerm(mpc_, Eigen::VectorXd(9, 1).setZero(), 
						Eigen::VectorXd(9 * MPC_HORIZON, 1).setZero());

			// system status and input constrains
			ALLConstraint(mpc_);

			// Inequality constrains (none)

			mpc_.u_optimal.resize(mpc_.f.rows(), 1);

			std::cout << "[MPC] Problem formulated" << std::endl;
		}

		void QuadraticTerm(mpc_osqp_t &mpc,
						   const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
						   const Eigen::MatrixXd &R_con, const Eigen::MatrixXd &F)
		{
			mpc.Q_bar.resize(Q.rows() * MPC_HORIZON, Q.cols() * MPC_HORIZON);
			mpc.R_bar.resize(R.rows() * MPC_HORIZON, R.cols() * MPC_HORIZON);
			mpc.R_con_bar.resize(R_con.rows() * MPC_HORIZON, R_con.cols() * MPC_HORIZON);

			mpc.Q_bar.setZero();
			mpc.R_bar.setZero();
			mpc.R_con_bar.setZero();

			for (int i = 0; i < MPC_HORIZON; i++)
			{
				mpc.Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
				mpc.R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
				if (i == 0) // First iteration
					mpc.R_con_bar.block(0, 0, R_con.rows(), R_con.cols()) = R_con;
				else if (i == MPC_HORIZON - 1)
				{ // Second last iteration
					mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = R_con;
					mpc.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
				}
				else
				{ // Intermediate iteration
					mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = 2 * R_con;
					mpc.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
				}
			}
			mpc.Q_bar.block((MPC_HORIZON - 1) * Q.rows(), (MPC_HORIZON - 1) * Q.cols(), F.rows(), F.cols()) = F;

			mpc.H.resize(mpc.C.rows(), mpc.C.cols());
			mpc.H = mpc.C.transpose() * mpc.Q_bar * mpc.C + mpc.R_bar + mpc.R_con_bar;
		}

		void LinearTerm(mpc_osqp_t &mpc, const Eigen::VectorXd &x_0,
						const Eigen::VectorXd &x_r)
		{
			if (x_r.rows() != mpc.M.rows())
			{
				std::cout << "[MPC]: MPC linear term set goal error!" << std::endl;
				return;
			}
			mpc.f.resize(mpc.C.rows(), 1);
			mpc.f = ((x_0.transpose() * mpc.M.transpose() - x_r.transpose()) * mpc.Q_bar * mpc.C).transpose();
		}

		/**
		 * @brief Set linear dynamical model : x_k+1 = Ax * x_k + Bx * u_k
		 *
		 * @param A
		 * @param B
		 * @param t
		 */
		void SystemModel(Eigen::MatrixXd &A, Eigen::MatrixXd &B, double t)
		{
			// Ax * {px, py, pz, vx, vy, vz, ax, ay, az}
			// + Bx * {ux, uy, uz}

			// Block of size (p,q), starting at (i,j)
			// 		matrix.block(i,j,p,q);
			//		matrix.block<p,q>(i,j);

			A.resize(9, 9);
			A.setZero();
			A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
			A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * t;
			A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * t * t * 0.5;
			A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (Eigen::Matrix3d::Identity() - Drag_ * t);
			A.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * t;
			A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

			B.resize(9, 3);
			B.setZero();
			B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0 / 6.0 * std::pow(t, 3);
			B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * 1.0 / 2.0 * std::pow(t, 2);
			B.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity() * t;
		}

		/**
		 * @brief Set linear dynamical model across mpc horizon iterations
		 *
		 * @param A
		 * @param B
		 * @param M
		 * @param C
		 */
		void MPCModel(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
					  Eigen::MatrixXd &M, Eigen::MatrixXd &C)
		{
			M.resize(MPC_HORIZON * A.rows(), A.cols());
			M.setZero();
			C.resize(MPC_HORIZON * B.rows(), MPC_HORIZON * B.cols());
			C.setZero();

			Eigen::MatrixXd temp = Eigen::MatrixXd(A.rows(), A.cols()).setIdentity();
			for (int i = 0; i < MPC_HORIZON; i++)
			{
				if (i == 0)
				{
					// For first iteration
					C.block(0, 0, B.rows(), B.cols()) = B;
				}
				else
				{
					// For subsequent iterations
					Eigen::MatrixXd temp_c = Eigen::MatrixXd(B.rows(), C.cols());

					// Take rows from previous iteration (i-1)
					temp_c << temp * B, C.block((i - 1) * B.rows(), 0,
												B.rows(), B.cols() * (MPC_HORIZON - 1));
					C.block(B.rows() * i, 0, B.rows(), C.cols()) = temp_c;
				}

				temp = temp * A;
				M.block(A.rows() * i, 0, A.rows(), A.cols()) = temp;
			}
		}

		initParams(const MPCControllerParams &params)
		{
			// MPC parameters
			MPC_HORIZON = params.MPC_HORIZON;
			MPC_STEP = params.MPC_STEP;

			// Objective Weights
			R_p_ = params.R_p;
			R_v_ = params.R_v;
			R_a_ = params.R_a;
			R_u_ = params.R_u;
			R_u_con_ = params.R_u_con;
			R_pN_ = params.R_pN;
			R_vN_ = params.R_vN;
			R_aN_ = params.R_aN;

			// Dynamical Parameters
			Drag_ = params.Drag_;

			// State bounds
			v_min_ = params.v_min;
			v_max_ = params.v_max;
			a_min_ = params.a_min;
			a_max_ = params.a_max;
			// Control bounds
			u_min_ = params.u_min;
			u_max_ = params.u_max;
		}

	private:
		mpc_osqp_t mpc_; // MPC data

	private:
		// Controller params
		int MPC_HORIZON{5};
		double MPC_STEP{0.1}; // [s] time step for dynamics

		// Dynamical parameters
		Eigen::Matrix3d Drag_;

		// Objective Weights
		double R_p_{1000.0};  // Intermediate Position
		double R_v_{0.0};	  // Intermediate velocity
		double R_a_{0.0};	  // Intermediate acceleration
		double R_u_{0.0};	  // Controls
		double R_u_con_{0.2}; // Change in controls
		double R_pN_{2000.0}; // Terminal position
		double R_vN_{1000.0}; // Terminal velocity
		double R_aN_{1000.0}; // Terminal acceleration

		// State Bounds
		Eigen::Vector3d v_min_{-10.0, -10.0, -10.0};
		Eigen::Vector3d v_max_{10.0, 10.0, 10.0};
		Eigen::Vector3d a_min_{-20.0, -20.0, -20.0};
		Eigen::Vector3d a_max_{20.0, 20.0, 20.0};
		// Control bounds
		Eigen::Vector3d u_min_{-50.0, -50.0, -50.0};
		Eigen::Vector3d u_max_{50.0, 50.0, 50.0};
	}

} // namespace pvaj_mpc

#endif // PVAJ_MPC_HPP_
