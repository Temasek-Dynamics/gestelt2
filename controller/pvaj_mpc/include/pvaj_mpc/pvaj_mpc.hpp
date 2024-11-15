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

#include <algorithm>

#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"

namespace pvaj_mpc
{

	/* QP formulation:
		min 1/2* x^T H x + f^T x   subject to
		b <= Ax <= b (Ax = b),  
		d <= Ax <= f,  
		l <= x <= u
	*/
	struct mpc_osqp_t
	{
		Eigen::MatrixXd Ax; // State transition matrix

		Eigen::MatrixXd Bx; // Control matrix

		/**
		 * Matrix M
		 *
		 * 		A
		 * 		A^2
		 * 		A^3
		 * 		...
		 * 		A^(N-1)
		 */
		Eigen::MatrixXd M;	// State transition matrix 

		/**
		 * Matrix C
		 *
		 * 		B
		 * 		AB		B
		 * 		A^2B	AB	 B
		 * 		...				...
		 * 		A^(N-1) ...			 B
		 */
		Eigen::MatrixXd C;	// Maps from control u to state x, i.e. x_t+1 = M * x_t + C * u_t

		/**
		 * Matrix Q_bar
		 *
		 * 		Q
		 * 			Q
		 * 			   ...
		 * 					 Q
		 */
		Eigen::MatrixXd Q_bar;	// Weight matrix across all iterations

		/**
		 * Matrix R_bar
		 *
		 * 		R
		 * 			R
		 * 			   ...
		 * 					 R
		 */
		Eigen::MatrixXd R_bar; // Weight for control magnitude 
		
		/**
		 * Matrix R_bar
		 *
		 * 		R_c
		 * 		-2R_c  2R_c 
		 * 			   -2R_c  2R_c
		 * 							...
		 * 								 -2R_c  R_c
		 */
		Eigen::MatrixXd R_con_bar; // Weight for change in controls

		Eigen::VectorXd u_low, u_upp, a_low, a_upp, v_low, v_upp;	// Vector of lower and upper bounds on state and controls
		Eigen::MatrixXd A_a, A_v, A_p; // Transform matrix from input u to states p, v, a 
		Eigen::MatrixXd M_a, M_v, M_p; // Transformation matrix from initial state x_0 to next state (using only matrix A)
		Eigen::VectorXd B_a, B_v, B_p;	// States p,v,a from applying transformation M_a, M_v, M_p to x_0

		/* Coefficients of hyperplane ax + by + cx = d */
		Eigen::MatrixXd T; // Each row is coefficient of plane normal vector (a, b, c)
		Eigen::VectorXd D_T; // coefficient d 

		Eigen::MatrixXd A_sys; // Transform from input u ot states p, v, a [A_u; A_a; A_v ]
		Eigen::VectorXd A_sys_low, A_sys_upp; // Vector of lower and upper bound [u_upp, a_upp - B_u, v_upp, - B_v]
		Eigen::MatrixXd A_sfc;
		Eigen::VectorXd A_sfc_low, A_sfc_upp;

		/* Data to be used in solver */ 
		Eigen::MatrixXd H;// Hessian for control inputs as in 1/2* u^T H u + f^T u 
		Eigen::VectorXd f; // Gradient for inputs as in 1/2* u^T H u + f^T u 
		Eigen::MatrixXd A; // Non-sparse linear constraint matrix as in l <= Ax <= u (Includes safe flight corridor bound)
		Eigen::VectorXd Alow, Aupp; // Lower and Upper bound as in l <= Ax <= u (Includes safe flight corridor bound)
		Eigen::SparseMatrix<double> H_sparse; // Sparse Hessian matrix
		Eigen::SparseMatrix<double> A_sparse; // Sparse linear constraint matrix as in l <= Ax <= u

		Eigen::VectorXd u_optimal; // Optimal controls
	};

	struct MPCControllerParams
	{
		int drone_id{-1};

		// Controller params
		int MPC_HORIZON{15};	// Planning horizon
		double MPC_STEP{0.1}; // [s] MPC time step

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
	};

	class MPCController
	{
	public:
		MPCController(const MPCControllerParams &params)
		{
			initParams(params);

			setProblem();

			X_0_.resize(mpc_.M.cols(), 1);
			X_r_.resize(mpc_.M.rows(), 1);
			planes_.resize(MPC_HORIZON);
		}

		/**
		 * @brief Initialize parameteres for MPC problem
		 * 
		 * @param params 
		 */
		void initParams(const MPCControllerParams &params)
		{
			drone_id_ = params.drone_id;

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
			Drag_ = params.Drag;

			// State bounds
			v_min_ = params.v_min;
			v_max_ = params.v_max;
			a_min_ = params.a_min;
			a_max_ = params.a_max;
			// Control bounds
			u_min_ = params.u_min;
			u_max_ = params.u_max;
		}

		/**
		 * @brief Run the solver
		 * 
		 * @return true 
		 * @return false 
		 */
		bool run(void)
		{

			// update reference
			setLinearTerm(mpc_, X_0_, X_r_);

			// update system status and their constraints
			UpdateBound(mpc_, X_0_);

			// update safe flight corridor constraints	
			bool add_sfc_flag = mpc_.T.rows() == 0 ? false : true;

			// generate all inequality constraints
			if (add_sfc_flag) {
				// Update SFC matrix
				mpc_.A_sfc.resize(mpc_.T.rows(), mpc_.A_p.cols());
				mpc_.A_sfc_upp.resize(mpc_.D_T.rows(), 1);
				// Set SFC upper bound
				// T * A_p <= -D_T -T * B_p
				mpc_.A_sfc = mpc_.T * mpc_.A_p;
				mpc_.A_sfc_upp = -mpc_.D_T - mpc_.T * mpc_.B_p;

				// Update entire matrix
				mpc_.A.resize(mpc_.A_sys.rows() + mpc_.A_sfc.rows(), mpc_.A_sys.cols());
				mpc_.A.block(0, 0, mpc_.A_sys.rows(), mpc_.A_sys.cols()) = mpc_.A_sys;
				mpc_.A.block(mpc_.A_sys.rows(), 0, mpc_.A_sfc.rows(), mpc_.A_sfc.cols()) = mpc_.A_sfc;

				mpc_.Alow.resize(mpc_.A_sys_low.rows() + mpc_.A_sfc_low.rows(), 1);
				mpc_.Alow.block(0, 0, mpc_.A_sys_low.rows(), 1) = mpc_.A_sys_low;
				mpc_.Alow.block(mpc_.A_sys_low.rows(), 0, mpc_.A_sfc_low.rows(), 1) = mpc_.A_sfc_low; // Set as -OSQP_INFTY

				mpc_.Aupp.resize(mpc_.A_sys_upp.rows() + mpc_.A_sfc_upp.rows(), 1);
				mpc_.Aupp.block(0, 0, mpc_.A_sys_upp.rows(), 1) = mpc_.A_sys_upp;
				mpc_.Aupp.block(mpc_.A_sys_upp.rows(), 0, mpc_.A_sfc_upp.rows(), 1) = mpc_.A_sfc_upp;
			} 
			else {
				mpc_.A.resize(mpc_.A_sys.rows(), mpc_.A_sys.cols());
				mpc_.A.block(0, 0, mpc_.A_sys.rows(), mpc_.A_sys.cols()) = mpc_.A_sys;

				mpc_.Alow.resize(mpc_.A_sys_low.rows(), 1);
				mpc_.Alow.block(0, 0, mpc_.A_sys_low.rows(), 1) = mpc_.A_sys_low;

				mpc_.Aupp.resize(mpc_.A_sys_upp.rows(), 1);
				mpc_.Aupp.block(0, 0, mpc_.A_sys_upp.rows(), 1) = mpc_.A_sys_upp;
			}

			mpc_.H_sparse = mpc_.H.sparseView();
			mpc_.A_sparse = mpc_.A.sparseView();

			// data reset
			mpc_.T.resize(0, 0);
			mpc_.A_sfc.resize(0, 0);
			mpc_.A_sfc_low.resize(0, 1);
			mpc_.A_sfc_upp.resize(0, 1);
			mpc_.D_T.resize(0, 1);

			// use osqp-eigen to solve MPC problem
			OsqpEigen::Solver solver;
			// solver.settings()->setTimeLimit(0.008);
			solver.settings()->setVerbosity(0); // osqp stop print
			solver.settings()->setWarmStart(true);
			// solver.setWarmStart()
			solver.data()->setNumberOfConstraints(mpc_.A_sparse.rows());
			solver.data()->setNumberOfVariables(mpc_.f.rows());
			solver.data()->setHessianMatrix(mpc_.H_sparse);
			solver.data()->setGradient(mpc_.f);
			solver.data()->setLinearConstraintsMatrix(mpc_.A_sparse);
			solver.data()->setLowerBound(mpc_.Alow);
			solver.data()->setUpperBound(mpc_.Aupp);


			bool init_flag = solver.initSolver();
			OsqpEigen::ErrorExitFlag solve_flag;

			if (init_flag) { // Solver initialized successfully
				solve_flag = solver.solveProblem();
			} 
			else {
				std::cerr << "[MPC]: Can't set mpc problem!" << std::endl;
			}

			// fps_++;
			
			if (solve_flag == OsqpEigen::ErrorExitFlag::NoError 
				&& init_flag == true) 
			{

				mpc_.u_optimal = solver.getSolution();

				// static double cur_vel = 0.0;
				// if (cur_vel < X_0_.block<3,1>(3,0).norm()) {
				// 	cur_vel = X_0_.block<3,1>(3,0).norm();
				// }
				// if ((ros::Time::now() - print_time_).toSec() > 2.0) {
				// 	print_time_ = ros::Time::now();
				// 	std::cout << "mpc fps: " << fps_/2 << ", this time is: " << (ros::Time::now()-time_0).toSec()*1000 << " ms. " 
				// 		"Velocity now is: " << cur_vel << "m/s. " << std::endl;
				// 	fps_ = 0;
				// }

				// std::cerr   << "[MPC] D" << drone_id_ << ": " <<
				// 				"Successful run()" << std::endl;

				return true;
			} 
			else { // FAILURE
				bool flag = isInFSC(X_0_.block<3,1>(0,0), planes_[0]);
				// std::cout << "planes: " << std::endl << planes_ << std::endl;
				std::cout << "pos: " << X_0_.block(0,0,3,1).transpose() << "  isInFSC: " << std::boolalpha << flag << std::endl;
				if (init_flag) {
					OsqpEigen::Status status = solver.getStatus();
					if (status == OsqpEigen::Status::DualInfeasibleInaccurate) {
						std::cerr << "[MPC]: Error status: Dual Infeasible Inaccurate" << std::endl;
					}
					if (status == OsqpEigen::Status::PrimalInfeasibleInaccurate) {
						std::cerr << "[MPC]: Error status: Primal Infeasible Inaccurate" << std::endl;
					}
					if (status == OsqpEigen::Status::SolvedInaccurate) {
						std::cerr << "[MPC]: Error status: Solved Inaccurate" << std::endl;
					}
					if (status == OsqpEigen::Status::Sigint) {
						std::cerr << "[MPC]: Error status: Sigint" << std::endl;
					}
					if (status == OsqpEigen::Status::MaxIterReached) {
						std::cerr << "[MPC]: Error status: Max Iter Reached" << std::endl;
					}
					if (status == OsqpEigen::Status::PrimalInfeasible) {
						std::cerr << "[MPC]: Error status: Primal Infeasible" << std::endl;
						// std::cout << "init state: " << X_0_.transpose() << std::endl;
					}
					if (status == OsqpEigen::Status::DualInfeasible) {
						std::cerr << "[MPC]: Error status: Dual Infeasible" << std::endl;
					}
					if (status == OsqpEigen::Status::NonCvx) {
						std::cerr << "[MPC]: Error status: NonCvx" << std::endl;
					}
				}
				return false;
			}
		}
		
		/**
		 * @brief Set safe flight corridor 
		 * 
		 * @param planes 
		 * @param step 
		 */
		void setFSC(Eigen::MatrixX4d& planes, int step)
		{
			if (step >= MPC_HORIZON || step < 0)
			{
				std::cerr << "[MPC]: Check sfc index! Error index: " << step << std::endl;
			}

			planes_[step] = planes;

			mpc_.T.conservativeResize(mpc_.T.rows() + planes.rows(), mpc_.A_p.rows());

			mpc_.A_sfc_low.conservativeResize(mpc_.A_sfc_low.rows() + planes.rows(), 1);

			mpc_.D_T.conservativeResize(mpc_.D_T.rows() + planes.rows(), 1);


			for (int i = 0; i < planes.rows(); i++) { // For each hyperplane
				mpc_.T.row(mpc_.T.rows()-1 - i).setZero();
				// Assign first 3 coefficients of plane, ax + by + cx
				mpc_.T.block(mpc_.T.rows()-1 - i, step*3, 1, 3) = Eigen::Vector3d(planes(i, 0), planes(i, 1), planes(i, 2)).transpose(); 

				// Set SFC lower bound 
				mpc_.A_sfc_low(mpc_.A_sfc_low.rows()-1 - i, 0) = -OSQP_INFTY;

				// Last coefficient of plane, d
				mpc_.D_T(mpc_.D_T.rows() - 1 - i, 0) = planes(i, 3);
			}


		}

		/**
		 * @brief Formulate problem
		 * 
		 */
		void setProblem(void)
		{
			// Set Initial state
			Eigen::VectorXd x_0, x_r;
			x_0.resize(9, 1);
			x_0.setZero();

			x_r.resize(9 * MPC_HORIZON, 1);
			x_r.setZero();

			/* 	states: {p1, v1, a1, p2, v2, a2, ... , pN, vN, aN}
				input: {u0, u1, u2, ... , u(N-1)}
			*/

			// getSystemModel: Set up the matrices Ax and Bc as part of linear dynamical model : x_k+1 = Ax * x_k + Bx * u_k
			getSystemModel(mpc_.Ax, mpc_.Bx, MPC_STEP);
			// setModel: Set linear dynamical model across mpc horizon iterations
			setModel(mpc_.Ax, mpc_.Bx, mpc_.M, mpc_.C);

			/**
			 * sum_{k=0}^{N-1} (x_k - x_r).T * Q * (x_k - x_r) 	// Intermediate states
			 * 	+ sum_{k=0}^{N-1} u_k.T * R * u_k				// Intermediate controls
			 * 	+ sum_{k=0}^{N-1} du_k.T * R_con * du_k			// Change in intermediate controls
			 *	+ (x_N - x_r).T * F * (x_N - x_r)				// Terminal state
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

			// Cost: 1/2* x^T H x + f^T x   
			// setQuadraticTerm: Constructs H matrix
			setQuadraticTerm(mpc_, Q, R, R_con, F);
			// setLinearTerm: Constructs f matrix
			setLinearTerm(mpc_, x_0, x_r);

			// system status and input constrains
			setConstraints(mpc_);

			// Update bounds on state
			UpdateBound(mpc_, x_0);

			// Inequality constrains (none)

			// Resize optimal controls
			mpc_.u_optimal.resize(mpc_.f.rows(), 1);
		}

		/**
		 * @brief Construct quadratic terms
		 * 
		 * Q: weight on intermediate states
		 * R: weight on intermediate controls
		 * R_con: weight on change in intermediate controls
		 * F: weight on terminal state
		 */
		void setQuadraticTerm(mpc_osqp_t &mpc,
						   const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
						   const Eigen::MatrixXd &R_con, const Eigen::MatrixXd &F)
		{
			/**
			 *  (x_k - x_r).T   * Q_bar * (x_k - x_r) 	// Intermediate states
			 * 	+ u_k.T 	    * R_bar * u_k								// Intermediate controls
			 * 	+ du_k.T        * R_con_bar * du_k							// Change in intermediate controls
			 */

			mpc.Q_bar.resize(Q.rows() * MPC_HORIZON, Q.cols() * MPC_HORIZON);
			mpc.R_bar.resize(R.rows() * MPC_HORIZON, R.cols() * MPC_HORIZON);
			mpc.R_con_bar.resize(R_con.rows() * MPC_HORIZON, R_con.cols() * MPC_HORIZON);

			mpc.Q_bar.setZero();
			mpc.R_bar.setZero();
			mpc.R_con_bar.setZero();

			for (int i = 0; i < MPC_HORIZON; i++)
			{
				// Populate Q_bar, R_bar and R_con_bar blocks with weights 
				mpc.Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
				mpc.R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
				if (i == 0) // First iteration
				{
					mpc.R_con_bar.block(0, 0, R_con.rows(), R_con.cols()) = R_con;
				}
				else if (i == MPC_HORIZON - 1) // last iteration
				{ 
					mpc.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
					mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = R_con;
				}
				else // Intermediate iteration
				{ 
					mpc.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
					mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = 2 * R_con;
				}
			}
			// Set Terminal state weights
			mpc.Q_bar.block((MPC_HORIZON - 1) * Q.rows(), (MPC_HORIZON - 1) * Q.cols(), F.rows(), F.cols()) = F;

			/* QP formulation:
				min 1/2* x^T H x + f^T x   
			*/

			// What is H? Weight for controls?

			mpc.H.resize(mpc.C.rows(), mpc.C.cols());
			// H = C.T * Q_bar * C + R + R_con
			mpc.H = mpc.C.transpose() * mpc.Q_bar * mpc.C + mpc.R_bar + mpc.R_con_bar;
		}

		/**
		 * @brief 
		 * 
		 * x_0: 
		 * x_r: reference state
		 */
		void setLinearTerm(mpc_osqp_t &mpc, const Eigen::VectorXd &x_0, const Eigen::VectorXd &x_r)
		{
			if (x_r.rows() != mpc.M.rows())
			{
				std::cerr << "[MPC]: MPC linear term set goal error!" << std::endl;
				return;
			}

			mpc.f.resize(mpc.C.rows(), 1);
			mpc.f = ((x_0.transpose() * mpc.M.transpose() - x_r.transpose()) * mpc.Q_bar * mpc.C).transpose();
		}

		/**
		 * @brief Set linear dynamical model : x_k+1 = Ax * x_k + Bx * u_k
		 *
		 * @param A Set State transtiion matrix
		 * @param B Set control input matrix
		 * @param t timestep
		 */
		void getSystemModel(Eigen::MatrixXd &A, Eigen::MatrixXd &B, double t)
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
		void setModel(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
					  Eigen::MatrixXd &M, Eigen::MatrixXd &C)
		{
			M.resize(MPC_HORIZON * 9, 9);
			M.setZero();
			C.resize(MPC_HORIZON * 9, MPC_HORIZON * 3);
			C.setZero();

			Eigen::MatrixXd temp = Eigen::MatrixXd(A.rows(), A.cols()).setIdentity();
			for (int i = 0; i < MPC_HORIZON; i++)
			{
				if (i == 0) // For first iteration
				{
					
					C.block(0, 0, B.rows(), B.cols()) = B;
				}
				else // For subsequent iterations
				{
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

		/**
		 * @brief Set constraints
		 */

		// TODO: Change name to setUpConstraints
		void setConstraints(mpc_osqp_t& mpc)
		{

			/* --system constraint: input, acceleration, velocity limit-- */
			mpc.u_low.resize(3 * MPC_HORIZON, 1);
			mpc.u_upp.resize(3 * MPC_HORIZON, 1);
			mpc.a_low.resize(3 * MPC_HORIZON, 1);
			mpc.a_upp.resize(3 * MPC_HORIZON, 1);
			mpc.v_low.resize(3 * MPC_HORIZON, 1);
			mpc.v_upp.resize(3 * MPC_HORIZON, 1);
			for (int i = 0; i < MPC_HORIZON; i++) {
				mpc.u_low.block(i * u_min_.rows(), 0, u_min_.rows(), 1) = u_min_;
				mpc.u_upp.block(i * u_max_.rows(), 0, u_max_.rows(), 1) = u_max_;
				mpc.a_low.block(i * a_min_.rows(), 0, a_min_.rows(), 1) = a_min_;
				mpc.a_upp.block(i * a_max_.rows(), 0, a_max_.rows(), 1) = a_max_;
				mpc.v_low.block(i * v_min_.rows(), 0, v_min_.rows(), 1) = v_min_;
				mpc.v_upp.block(i * v_max_.rows(), 0, v_max_.rows(), 1) = v_max_;
			}
			
			// input constraint
			Eigen::MatrixXd A_u;
			A_u.resize(3 * MPC_HORIZON, 3 * MPC_HORIZON);
			A_u.setIdentity();

			// calculate A_p, A_v, A_a: system state position p, v, a transform to input u
			mpc.A_p.resize(3 * MPC_HORIZON, mpc.C.cols());
			mpc.A_v.resize(3 * MPC_HORIZON, mpc.C.cols());
			mpc.A_a.resize(3 * MPC_HORIZON, mpc.C.cols());
			// calculate M_p, M_v, M_a: 
			mpc.M_p.resize(3 * MPC_HORIZON, mpc.M.cols());
			mpc.M_v.resize(3 * MPC_HORIZON, mpc.M.cols());
			mpc.M_a.resize(3 * MPC_HORIZON, mpc.M.cols());

			for (int i = 0; i < MPC_HORIZON; i++) {
				mpc.A_p.block(3 * i, 0, 3, mpc.A_p.cols()) = mpc.C.block(9 * i + 0, 0, 3, mpc.C.cols());
				mpc.A_v.block(3 * i, 0, 3, mpc.A_v.cols()) = mpc.C.block(9 * i + 3, 0, 3, mpc.C.cols());
				mpc.A_a.block(3 * i, 0, 3, mpc.A_a.cols()) = mpc.C.block(9 * i + 6, 0, 3, mpc.C.cols());

				mpc.M_p.block(3 * i, 0, 3, mpc.M_p.cols()) = mpc.M.block(9 * i + 0, 0, 3, mpc.M.cols());
				mpc.M_v.block(3 * i, 0, 3, mpc.M_v.cols()) = mpc.M.block(9 * i + 3, 0, 3, mpc.M.cols());
				mpc.M_a.block(3 * i, 0, 3, mpc.M_a.cols()) = mpc.M.block(9 * i + 6, 0, 3, mpc.M.cols());
			}

			// Transformation from control input to states
			mpc.A_sys.resize(3 * MPC_HORIZON * 3, 3 * MPC_HORIZON);

			mpc.A_sys.block(0, 0, A_u.rows(), A_u.cols()) = A_u;
			mpc.A_sys.block(A_u.rows(), 0, mpc.A_a.rows(), mpc.A_a.cols()) = mpc.A_a;
			mpc.A_sys.block(A_u.rows() + mpc.A_a.rows(), 0, mpc.A_v.rows(), mpc.A_v.cols()) = mpc.A_v;

			// std::cout << "mpc.A_a: " << std::endl << mpc.A_a << std::endl;
			// std::cout << "mpc.A_v: " << std::endl << mpc.A_v << std::endl;
			// std::cout << "mpc.A_p: " << std::endl << mpc.A_p << std::endl;

			//Resize
			mpc.B_a.resize(mpc.M_p.rows(), 1);
			mpc.B_v.resize(mpc.M_v.rows(), 1);
			mpc.B_p.resize(mpc.M_a.rows(), 1);

			mpc.A_sys_low.resize(3 * MPC_HORIZON * 3, 1);
			mpc.A_sys_upp.resize(3 * MPC_HORIZON * 3, 1);
		}

		/**
		 * @brief Update bounds on state d <= Ax <= f
		 * 
		 * @param mpc 
		 * @param x_0 
		 */
		void UpdateBound(mpc_osqp_t& mpc, const Eigen::VectorXd& x_0)
		{
			if (x_0.rows() != mpc.M_p.cols() 
				|| x_0.rows() != mpc.M_v.cols() 
				|| x_0.rows() != mpc.M_a.cols()) 
			{
				std::cerr << "[MPC]: Update bound error!" << std::endl;
				return;
			}

			// Update bounds on position, velocity, and acceleration
			mpc.B_p = mpc.M_p * x_0;
			mpc.B_v = mpc.M_v * x_0;
			mpc.B_a = mpc.M_a * x_0;

			// Update control input
			mpc.A_sys_low.block(0, 0, mpc.u_low.rows(), 1) = mpc.u_low;
			mpc.A_sys_upp.block(0, 0, mpc.u_upp.rows(), 1) = mpc.u_upp;
			// Update acceleration 
			mpc.A_sys_low.block(mpc.u_low.rows(), 0, mpc.a_low.rows(), 1) = mpc.a_low - mpc.B_a;
			mpc.A_sys_upp.block(mpc.u_upp.rows(), 0, mpc.a_upp.rows(), 1) = mpc.a_upp - mpc.B_a;
			// Update velocity
			mpc.A_sys_low.block(mpc.u_low.rows() + mpc.a_low.rows(), 0, mpc.v_low.rows(), 1) = mpc.v_low - mpc.B_v;
			mpc.A_sys_upp.block(mpc.u_upp.rows() + mpc.a_upp.rows(), 0, mpc.v_upp.rows(), 1) = mpc.v_upp - mpc.B_v;

			// std::cout << "mpc.B_a: " << mpc.B_a.transpose() << std::endl;
			// std::cout << "mpc.B_v: " << mpc.B_v.transpose() << std::endl;
			// std::cout << "mpc.B_p: " << mpc.B_p.transpose() << std::endl;
		}

		/* Getters */

		/**
		 * @brief Get the optimal control at a given segment
		 * 
		 * @param u Control to be modified in place
		 * @param segment control segment
		 */
		void getOptimalControl(Eigen::Vector3d& u, int segment) {
			
			segment = segment >= MPC_HORIZON ? MPC_HORIZON - 1 : segment;

			u = mpc_.u_optimal.block<3,1>(segment * 3, 0);
		}

		/* Setters */

		/**
		 * @brief Set reference PVA for given step
		 * 
		 * @param pr position 
		 * @param vr velocity
		 * @param ar acceleration
		 * @param step 
		 */
		void setReference(Eigen::Vector3d pr, Eigen::Vector3d vr, Eigen::Vector3d ar, int step) {
			if (step > MPC_HORIZON || step < 0) {
				std::cerr << "[MPC]: Check goal index! Error index: " <<  step << std::endl;
				return;
			}

			Eigen::VectorXd x_0(9, 1);
			x_0.block<3, 1>(0, 0) = pr;
			x_0.block<3, 1>(3, 0) = vr;
			x_0.block<3, 1>(6, 0) = ar;
			// Set reference PVA at given step
			X_r_.block<9, 1>(9*step, 0) = x_0;
		}

		/**
		 * @brief Set initial condition
		 * 
		 * @param p0 
		 * @param v0 
		 * @param a0 
		 */
		void setInitialCondition(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0) {
			saturateReference(v0, a0);
			X_0_.block(0, 0, p0.rows(), 1) = p0;
			X_0_.block(p0.rows(), 0, v0.rows(), 1) = v0;
			X_0_.block(p0.rows()+v0.rows(), 0, a0.rows(), 1) = a0;
		}

		/* Checks */

		/**
		 * @brief Is in safe flight corridor
		 * 
		 * @param pos Position to check
		 * @param planes Set of hyperplanes
		 * @return true In safe flight corridor 
		 * @return false Outside safe flight corridor
		 */
		bool isInFSC(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes)
		{
			if (planes.rows() == 0) {
				return false;
			}
			
			for (int i = 0; i < planes.rows(); i++) {
				if (pos.x()*planes(i,0) 
					+ pos.y()*planes(i,1) 
					+ pos.z()*planes(i,2) 
					+ planes(i,3) > 0) // fulfilling this means on wrong side of plane (outside SFC)
				{ 
					// std::cout << "plane:" << i << " " << planes_.block<1, 4>(i, 0) << std::endl;
					return false;
				}
			}

			return true;
		}

		/**
		 * @brief Clamp velocity and acceleration values to be within min and max bounds
		 * 
		 * @param v0 
		 * @param a0 
		 */
		void saturateReference(Eigen::Vector3d& v0, Eigen::Vector3d& a0) {
			for (int i = 0; i < 3; i++) {
				v0(i, 0) = std::clamp(v0(i, 0), v_min_(i, 0), v_max_(i, 0));
				a0(i, 0) = std::clamp(a0(i, 0), a_min_(i, 0), a_max_(i, 0));
			}
		}


	public:
		mpc_osqp_t mpc_; // MPC data
		// int fps_;

		// Controller params
		int MPC_HORIZON{5};
		double MPC_STEP{0.1}; // [s] time step for dynamics
		
		Eigen::VectorXd X_0_, X_r_; // Current position and reference position

	private:
		int drone_id_{-1};

		// Vector of set of hyper planes, (n, 4) sized matrix with each row 
		// containing hyperplane coefficients. Each element is a different step 
		// of the MPC problem
	    std::vector<Eigen::MatrixX4d> planes_; 

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

	}; // class MPCController




} // namespace pvaj_mpc

#endif // PVAJ_MPC_HPP_
