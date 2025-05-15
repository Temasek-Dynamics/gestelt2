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

#include <iostream>
#include <ctime>
#include <random>
#include <math.h>
#include <chrono>
#include <thread>

#include <filesystem>

#include <Eigen/Eigen>

#include <nlohmann/json.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;
using json = nlohmann::json;

/* Structure representing a cylinder */
struct Cylinder 
{	
	Cylinder(const double& x, const double& y, const double& radius, const double& height): 
		x(x), y(y), radius(radius), height(height){};

	Eigen::Vector2d position() const {
		return Eigen::Vector2d(x, y);
	}
	double x;
	double y;
	double radius;
	double height;
};

/* Square used to denote an obstacle free area*/
struct ObsFreeSquare
{
	ObsFreeSquare(const double& _min_x, const double& _max_x, const double& _min_y, const double& _max_y): 
		min_x(_min_x), min_y(_min_y), max_x(_max_x), max_y(_max_y){};

	bool isInObsFree(const double& x, const double& y){
		return (x > this->min_x && x < this->max_x)
					&& (y > this->min_y && y < this->max_y); 
	}

	double min_x, max_x;
	double min_y, max_y;
};

/* Circle used to denote an obstacle free area*/
struct ObsFreeRadial
{ // Encapsulates a circular region where obstacles cannot be placed
	ObsFreeRadial(const double& x, const double& y, const double radius): 
		x(x), y(y), radius(radius) {};

	bool isInObsFree(const double& x, const double& y){
		return fabs(x - this->x) < radius && fabs(y - this->y) < radius; 
	}

	double x, y, radius;
};

class PCDMapGen : public rclcpp::Node
{
	public:

		PCDMapGen() : Node("pcd_map_generator")
		{
			this->declare_parameter("map.filename", "new_map");
			this->declare_parameter("map.filepath", "");
			this->declare_parameter("map.resolution", 0.05);
			this->declare_parameter("map.x_size", 5.0);
			this->declare_parameter("map.y_size", 5.0);
			this->declare_parameter("map.z_size", 5.0);

			// Random forest params
			this->declare_parameter("random_forest.num_cylindrical_obs", -1);
			this->declare_parameter("random_forest.num_hoop_obs", -1);
			this->declare_parameter("random_forest.rnd_seed", -1);

			// Params for cylinders
			this->declare_parameter("cylinder.lower_radius", 0.3);
			this->declare_parameter("cylinder.upper_radius", 0.8);
			this->declare_parameter("cylinder.lower_height", 3.0);
			this->declare_parameter("cylinder.upper_height", 7.0);
			this->declare_parameter("cylinder.inter_obstacle_clearance", 1.0);
			// Params for circles
			this->declare_parameter("hoop.lower_radius", 7.0);
			this->declare_parameter("hoop.upper_radius", 7.0);
			this->declare_parameter("hoop.lower_height", 7.0);
			this->declare_parameter("hoop.upper_height", 7.0);
			this->declare_parameter("hoop.theta", 7.0);

			// Params for map
			filename_ = this->get_parameter("map.filename").as_string();
			filepath_ = this->get_parameter("map.filepath").as_string();
			map_res_ = this->get_parameter("map.resolution").as_double();
			map_x_size_ = this->get_parameter("map.x_size").as_double();
			map_y_size_ = this->get_parameter("map.y_size").as_double();
			map_z_size_ = this->get_parameter("map.z_size").as_double();

			cylindrical_obs_num_ = this->get_parameter("random_forest.num_cylindrical_obs").as_int();
			hoop_obs_num_ = this->get_parameter("random_forest.num_hoop_obs").as_int();
			seed_ = this->get_parameter("random_forest.rnd_seed").as_int();

			// Params for cylinders
			cyl_lower_rad_ = this->get_parameter("cylinder.lower_radius").as_double();
			cyl_upper_rad_ = this->get_parameter("cylinder.upper_radius").as_double();
			cyl_lower_height_ = this->get_parameter("cylinder.lower_height").as_double();
			cyl_upper_height_ = this->get_parameter("cylinder.upper_height").as_double();
			inter_obs_clearance_ = this->get_parameter("cylinder.inter_obstacle_clearance").as_double();

			// Params for hoops
			hoop_lower_rad_ = this->get_parameter("hoop.lower_radius").as_double();
			hoop_upper_rad_ = this->get_parameter("hoop.upper_radius").as_double();
			hoop_lower_height_ = this->get_parameter("hoop.lower_height").as_double();
			hoop_upper_height_ = this->get_parameter("hoop.upper_height").as_double();
			theta_ = this->get_parameter("hoop.theta").as_double();

			map_min_x_ = -map_x_size_ / 2.0;	// Lower x bound
			map_max_x_ = map_x_size_ / 2.0; 	// Upper x bound

			map_min_y_ = -map_y_size_ / 2.0;	// Lower y bound
			map_max_y_ = map_y_size_ / 2.0;	// Upper y bound

			eng.seed(seed_);
			// unsigned int seed = rd();
		}

		~PCDMapGen(){}

		void createMap()
		{
			// Map initial setup
			AddXYPlaneToMap(map_x_size_, map_y_size_, 0.0); // Add floor
			AddXYPlaneToMap(map_x_size_, map_y_size_, map_z_size_); // Add ceiling

			// createSimpleTestMap();
			// createSimpleTestMap2();

			// // Create regions where obstacles cannot be created.
			// obs_free_radial_.push_back(ObsFreeRadial(0.0, 0.0, 0.1));

			// We can either:
			// A1) Generate a test map for the vicon room
			// createViconMap();
			addSingleCylinder(0.0, 0.0, 0.3, map_z_size_);
			addRoomBoundsToMap(map_x_size_, map_y_size_, map_z_size_,  //x_size, y_size, z_size
								-map_x_size_/2.0, map_x_size_/2.0, -map_y_size_/2.0, map_y_size_/2.0, map_z_size_); //rm_min_x, rm_max_x, rm_min_y, rm_max_y, rm_height

			// A2) Generate a antipodal map for the vicon room
			// createViconAntipodalMap();
			// addRoomBoundsToMap(	 5.9, 5.9, 3.0, 
			// 						-2.95, 2.95, -2.95, 2.95, 3.0);

			/* B) Generate forest map*/
			// createRandomForestMap();

			/* C) Generate a narrow window (1m x 1m) for benchmarking  */
			// double win_side_length = 1.0;
			// double win_top_btm_height= 1.0;
			// double win_length = 1.0;
			// double win_height = 1.0;
			// // Side wall along window
			// addWallToMap(Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{0.0, win_side_length}, 
			// 	0.0, 2 * win_top_btm_height + win_height);
			// addWallToMap(Eigen::Vector2d{0.0, win_side_length + win_length}, Eigen::Vector2d{0.0, 2*win_side_length + win_length}, 
			// 	0.0, 2 * win_top_btm_height + win_height);
			// // Top and bottom wall of window
			// addWallToMap(Eigen::Vector2d{0.0, win_side_length}, Eigen::Vector2d{0.0, win_side_length + win_length}, 
			// 	0.0, win_top_btm_height);
			// addWallToMap(Eigen::Vector2d{0.0, win_side_length}, Eigen::Vector2d{0.0, win_side_length + win_length}, 
			// 	win_top_btm_height + win_height, 2 * win_top_btm_height + win_height);
			// // Generate floor
			// AddXYPlaneToMap(20.0, 20.0, 0.0);	

			/* D) Generate anti-podal map*/
			// double a = 2.0;
			// double b = 2.0;
			// double buffer_rad = 0.3;

			// // Add all 4 origin positions of the drones + 0.5 buffer
			// obs_free_radial_.push_back(ObsFreeRadial(a+b, a+b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(a+b, -a-b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(-a-b, -a-b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(-a-b, a+b, b + buffer_rad));

			cloud_map_.width = cloud_map_.points.size();
			cloud_map_.height = 1;
			cloud_map_.is_dense = true;
		}

		/* Save map to PCD file*/
		void savePCDMap()
		{
			std::filesystem::path dir(this->get_directory());
			std::filesystem::path file(this->get_filename()+ ".pcd");

			std::filesystem::path full_path = dir / file;
			pcl::io::savePCDFileASCII(full_path.string(), cloud_map_);
			printf("Saved PCD map to path: %s\n",  full_path.c_str());
		}

		/* Save all generated obstacles into JSON file*/
		void saveObstacleJSON()
		{
			std::filesystem::path dir(this->get_directory());
			std::filesystem::path file(this->get_filename()+ "_obs.json");
			std::filesystem::path full_path = dir / file;

			json obs_json;

			for (Cylinder& cyl: rnd_gen_cylinders_){
				obs_json["cylinders"].push_back({
					{"x", cyl.x},
					{"y", cyl.y},
					{"radius", cyl.radius},
					{"height", cyl.height}
				});
			}

			std::ofstream o(full_path.c_str());
			o << std::setw(4) << obs_json << std::endl;

			printf("Saved obstacle description json to path: %s\n",  full_path.c_str());
		}

		/**
		 * @brief Generate map with random cylinders and circles
		 */
		void createRandomForestMap()
		{
			int max_attempts = 999;	// Max number of attempts if cylinders cannot be generated

			// Random (x,y) distribution for placement of obstacles
			rand_x = std::uniform_real_distribution<double>(map_min_x_, map_max_x_);
			rand_y = std::uniform_real_distribution<double>(map_min_y_, map_max_y_);

			// Random cylinder radius/height distribution
			rand_w = std::uniform_real_distribution<double>(cyl_lower_rad_, cyl_upper_rad_);
			rand_h = std::uniform_real_distribution<double>(cyl_lower_height_, cyl_upper_height_);

			rand_radius_ = std::uniform_real_distribution<double>(hoop_lower_rad_, hoop_upper_rad_);
			rand_radius2_ = std::uniform_real_distribution<double>(hoop_lower_rad_, 1.2);
			rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
			rand_z_ = std::uniform_real_distribution<double>(hoop_lower_height_, hoop_upper_height_);


			// generate cylindrical obs
			for (int i = 0, iter = 0; i < cylindrical_obs_num_ ; i++)
			{	
				if (iter > max_attempts){
					break;
				}

				double x = rand_x(eng);		// [m] Random cylinder x position
				double y = rand_y(eng);		// [m] Random cylinder y position
				double height = rand_h(eng);				// [m] Random height 
				double radius = rand_w(eng);		// [m] Random radius

				double x_vox = (x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
				double y_vox = (y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
				double height_vox = height / map_res_;	// [voxels] Random Height 
				double radius_vox = radius / map_res_;	// [voxels] Random radius 

				// Only allow cylinders outside of obstacle free regions
				if (isInObsFreeRegion(x,y)){
					i--; // retry current iteration
					iter++;
					continue;
				}

				// Only allow generating cylinders at least a certain clearance away from existing cylinders
				for (Cylinder& cyl : rnd_gen_cylinders_){ 
					if ((Eigen::Vector2d(x, y) - cyl.position()).norm() < inter_obs_clearance_)
					{
						i--; // retry current iteration
						iter++;
						continue;
					}
				}

				addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);

				rnd_gen_cylinders_.push_back(Cylinder(x, y, radius, height));
			}

			// generate circle obs
			for (int i = 0; i < hoop_obs_num_; ++i)
			{
				double x, y, z;
				x = rand_x(eng);
				y = rand_y(eng);
				z = rand_z_(eng);

				x = floor(x / map_res_) * map_res_ + map_res_ / 2.0;
				y = floor(y / map_res_) * map_res_ + map_res_ / 2.0;
				z = floor(z / map_res_) * map_res_ + map_res_ / 2.0;

				Eigen::Vector3d translate(x, y, z);

				double theta = rand_theta_(eng);
				Eigen::Matrix3d rot_mat;
				rot_mat << 	cos(theta), -sin(theta), 0.0, 
							sin(theta), cos(theta), 0.0, 
							0, 0, 1;

				double radius1 = rand_radius_(eng);
				double radius2 = rand_radius2_(eng);

				// draw a hoop centered at (x,y,z)
				Eigen::Vector3d cpt;
				for (double angle = 0.0; angle < 6.282; angle += map_res_ / 2)
				{
					cpt(0) = 0.0;
					cpt(1) = radius1 * cos(angle);
					cpt(2) = radius2 * sin(angle);

					// inflate obstacle
					Eigen::Vector3d cpt_if;
					for (int ifx = -0; ifx <= 0; ++ifx)
					{
						for (int ify = -0; ify <= 0; ++ify)
						{
							for (int ifz = -0; ifz <= 0; ++ifz)
							{
								cpt_if = cpt + Eigen::Vector3d(ifx * map_res_, ify * map_res_,
															ifz * map_res_);
								cpt_if = rot_mat * cpt_if + Eigen::Vector3d(x, y, z);
								pcl::PointXYZ pt(cpt_if(0), cpt_if(1), cpt_if(2));
								cloud_map_.push_back(pt);
							}
						}
					}
				}
			}
		}

		// Generate empty vicon test map
		void createViconMap(){
			// // Create regions where obstacles cannot be created.
			// obs_free_squares_.push_back(ObsFreeSquare(-1.1, 1.1, -1.1, 1.1));

			// double min_x = 0.6;
			// double min_y = 0.6;
			// double max_x = 2.8;
			// double max_y = 2.8;

			// // Top left
			// obs_free_squares_.push_back(ObsFreeSquare(min_x, max_x, min_y, max_y));	
			// // Top right
			// obs_free_squares_.push_back(ObsFreeSquare(min_x, max_x, -max_y, -min_y));
			// // Bottom left
			// obs_free_squares_.push_back(ObsFreeSquare(-max_x, -min_x, min_y, max_y));
			// // Bottom right
			// obs_free_squares_.push_back(ObsFreeSquare(-max_x, -min_x,  -max_y, -min_y));


			// std::vector<Cylinder> cylinders;

			// cylinders.push_back(Cylinder(1.25, 0.0, 0.15, 3.0));
			// cylinders.push_back(Cylinder(0.0, -1.25, 0.15, 3.0));
			// cylinders.push_back(Cylinder(-1.25, 0.0, 0.15, 3.0));
			// cylinders.push_back(Cylinder(0.0, 1.25, 0.15, 3.0));

			// for (auto cyl : cylinders){
			// 	double x_vox = (cyl.x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
			// 	double y_vox = (cyl.y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
			// 	double radius_vox = cyl.radius / map_res_;	// [voxels] Random radius 
			// 	double height_vox = cyl.height / map_res_;	// [voxels] Random Height 

			// 	addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);
			// }
		}

		// Generate antipodal swap map for 6 drones
		void createSimpleTestMap(){
			double dist = 3.0; 
			double radius = 0.5;
			double height = 5.0;
			
			//Cylinder(x, y, radius,, height):
			std::vector<Cylinder> cylinders;
			cylinders.push_back(Cylinder(dist, 0.0, radius, height));
			cylinders.push_back(Cylinder(0.0, -dist, radius, height));
			cylinders.push_back(Cylinder(-dist, 0.0, radius, height));
			cylinders.push_back(Cylinder(0.0, dist, radius, height));

			for (auto cyl : cylinders){
				double x_vox = (cyl.x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
				double y_vox = (cyl.y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
				double radius_vox = cyl.radius / map_res_;	// [voxels] Random radius 
				double height_vox = cyl.height / map_res_;	// [voxels] Random Height 

				addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);
			}
		}

		void addSingleCylinder(const double& x, const double& y, 
							const double& radius, const double& height)
		{
			double x_vox = (x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
			double y_vox = (y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
			double radius_vox = radius / map_res_;	// [voxels] Random radius 
			double height_vox = height / map_res_;	// [voxels] Random Height 

			addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);
		}

		void createSimpleTestMap2(){
			double dist = 3.0; 
			double radius = 0.5;
			double height = 5.0;
			
			//Cylinder(x, y, radius,, height):
			std::vector<Cylinder> cylinders;
			cylinders.push_back(Cylinder(dist, dist, radius, height));
			cylinders.push_back(Cylinder(dist, -dist, radius, height));
			cylinders.push_back(Cylinder(-dist, dist, radius, height));
			cylinders.push_back(Cylinder(-dist, -dist, radius, height));

			for (auto cyl : cylinders){
				double x_vox = (cyl.x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
				double y_vox = (cyl.y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
				double radius_vox = cyl.radius / map_res_;	// [voxels] Random radius 
				double height_vox = cyl.height / map_res_;	// [voxels] Random Height 

				addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);
			}
		}

		// Generate antipodal swap map for 6 drones
		void createViconAntipodalMap(){
			// // No obstacle space
			// double buffer_rad = 0.3;
			// for (size_t i = 0; i < 6; i++){ 
			// 	Eigen::Matrix3d rot_mat = Eigen::AngleAxisd((M_PI/180.0) * i * 60, Eigen::Vector3d::UnitZ()).matrix();

			// 	Eigen::Vector3d drone_pos = rot_mat * Eigen::Vector3d{2.4, 0, 0};
			// 	obs_free_radial_.push_back(ObsFreeRadial(drone_pos(0), drone_pos(1), buffer_rad));

			// 	std::cout << "drone_pos: " << drone_pos.transpose() << std::endl;
			// }

			// std::vector<Cylinder> cylinders;

			// cylinders.push_back(Cylinder(1.25, 0.0, 0.15, 3.0));
			// cylinders.push_back(Cylinder(0.0, -1.25, 0.15, 3.0));
			// cylinders.push_back(Cylinder(-1.25, 0.0, 0.15, 3.0));
			// cylinders.push_back(Cylinder(0.0, 1.25, 0.15, 3.0));

			// for (auto cyl : cylinders){
			// 	double x_vox = (cyl.x / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder x position 
			// 	double y_vox = (cyl.y / map_res_) * map_res_ + map_res_ / 2.0; // [voxels] Random cylinder y position 
			// 	double radius_vox = cyl.radius / map_res_;	// [voxels] Random radius 
			// 	double height_vox = cyl.height / map_res_;	// [voxels] Random Height 

			// 	addCylinderToMap(x_vox, y_vox, radius_vox, height_vox);
			// }
		}

		pcl::PointCloud<pcl::PointXYZ> get_pcl() 
		{
			return cloud_map_;
		}

		std::string get_directory() const
		{
			return filepath_;
		}

		std::string get_filename() const
		{
			return filename_;
		}

		void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
		{
			std::string id_base = "goal_region";
			
			int i = 0; 
			for (auto goal_region : obs_free_squares_){
				// ROS_INFO("Goal region: (%f, %f), (%f, %f)",  goal_region.min_x, goal_region.max_x, goal_region.max_y, goal_region.max_y);
				vizSquares(viewer, goal_region.min_x, goal_region.max_x, 
								goal_region.min_y, goal_region.max_y, 
								id_base + std::to_string(i));
				i++;
			}

			for (auto goal_region : obs_free_radial_){
				// ROS_INFO("Goal region: (%f, %f), (%f, %f)",  goal_region.min_x, goal_region.max_x, goal_region.max_y, goal_region.max_y);
				vizSpheres(viewer, goal_region.x, goal_region.y, 1.0, 	
								id_base + std::to_string(i));
				i++;
			}
		}

	private:
		/* METHODS FOR GENERATION OF PRIMITIVES */
		void addRoomBoundsToMap(const double& x_size, 
			const double& y_size, const double& z_size, 
			const double& rm_min_x, const double& rm_max_x,
			const double& rm_min_y, const double& rm_max_y,
			const double& rm_height)
		{
			// Generate floor, ceiling and walls
			AddXYPlaneToMap(x_size, y_size, 0.0);
			AddXYPlaneToMap(x_size, y_size, z_size);

			addYWallToMap(rm_max_x, rm_min_y,  rm_max_y, rm_height);
			addYWallToMap(rm_min_x, rm_min_y,  rm_max_y, rm_height);

			addXWallToMap(rm_max_y,  rm_min_x,  rm_max_x,  rm_height);
			addXWallToMap(rm_min_y,  rm_min_x,  rm_max_x, rm_height);
		}

		/**
		 * @brief Generate a cylinder
		 * 
		 * @param x 
		 * @param y 
		 * @param radius 
		 * @param height 
		 */
		void addCylinderToMap(	const double& x_vox, const double& y_vox, 
								const double& radius_vox, const double& height_vox)
		{
			for (int r = -radius_vox ; r < radius_vox ; r++)	// for each x axis voxel
			{
				for (int s = -radius_vox; s < radius_vox; s++) // for each y axis voxel  
				{
					for (int t = 0; t < height_vox; t++) // for each height voxel
					{
						double temp_x = x_vox + (r + 0.5) * map_res_ + 1e-2;
						double temp_y = y_vox + (s + 0.5) * map_res_ + 1e-2;
						double temp_z = (t + 0.5) * map_res_ + 1e-2;
						double radius = radius_vox * map_res_;	// [m] Random radius 
						// If current point is within the radius of the circle center, then add to point cloud
						if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x_vox, y_vox)).norm() <= radius)
						{
							pcl::PointXYZ pt(temp_x, temp_y, temp_z);
							cloud_map_.points.push_back(pt);
						}
					}
				}
			}
		}

		void addWallToMap(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt, const double& start_z, const double& height)
		{
			pcl::PointXYZ pt;

			long num_z_cells =  floor((height - start_z)/ map_res_);

			int x0 = floor(start_pt(0)/map_res_);
			int y0 = floor(start_pt(1)/map_res_);

			int x1 = floor(end_pt(0)/map_res_);
			int y1 = floor(end_pt(1)/map_res_);

			int dx = abs(x1 - x0);
			int sx = x0 < x1 ? 1 : -1;
			int dy = -abs(y1 - y0);
			int sy = y0 < y1 ? 1 : -1;
			int error = dx + dy;
			
			// We need to iterate through cell space
			while (true){
				pt.x = x0 * map_res_;
				pt.y = y0 * map_res_;
				for (int k = 0; k < num_z_cells; k++){
					pt.z = start_z + k * map_res_ + 1e-2;
					cloud_map_.points.push_back(pt);
				}

				if (x0 == x1 && y0 == y1) {
					break;
				}
				int e2 = 2 * error;
				if (e2 >= dy){
					if (x0 == x1) {
						break;
					}
					error = error + dy;
					x0 = x0 + sx;
				}
				if (e2 <= dx){
					if (y0 == y1) {
						break;
					}
					error = error + dx;
					y0 = y0 + sy;
				}
			}

		}

		/**
		 * @brief Generates a wall along y axis. 
		 * 
		 * @param x 
		 * @param y1 
		 * @param y2 
		 * @param height 
		 */
		void addYWallToMap(double x, double y1, double y2, double height)
		{
			pcl::PointXYZ pt;

			// Get length of wall in number of cells
			long length_num_cells = (y2 - y1)/map_res_;
			int height_num_cells = ceil(height / map_res_);

			// Convert points to be within resolution grid
			double x_grid = floor(x / map_res_) * map_res_ + map_res_ / 2.0;
			double y1_grid = floor(y1 / map_res_) * map_res_ + map_res_ / 2.0;

			pt.x = x_grid;

			// iterate through length of the wall along y axis
			for (int i = 0; i < length_num_cells; i++ ){
				pt.y = y1_grid + i * map_res_  + 1e-2;

				// iterate through height of the wall along z axis
				for (int t = 0; t < height_num_cells; t++){
					pt.z = (t * map_res_) + 1e-2;
					cloud_map_.points.push_back(pt);
				}
			}
		}

		/**
		 * @brief Generates a wall along X axis. 
		 * 
		 * @param y
		 * @param x1 
		 * @param x2 
		 * @param height 
		 */
		void addXWallToMap(double y, double x1, double x2, double height)
		{
			pcl::PointXYZ pt;

			// Get length of wall in number of cells
			long length_num_cells = (x2 - x1)/map_res_;
			int height_num_cells = ceil(height / map_res_);

			// Convert points to be within resolution grid
			double y_grid = floor(y / map_res_) * map_res_ + map_res_ / 2.0;
			double x1_grid = floor(x1 / map_res_) * map_res_ + map_res_ / 2.0;

			pt.y = y_grid;

			// iterate through length of the wall along x axis
			for (int i = 0; i < length_num_cells; i++ ){
				pt.x = x1_grid + i * map_res_  + 1e-2;

				// iterate through height of the wall along z axis
				for (int t = 0; t < height_num_cells; t++){
					pt.z = (t * map_res_) + 1e-2;
					cloud_map_.points.push_back(pt);
				}
			}
		}

		void AddXYPlaneToMap(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt, const double& z_height)
		{
			pcl::PointXYZ pt;
			pt.z = z_height;

			long num_x_cells = floor((end_pt(0) - start_pt(0)) / map_res_);
			long num_y_cells = floor((end_pt(1) - start_pt(1)) / map_res_);

			for (int i = 0; i < num_x_cells; i++){ // x-axis
				// Points need to be in units of meters
				pt.x = start_pt(0) + i * map_res_ + 1e-2;
				for (int j = 0; j < num_y_cells; j++){ // y-axis
					pt.y = start_pt(1) + j * map_res_ + 1e-2;

					cloud_map_.points.push_back(pt);
				}
			}
			
		}

		/**
		 * @brief Add a horizontal XY plane of predefined size and height
		 * 
		 * @param size_x 
		 * @param size_y 
		 * @param size_z 
		 */
		void AddXYPlaneToMap(double size_x, double size_y, double z_height)
		{
			pcl::PointXYZ pt;
			pt.z = z_height;

			long num_x_cells = floor(size_x / map_res_);
			long num_y_cells = floor(size_y / map_res_);

			for (int i = -num_x_cells/2; i < num_x_cells/2; i++){ // x-axis
				for (int j = -num_y_cells/2; j < num_y_cells/2; j++){ // y-axis
					// Points need to be in units of meters
					pt.x = i*map_res_ + 1e-2;
					pt.y = j*map_res_ + 1e-2;

					cloud_map_.points.push_back(pt);
				}
			}

		}

		/**
		 * @brief Check if obstacle is in goal regions. IF true, then return false.
		 * 
		 */
		bool isInObsFreeRegion(double x, double y){
			 for (auto obs_free_region : obs_free_squares_){
				if ( obs_free_region.isInObsFree(x, y))
				{
					return true;
				}
			 }

			 for (auto obs_free_region : obs_free_radial_){
				if (obs_free_region.isInObsFree(x, y))
				{
					return true;
				}
			 }

			 return false;
		}

		void vizSquares(pcl::visualization::PCLVisualizer& viewer, 
							double x_min, double y_min, double x_max, double y_max, 
							const std::string& id)
		{
			int viewport = 0;
			double z_min = 0.0;
			double z_max = 0.1;

			Eigen::Vector3d color(1.0, 0.5, 1.0);

			viewer.addCube(x_min, y_min, x_max, y_max, z_min, z_max, 
							color(0), color(1), color(2), id, viewport);
		}

		void vizSpheres(pcl::visualization::PCLVisualizer& viewer, 
							double x, double y, double radius, const std::string& id)
		{
			pcl::PointXYZ center(x, y, 0.0);
			Eigen::Vector3d color(1.0, 0.5, 1.0);
			int viewport = 0;

			viewer.addSphere(center, radius, color(0), color(1), color(2), id, viewport);
		}

	private:
		std::random_device rd;
		std::default_random_engine eng;
		std::uniform_real_distribution<double> rand_x;
		std::uniform_real_distribution<double> rand_y;
		std::uniform_real_distribution<double> rand_w;
		std::uniform_real_distribution<double> rand_h;

		std::string filepath_;	// parent directory of pcd file to be saved
		std::string filename_;	// name of pcd file to be saved

		int seed_;	// random seed

		std::vector<ObsFreeSquare> obs_free_squares_;	// Obstacle free regions
		std::vector<ObsFreeRadial> obs_free_radial_;	// Obstacle free regions

		std::vector<Cylinder> rnd_gen_cylinders_; // Vector of randomly generated cylinders

		// Params
		double map_max_x_, map_min_x_;
		double map_min_y_, map_max_y_;

		int cylindrical_obs_num_;	// Number of cylindrical obstacles
		double map_x_size_, map_y_size_, map_z_size_;	// Size of map
		double cyl_lower_rad_, cyl_upper_rad_, cyl_lower_height_, cyl_upper_height_;
		double map_res_;	// Map resolution
		double inter_obs_clearance_; // Clearance between obstacles

		int hoop_obs_num_;
		double hoop_lower_rad_, hoop_upper_rad_, hoop_lower_height_, hoop_upper_height_;
		double theta_;

		// Uniform distribution for hoops
		std::uniform_real_distribution<double> rand_radius_;
		std::uniform_real_distribution<double> rand_radius2_;
		std::uniform_real_distribution<double> rand_theta_;
		std::uniform_real_distribution<double> rand_z_;

		pcl::PointCloud<pcl::PointXYZ> cloud_map_; // PCD obstacle map to be saved

};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	auto node = std::make_shared<PCDMapGen>();

	node->createMap();
	node->savePCDMap();
	// node->saveObstacleJSON();

	pcl::PointCloud<pcl::PointXYZ> cloud_map = node->get_pcl();

	pcl::visualization::CloudViewer viewer("PCD Map Viewer");
	viewer.showCloud(cloud_map.makeShared());
	viewer.runOnVisualizationThreadOnce(boost::bind(&PCDMapGen::viewerOneOff, node, std::placeholders::_1));

	while (!viewer.wasStopped() && rclcpp::ok())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	rclcpp::shutdown();

	return 0;
}



// /**
//  * @brief Generate a rectangular tunnel 
//  * 
//  */
// void generateRectangularTunnel(
// 	const Eigen::Vector3d& start_pt, 
// 	const double& length, const double& width, const double& height){

// 	Eigen::Vector3d end_pt = start_pt + Eigen::Vector3d{length, width, 0.0};

// 	// Generate tunnel floor
// 	AddXYPlaneToMap(
// 		Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, 0.0);

// 	// Generate tunnel ceiling
// 	AddXYPlaneToMap(
// 		Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, height);

// 	/* Generate tunnel walls */
// 	// Right wall (keep y1 constant, and change x)
// 	addWallToMap(Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),start_pt(1)}, 0.0, height);

// 	// Left wall (keep y2 constant, and change x)
// 	addWallToMap(Eigen::Vector2d{start_pt(0),end_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, 0.0, height);
// }
