#include <voronoi_planner/voronoi_planner.hpp>

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto node = std::make_shared<navigator::VoronoiPlanner>();
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}

