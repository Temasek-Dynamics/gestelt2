#include <fake_drone/fake_drone.hpp>

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto node = std::make_shared<FakeDrone>();
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}

