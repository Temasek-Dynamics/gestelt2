#include <fake_map/fake_map_publisher.hpp>

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	auto node = std::make_shared<FakeMapPublisher>();
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
