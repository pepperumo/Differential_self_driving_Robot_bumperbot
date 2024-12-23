import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        # Declare parameters with default values
        self.declare_parameter('simple_int_param', 28)
        self.declare_parameter('simple_string_param', 'Antonio')

        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == "simple_int_param" and param.type == Parameter.Type.INTEGER:
                self.get_logger().info(
                    "Param simple_int_param changed! New value is %d" % param.value
                )
            elif param.name == "simple_string_param" and param.type == Parameter.Type.STRING:
                self.get_logger().info(
                    "Param simple_string_param changed! New value is %s" % param.value
                )
            else:
                self.get_logger().warn(
                    f"Parameter {param.name} update failed or has unsupported type."
                )
                result.successful = False

        return result


def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
