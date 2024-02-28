import rclpy
import rclpy.node

class NDSILDAdapter(rclpy.node.Node):
    def __init__(self):
        super().__init__('sd_ngsild_adapter')

def main():
    rclpy.init()
    node = NDSILDAdapter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()