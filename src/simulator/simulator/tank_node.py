from rclpy.node import Node
from water_tank_interface.msg import TankInput, TankOutput


class TankNode(Node):
  
    def __init__(self, default_cmd: float = 0.0):
        super().__init__('tank_sim')
        self._cmd = float(default_cmd)
        self.create_subscription(TankInput, '/inlet_valve', self._on_cmd, 10)
        self._level_pub = self.create_publisher(TankOutput, '/water_level', 10)

    def _on_cmd(self, msg: TankInput):
        self._cmd = float(msg.inlet_valve)          # <- field name from TankInput.msg

    def get_cmd(self) -> float:
        return self._cmd

    def publish_level(self, level: float):
        msg = TankOutput()
        msg.level = float(level)
        self._level_pub.publish(msg)
