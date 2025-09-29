#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

class FormationNode(Node):
    def __init__(self):
        super().__init__("formation_node")
        self.declare_parameter("connections", ["udp://:14540", "udp://:14541", "udp://:14542", "udp://:14543"])
        self.declare_parameter("takeoff_alt", 3.0)
        self.declare_parameter("spacing", 2.0)
        self.connections = self.get_parameter("connections").get_parameter_value().string_array_value
        self.takeoff_alt = self.get_parameter("takeoff_alt").get_parameter_value().double_value
        self.spacing = self.get_parameter("spacing").get_parameter_value().double_value
        self.get_logger().info(f"Connections: {self.connections}")
        self.drones = []

    async def run(self):
        await self.connect_all()
        await self.arm_and_takeoff_all()
        await self.form_line_and_move()

    async def connect_all(self):
        for conn in self.connections:
            drone = System()
            await drone.connect(system_address=conn)
            self.get_logger().info(f"Connecting to {conn}...")
            async for state in drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info(f"Connected to {conn}")
                    break
            self.drones.append(drone)

    async def arm_and_takeoff_all(self):
        tasks = [self.arm_and_takeoff(drone, self.takeoff_alt) for drone in self.drones]
        await asyncio.gather(*tasks)

    async def arm_and_takeoff(self, drone: System, altitude: float):
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(3)

    async def form_line_and_move(self):
        leader = self.drones[0]
        async for pos in leader.telemetry.position():
            leader_lat = pos.latitude_deg
            leader_lon = pos.longitude_deg
            leader_rel_alt = pos.relative_altitude_m
            break
        offsets = [(-i * self.spacing, 0.0, 0.0) for i in range(len(self.drones))]
        tasks = [self.move_to_offset(drone, off) for drone, off in zip(self.drones, offsets)]
        await asyncio.gather(*tasks)
        distance = 10.0
        steps = 20
        for s in range(steps):
            step_n = (s+1) * (distance/steps)
            tasks = [self.set_velocity_ned(drone, step_n - idx*self.spacing, 0.0, 0.0)
                     for idx, drone in enumerate(self.drones)]
            await asyncio.gather(*tasks)
            await asyncio.sleep(0.5)
        await asyncio.gather(*[d.action.land() for d in self.drones])

    async def move_to_offset(self, drone: System, offset_ned):
        n, e, d = offset_ned
        await self.set_velocity_ned(drone, n, e, 0.0)
        await asyncio.sleep(2)
        await self.set_velocity_ned(drone, 0.0, 0.0, 0.0)

    async def set_velocity_ned(self, drone: System, vn, ve, vd, yaw_deg=0.0):
        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))
        except OffboardError:
            try:
                await drone.offboard.start()
                await asyncio.sleep(0.1)
                await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))
            except Exception as ex:
                self.get_logger().warn(f"Offboard start failed: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = FormationNode()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.run())
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
