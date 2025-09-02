#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import socket
import threading
import numpy as np
import json
from datetime import datetime, timedelta
import asyncio
from std_msgs.msg import UInt16
from battery_soc import get_battery_status_mv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

class CommandModel(BaseModel):
    cmd: str
    speed: float = 1.0


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # --- 参数：电机增益 ---
        self.declare_parameter('left_motor_gain', 1.0)
        self.declare_parameter('right_motor_gain', 1.0)
        self.left_gain = self.get_parameter('left_motor_gain').get_parameter_value().double_value
        self.right_gain = self.get_parameter('right_motor_gain').get_parameter_value().double_value

        # --- 参数：UDP 端口 ---
        self.declare_parameter('udp_port', 8080)
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # --- 参数：超时时间（毫秒）---
        self.declare_parameter('command_timeout_ms', 500)
        self.timeout = timedelta(milliseconds=self.get_parameter('command_timeout_ms').get_parameter_value().integer_value)

        # --- 参数：通信模式 (udp 或 api) ---
        self.declare_parameter('comm_mode', 'api')  # 可选值: 'udp', 'api'
        self.comm_mode = self.get_parameter('comm_mode').get_parameter_value().string_value.lower()
        if self.comm_mode not in ['udp', 'api']:
            self.get_logger().warn(f"Invalid comm_mode: {self.comm_mode}, defaulting to 'udp'")
            self.comm_mode = 'udp'

        # 电机控制发布者
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)

        # 共享状态
        self.command = "stop"
        self.speed = 1.0  # 默认速度
        self.last_command_time = datetime.now()
        self.command_lock = threading.Lock()

        # 根据模式启动不同服务
        if self.comm_mode == 'api':
            self.get_logger().info("Starting in FastAPI mode")
            self.fastapi_thread = threading.Thread(target=self.start_fastapi_server, daemon=True)
            self.fastapi_thread.start()
        else:  # udp 模式
            self.get_logger().info("Starting in UDP mode")
            self.udp_thread = threading.Thread(target=self.udp_server, args=(self.udp_port,), daemon=True)
            self.udp_thread.start()

        # 定时发布电机指令（10Hz）
        self.timer = self.create_timer(0.1, self.publish_motor_command)

        self.get_logger().info(
            f"Motor control node started in '{self.comm_mode}' mode, "
            f"timeout={self.timeout.total_seconds()*1000:.0f}ms, "
            f"left_gain={self.left_gain}, right_gain={self.right_gain}"
        )

        # --- 电池状态 ---
        self.latest_battery_mv = None
        self.battery_timestamp = None
        self.battery_lock = threading.Lock()

        self.battery_sub = self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self.battery_callback,
            10
        )
        self.get_logger().info("Subscribed to '/ros_robot_controller/battery' for battery status.")

    def battery_callback(self, msg: UInt16):
        mv = msg.data
        with self.battery_lock:
            self.latest_battery_mv = mv
            self.battery_timestamp = datetime.now()
        self.get_logger().debug(f"Battery updated: {mv} mV")

    def udp_server(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('0.0.0.0', port)  # 允许外部访问
        try:
            sock.bind(server_address)
            self.get_logger().info(f"UDP server listening on {server_address}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")
            return

        while rclpy.ok():
            try:
                sock.settimeout(1.0)  # 防止无限阻塞
                data, address = sock.recvfrom(1024)
                message = data.decode('utf-8').strip()

                try:
                    data = json.loads(message)
                    cmd = data.get("cmd", "").lower()
                    speed = data.get("speed", 1.0)

                    if cmd not in ["forward", "backward", "left", "right", "stop"]:
                        self.get_logger().warn(f"Invalid command: {cmd}")
                        continue

                    if not isinstance(speed, (int, float)) or speed < 0:
                        self.get_logger().warn(f"Invalid speed value: {speed}")
                        continue

                    with self.command_lock:
                        self.command = cmd
                        self.speed = float(speed)
                        self.last_command_time = datetime.now()

                    self.get_logger().info(f"Received from {address}: {data}")

                except json.JSONDecodeError:
                    self.get_logger().warn(f"Invalid JSON: {message}")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")

    def start_fastapi_server(self):
        app = FastAPI(title="Motor Control API", version="1.0")

        # 允许 CORS（可选，用于前端调试）
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.post("/command")
        async def set_command(command: CommandModel):
            cmd = command.cmd.lower()
            speed = command.speed

            if cmd not in ["forward", "backward", "left", "right", "stop"]:
                raise HTTPException(status_code=400, detail="Invalid command")

            if speed < 0:
                raise HTTPException(status_code=400, detail="Speed must be >= 0")

            with self.command_lock:
                self.command = cmd
                self.speed = float(speed)
                self.last_command_time = datetime.now()

            self.get_logger().info(f"API received: {command}")
            return {"status": "ok", "received": command.dict()}

        @app.get("/status")
        async def get_status():
            with self.command_lock:
                cmd = self.command
                speed = self.speed
                elapsed = (datetime.now() - self.last_command_time).total_seconds()
                is_active = elapsed < self.timeout.total_seconds()
            return {
                "command": cmd,
                "speed": speed,
                "last_updated": self.last_command_time.isoformat(),
                "time_since_update_sec": elapsed,
                "active": is_active,
                "timeout_sec": self.timeout.total_seconds()
            }
        
        @app.get("/battery")
        async def get_battery():
            with self.battery_lock:
                mv = self.latest_battery_mv
                timestamp = self.battery_timestamp

            if mv is None:
                raise HTTPException(status_code=503, detail="Battery data not available yet")

            try:
                status = get_battery_status_mv(mv)
            except Exception as e:
                self.get_logger().error(f"Error computing battery status: {e}")
                raise HTTPException(status_code=500, detail="Failed to compute battery status")

            return {
                "voltage_V": status['voltage_V'],
                "soc": status['soc'],
                "alarm": status['alarm'],
                "should_shutdown": status['should_shutdown'],
                "status": status['status'],
                "timestamp": timestamp.isoformat() if timestamp else None
            }

        # 启动 Uvicorn 服务器
        config = uvicorn.Config(
            app=app,
            host="0.0.0.0", 
            port=11451,
            log_level="info",
            loop="asyncio",
        )
        server = uvicorn.Server(config)

        try:
            asyncio.run(server.serve())
        except Exception as e:
            self.get_logger().error(f"FastAPI server error: {e}")

    def publish_motor_command(self):
        now = datetime.now()
        with self.command_lock:
            cmd = self.command
            speed = self.speed
            elapsed = now - self.last_command_time
            active = elapsed < self.timeout

        # 超时则停止
        if not active and cmd != "stop":
            cmd = "stop"
            self.get_logger().warn("Command timeout, stopping motors.")

        msg = MotorsState()
        motor1 = MotorState()
        motor2 = MotorState()
        motor1.id = 1
        motor2.id = 2

        base_speed = speed

        if cmd == "forward":
            motor1.rps = base_speed * self.left_gain
            motor2.rps = -base_speed * self.right_gain
        elif cmd == "backward":
            motor1.rps = -base_speed * self.left_gain
            motor2.rps = base_speed * self.right_gain
        elif cmd == "left":
            motor1.rps = 0.0
            motor2.rps = -base_speed * self.right_gain
        elif cmd == "right":
            motor1.rps = base_speed * self.left_gain
            motor2.rps = 0.0
        else:  # stop
            motor1.rps = 0.0
            motor2.rps = 0.0

        msg.data = [motor1, motor2]
        self.motor_pub.publish(msg)

        if active:
            self.get_logger().debug(
                f'Pub: L={motor1.rps:.2f}, R={motor2.rps:.2f} | '
                f'Cmd={cmd}, Speed={speed}, Elapsed={elapsed.total_seconds()*1000:.0f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()