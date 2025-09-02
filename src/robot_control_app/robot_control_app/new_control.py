#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import socket
import threading
import json
from datetime import datetime, timedelta
import asyncio
from std_msgs.msg import UInt16
from battery_soc import get_battery_status_mv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import uvicorn
from typing import Literal, Optional
from enum import Enum

# 定义常量
DEFAULT_UDP_PORT = 8080
DEFAULT_API_PORT = 11451
DEFAULT_COMMAND_TIMEOUT_MS = 500
DEFAULT_MOTOR_GAIN = 1.0
COMMAND_PUBLISH_RATE_HZ = 10

class CommandType(str, Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"

class CommMode(str, Enum):
    UDP = "udp"
    API = "api"

class CommandModel(BaseModel):
    cmd: CommandType
    speed: float = Field(default=1.0, ge=0.0, description="电机速度 (必须 >= 0)")

class BatteryStatus(BaseModel):
    voltage_V: float
    soc: float
    alarm: bool
    should_shutdown: bool
    status: str
    timestamp: Optional[str] = None

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # 初始化参数
        self._init_parameters()
        
        # 初始化发布者和订阅者
        self._init_ros_communication()
        
        # 初始化线程安全的共享状态
        self._init_shared_state()
        
        # 根据通信模式启动相应的服务器
        self._start_comm_server()
        
        # 启动命令发布定时器
        self.timer = self.create_timer(1.0/COMMAND_PUBLISH_RATE_HZ, self._publish_motor_command)
        
        self.get_logger().info(
            f"电机控制节点已启动，通信模式：'{self.comm_mode}'，"
            f"超时时间：{self.timeout.total_seconds()*1000:.0f}ms，"
            f"左电机增益：{self.left_gain}，右电机增益：{self.right_gain}"
        )

    def _init_parameters(self):
        """初始化节点参数"""
        # 电机增益参数
        self.declare_parameter('left_motor_gain', DEFAULT_MOTOR_GAIN)
        self.declare_parameter('right_motor_gain', DEFAULT_MOTOR_GAIN)
        self.left_gain = self.get_parameter('left_motor_gain').get_parameter_value().double_value
        self.right_gain = self.get_parameter('right_motor_gain').get_parameter_value().double_value
        
        # 通信参数
        self.declare_parameter('udp_port', DEFAULT_UDP_PORT)
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.declare_parameter('api_port', DEFAULT_API_PORT)
        self.api_port = self.get_parameter('api_port').get_parameter_value().integer_value
        
        # 命令超时参数
        self.declare_parameter('command_timeout_ms', DEFAULT_COMMAND_TIMEOUT_MS)
        timeout_ms = self.get_parameter('command_timeout_ms').get_parameter_value().integer_value
        self.timeout = timedelta(milliseconds=timeout_ms)
        
        # 通信模式参数
        self.declare_parameter('comm_mode', CommMode.API.value)
        comm_mode = self.get_parameter('comm_mode').get_parameter_value().string_value.lower()
        
        try:
            self.comm_mode = CommMode(comm_mode)
        except ValueError:
            self.get_logger().warn(f"无效的通信模式：{comm_mode}，默认使用'udp'")
            self.comm_mode = CommMode.UDP

    def _init_ros_communication(self):
        """初始化ROS发布者和订阅者"""
        # 电机控制发布者
        self.motor_pub = self.create_publisher(
            MotorsState, 
            '/ros_robot_controller/set_motor', 
            10
        )
        
        # 电池状态订阅者
        self.battery_sub = self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self._battery_callback,
            10
        )
        self.get_logger().info("已订阅 '/ros_robot_controller/battery' 获取电池状态")

    def _init_shared_state(self):
        """初始化线程安全的共享状态变量"""
        # 命令状态
        self.command = CommandType.STOP
        self.speed = 1.0
        self.last_command_time = datetime.now()
        self.command_lock = threading.RLock()
        
        # 电池状态
        self.latest_battery_mv = None
        self.battery_timestamp = None
        self.battery_lock = threading.RLock()

    def _start_comm_server(self):
        """根据模式启动相应的通信服务器"""
        if self.comm_mode == CommMode.API:
            self.get_logger().info(f"启动FastAPI模式，端口：{self.api_port}")
            self.api_thread = threading.Thread(
                target=self._start_fastapi_server, 
                daemon=True
            )
            self.api_thread.start()
        else:  # UDP模式
            self.get_logger().info(f"启动UDP模式，端口：{self.udp_port}")
            self.udp_thread = threading.Thread(
                target=self._udp_server, 
                daemon=True
            )
            self.udp_thread.start()

    def _battery_callback(self, msg: UInt16):
        """处理电池电压更新"""
        mv = msg.data
        with self.battery_lock:
            self.latest_battery_mv = mv
            self.battery_timestamp = datetime.now()
        self.get_logger().debug(f"电池电压已更新：{mv} mV")

    def _udp_server(self):
        """UDP服务器实现"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('0.0.0.0', self.udp_port)
        
        try:
            sock.bind(server_address)
            self.get_logger().info(f"UDP服务器监听于 {server_address}")
        except Exception as e:
            self.get_logger().error(f"UDP套接字绑定失败：{e}")
            return

        while rclpy.ok():
            try:
                sock.settimeout(1.0)  # 防止无限阻塞
                data, address = sock.recvfrom(1024)
                message = data.decode('utf-8').strip()

                try:
                    cmd_data = json.loads(message)
                    self._process_command(cmd_data, f"UDP客户端 {address}")
                except json.JSONDecodeError:
                    self.get_logger().warn(f"来自 {address} 的无效JSON：{message}")
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP接收错误：{e}")

    def _process_command(self, cmd_data, source="未知"):
        """处理来自任何源（UDP或API）的命令"""
        try:
            cmd = cmd_data.get("cmd", "").lower()
            speed = cmd_data.get("speed", 1.0)
            
            # 验证命令
            if cmd not in [e.value for e in CommandType]:
                self.get_logger().warn(f"来自 {source} 的无效命令：{cmd}")
                return False
                
            # 验证速度
            if not isinstance(speed, (int, float)) or speed < 0:
                self.get_logger().warn(f"来自 {source} 的无效速度值：{speed}")
                return False
                
            # 更新命令状态
            with self.command_lock:
                self.command = cmd
                self.speed = float(speed)
                self.last_command_time = datetime.now()
                
            self.get_logger().info(f"收到来自 {source} 的命令：cmd={cmd}, speed={speed}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"处理命令时出错：{e}")
            return False

    def _start_fastapi_server(self):
        """启动FastAPI服务器进行HTTP控制"""
        app = FastAPI(
            title="机器人电机控制API", 
            version="1.0",
            description="用于控制机器人电机和检查电池状态的API"
        )

        # 启用CORS以支持Web客户端
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.post("/command", response_model=dict)
        async def set_command(command: CommandModel):
            success = self._process_command(
                {"cmd": command.cmd, "speed": command.speed}, 
                "API"
            )
            if not success:
                raise HTTPException(status_code=400, detail="Invalid command parameters")
            return {"status": "ok", "received": command.dict()}

        @app.get("/status", response_model=dict)
        async def get_status():
            with self.command_lock:
                cmd = self.command
                speed = self.speed
                last_time = self.last_command_time
                elapsed = (datetime.now() - last_time).total_seconds()
                is_active = elapsed < self.timeout.total_seconds()
                
            return {
                "command": cmd,
                "speed": speed,
                "last_updated": last_time.isoformat(),
                "time_since_update_sec": elapsed,
                "active": is_active,
                "timeout_sec": self.timeout.total_seconds()
            }
        
        @app.get("/battery", response_model=BatteryStatus)
        async def get_battery():
            with self.battery_lock:
                mv = self.latest_battery_mv
                timestamp = self.battery_timestamp

            if mv is None:
                raise HTTPException(
                    status_code=503, 
                    detail="Battery data not available yet"
                )

            try:
                status = get_battery_status_mv(mv)
                status['timestamp'] = timestamp.isoformat() if timestamp else None
                return status
            except Exception as e:
                self.get_logger().error(f"计算电池状态时出错：{e}")
                raise HTTPException(
                    status_code=500, 
                    detail="Failed to compute battery status"
                )

        # 启动Uvicorn服务器
        config = uvicorn.Config(
            app=app,
            host="0.0.0.0", 
            port=self.api_port,
            log_level="info",
            loop="asyncio",
        )
        server = uvicorn.Server(config)

        try:
            asyncio.run(server.serve())
        except Exception as e:
            self.get_logger().error(f"FastAPI服务器错误：{e}")

    def _publish_motor_command(self):
        """根据当前状态发布电机命令"""
        now = datetime.now()
        
        with self.command_lock:
            cmd = self.command
            speed = self.speed
            elapsed = now - self.last_command_time
            active = elapsed < self.timeout

        # 如果命令已超时，则停止电机
        if not active and cmd != CommandType.STOP:
            cmd = CommandType.STOP
            self.get_logger().warn("命令超时，停止电机")

        # 创建电机命令消息
        msg = MotorsState()
        motor_left = MotorState(id=1, rps=0.0)
        motor_right = MotorState(id=2, rps=0.0)
        
        # 根据命令设置电机速度
        if active or cmd == CommandType.STOP:
            if cmd == CommandType.FORWARD:
                motor_left.rps = speed * self.left_gain
                motor_right.rps = -speed * self.right_gain
            elif cmd == CommandType.BACKWARD:
                motor_left.rps = -speed * self.left_gain
                motor_right.rps = speed * self.right_gain
            elif cmd == CommandType.LEFT:
                motor_left.rps = 0.0
                motor_right.rps = -speed * self.right_gain
            elif cmd == CommandType.RIGHT:
                motor_left.rps = speed * self.left_gain
                motor_right.rps = 0.0
            # 对于STOP，两个电机保持0.0

        msg.data = [motor_left, motor_right]
        self.motor_pub.publish(msg)

        if active:
            self.get_logger().debug(
                f'发布：左={motor_left.rps:.2f}, 右={motor_right.rps:.2f} | '
                f'命令={cmd}, 速度={speed}, 已过时间={elapsed.total_seconds()*1000:.0f}ms'
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
