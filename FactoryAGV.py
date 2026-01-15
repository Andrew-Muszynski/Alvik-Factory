#!/usr/bin/env python3
"""
FactoryAGV.py - AGV Fleet Manager Node

Manages Alvik AGVs with starting positions A, B, C and destinations D, E.
Handles queue advancement and service coordination.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, Optional


class AGVStatus:
    """Stores the status of a single AGV"""
    def __init__(self, name: str):
        self.name = name
        self.state = "UNKNOWN"  # IDLE, WAITING_LOAD, MOVING, ARRIVED, RETURNING
        self.position = "UNKNOWN"  # A, B, C, D, E, UNKNOWN
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.color = "unknown"
        self.last_update = 0.0
        self.available = False
    
    def update_from_json(self, json_str: str) -> bool:
        """Update status from JSON string"""
        try:
            data = json.loads(json_str)
            self.state = data.get('state', 'UNKNOWN')
            self.position = data.get('position', 'UNKNOWN')
            self.x = float(data.get('x', 0.0))
            self.y = float(data.get('y', 0.0))
            self.yaw = float(data.get('yaw', 0.0))
            self.color = data.get('color', 'unknown')
            self.last_update = time.time()
            
            # Robot is available if IDLE and at position A
            self.available = (self.state == "IDLE" and self.position == "A")
            return True
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            return False
    
    def is_online(self, timeout: float = 2.0) -> bool:
        """Check if robot has sent recent updates"""
        return (time.time() - self.last_update) < timeout
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        return {
            'name': self.name,
            'state': self.state,
            'position': self.position,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'color': self.color,
            'available': self.available,
            'online': self.is_online()
        }


class FactoryAGVManager(Node):
    """
    Manages a fleet of AGVs with queue positions A, B, C
    """
    
    def __init__(self):
        super().__init__('factory_agv_manager')
        
        # Configuration
        self.agv_names = ['Alvik1', 'Alvik2', 'Alvik3']
        self.agvs: Dict[str, AGVStatus] = {}
        self.command_publishers: Dict[str, any] = {}
        
        # Service tracking
        self.current_service_agv = None  # Which AGV is currently in service
        self.current_destination = None  # D or E
        
        # Initialize AGV tracking
        for agv_name in self.agv_names:
            self.agvs[agv_name] = AGVStatus(agv_name)
            
            # Subscribe to each AGV's status
            self.create_subscription(
                String,
                f'{agv_name}_status',
                lambda msg, name=agv_name: self.agv_status_callback(msg, name),
                10
            )
            
            # Create publisher for commands to each AGV
            self.command_publishers[agv_name] = self.create_publisher(
                String,
                f'{agv_name}_cmd',
                10
            )
        
        # Publisher for responses to supervisory
        self.response_pub = self.create_publisher(
            String,
            'factory_agv_response',
            10
        )
        
        # Subscriber for commands from supervisory
        self.supervisory_cmd_sub = self.create_subscription(
            String,
            'factory_agv_command',
            self.supervisory_command_callback,
            10
        )
        
        # Timer to check for queue advancement needs
        self.create_timer(1.0, self.check_queue_advancement)
        
        self.get_logger().info(f'Factory AGV Manager initialized with {len(self.agv_names)} AGVs')
        self.get_logger().info(f'Tracking: {", ".join(self.agv_names)}')
    
    def agv_status_callback(self, msg: String, agv_name: str):
        """Receive status update from an AGV"""
        if self.agvs[agv_name].update_from_json(msg.data):
            # Log significant state changes
            if not hasattr(self, '_last_state'):
                self._last_state = {}
            
            if self._last_state.get(agv_name) != self.agvs[agv_name].state:
                self.get_logger().info(
                    f'{agv_name}: {self.agvs[agv_name].state} @ '
                    f'{self.agvs[agv_name].position} '
                    f'({self.agvs[agv_name].x:.1f}, {self.agvs[agv_name].y:.1f})'
                )
                self._last_state[agv_name] = self.agvs[agv_name].state
    
    def supervisory_command_callback(self, msg: String):
        """
        Receive commands from supervisory control
        
        Expected JSON format:
        {
            "command": "get_status" | "request_service" | "ready_to_move" | "ready_to_return",
            "destination": "D" | "E" (for request_service),
            "agv_name": "Alvik1" (optional, for ready commands)
        }
        """
        try:
            cmd_data = json.loads(msg.data)
            command = cmd_data.get('command', '')
            
            if command == 'get_status':
                self.send_fleet_status()
            
            elif command == 'request_service':
                destination = cmd_data.get('destination', 'D')
                self.handle_service_request(destination)
            
            elif command == 'ready_to_move':
                # Part loaded, tell AGV to start moving
                if self.current_service_agv:
                    self.send_ready_command(self.current_service_agv)
                else:
                    self.get_logger().warn('No AGV in service to send ready_to_move')
            
            elif command == 'ready_to_return':
                # Part unloaded, tell AGV to return
                if self.current_service_agv:
                    self.send_ready_command(self.current_service_agv)
                    # After return completes, clear service tracking
                    # (will be handled when AGV reaches position C)
                else:
                    self.get_logger().warn('No AGV in service to send ready_to_return')
            
            else:
                self.get_logger().warn(f'Unknown command: {command}')
        
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in supervisory command: {msg.data}')
    
    def send_fleet_status(self):
        """Send current fleet status to supervisory"""
        fleet_data = {
            'response': 'fleet_status',
            'timestamp': time.time(),
            'agvs': [agv.to_dict() for agv in self.agvs.values()],
            'available_count': sum(1 for agv in self.agvs.values() if agv.available and agv.is_online()),
            'online_count': sum(1 for agv in self.agvs.values() if agv.is_online()),
            'current_service': self.current_service_agv
        }
        
        msg = String()
        msg.data = json.dumps(fleet_data)
        self.response_pub.publish(msg)
    
    def handle_service_request(self, destination: str):
        """
        Handle service request - find first available AGV at position A
        """
        # Find first available AGV at position A
        available_agv = None
        for agv in self.agvs.values():
            if agv.available and agv.is_online() and agv.position == "A":
                available_agv = agv
                break
        
        if available_agv:
            # Assign this AGV to service
            self.current_service_agv = available_agv.name
            self.current_destination = destination
            
            # Send deliver command to AGV
            self.send_deliver_command(available_agv.name, destination)
            
            # Send response to supervisory
            response = {
                'response': 'service_assigned',
                'agv_name': available_agv.name,
                'destination': destination,
                'position': {'x': available_agv.x, 'y': available_agv.y}
            }
            self.get_logger().info(f'Assigned {available_agv.name} to deliver to {destination}')
        else:
            # No AGV available
            response = {
                'response': 'no_agv_available',
                'online_count': sum(1 for agv in self.agvs.values() if agv.is_online())
            }
            self.get_logger().warn('No AGV available at position A for service')
        
        msg = String()
        msg.data = json.dumps(response)
        self.response_pub.publish(msg)
    
    def send_deliver_command(self, agv_name: str, destination: str):
        """Send deliver command to specific AGV"""
        if agv_name in self.command_publishers:
            msg = String()
            msg.data = f'deliver {destination}'
            self.command_publishers[agv_name].publish(msg)
            self.get_logger().info(f'Sent DELIVER {destination} to {agv_name}')
    
    def send_ready_command(self, agv_name: str):
        """Send READY command to specific AGV"""
        if agv_name in self.command_publishers:
            msg = String()
            msg.data = 'ready'
            self.command_publishers[agv_name].publish(msg)
            self.get_logger().info(f'Sent READY to {agv_name}')
    
    def send_advance_command(self, agv_name: str):
        """Send queue advancement command to specific AGV"""
        if agv_name in self.command_publishers:
            msg = String()
            msg.data = 'advance'
            self.command_publishers[agv_name].publish(msg)
            self.get_logger().info(f'Sent ADVANCE to {agv_name}')
    
    def check_queue_advancement(self):
        """
        Check if queue advancement is needed
        When an AGV at A starts moving, trigger B→A and C→B
        """
        # Find AGV states
        agv_at_a = None
        agv_at_b = None
        agv_at_c = None
        
        for agv in self.agvs.values():
            if agv.is_online():
                if agv.position == "A" and agv.state in ["WAITING_LOAD", "MOVING"]:
                    agv_at_a = agv
                elif agv.position == "B" and agv.state == "IDLE":
                    agv_at_b = agv
                elif agv.position == "C" and agv.state == "IDLE":
                    agv_at_c = agv
        
        # If A is in service and B is waiting, advance B→A
        if agv_at_a and agv_at_b:
            # Check if we should trigger advancement (only once per service)
            if not hasattr(self, '_advancement_triggered'):
                self._advancement_triggered = {}
            
            if not self._advancement_triggered.get(agv_at_a.name, False):
                # Trigger advancement after 2 seconds
                self.get_logger().info(f'Queue advancement: {agv_at_b.name} B→A')
                self.send_advance_command(agv_at_b.name)
                self._advancement_triggered[agv_at_a.name] = True
                
                # Also advance C→B if exists
                if agv_at_c:
                    self.get_logger().info(f'Queue advancement: {agv_at_c.name} C→B')
                    self.create_timer(2.0, lambda: self.send_advance_command(agv_at_c.name))
        
        # Check if service is complete (AGV returned to C and is IDLE)
        if self.current_service_agv:
            service_agv = self.agvs.get(self.current_service_agv)
            if service_agv and service_agv.position == "C" and service_agv.state == "IDLE":
                self.get_logger().info(f'Service complete: {self.current_service_agv} returned to C')
                self.current_service_agv = None
                self.current_destination = None
                if hasattr(self, '_advancement_triggered'):
                    self._advancement_triggered.clear()


def main(args=None):
    rclpy.init(args=args)
    node = FactoryAGVManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
