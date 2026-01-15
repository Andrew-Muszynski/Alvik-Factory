#!/usr/bin/env python3
"""
Supervisory.py - Factory Supervisory Control Node

Numeric command interface:
0 - Show fleet status
1 D/E - Request service (deliver to D or E)
2 - Ready to move (part loaded)
3 - Ready to return (part unloaded)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import select


class SupervisoryControl(Node):
    """High-level supervisory control with numeric command interface"""
    
    def __init__(self):
        super().__init__('supervisory_control')
        
        # Publisher to send commands to FactoryAGV
        self.command_pub = self.create_publisher(
            String,
            'factory_agv_command',
            10
        )
        
        # Subscriber to receive responses from FactoryAGV
        self.response_sub = self.create_subscription(
            String,
            'factory_agv_response',
            self.factory_response_callback,
            10
        )
        
        # Track current service
        self.current_agv = None
        self.current_destination = None
        self.service_state = "IDLE"
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('SUPERVISORY CONTROL - Numeric Commands')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Commands:')
        self.get_logger().info('  0           - Show fleet status')
        self.get_logger().info('  1 D         - Request service, deliver to D')
        self.get_logger().info('  1 E         - Request service, deliver to E')
        self.get_logger().info('  2           - Ready to move (part loaded)')
        self.get_logger().info('  3           - Ready to return (part unloaded)')
        self.get_logger().info('  q           - Quit')
        self.get_logger().info('=' * 70 + '\n')
        
        # Start command input in separate thread
        self.running = True
        import threading
        self.input_thread = threading.Thread(target=self.command_input_loop, daemon=True)
        self.input_thread.start()
    
    def factory_response_callback(self, msg: String):
        """Receive responses from FactoryAGV manager"""
        try:
            data = json.loads(msg.data)
            response_type = data.get('response', '')
            
            if response_type == 'fleet_status':
                self.display_fleet_status(data)
            
            elif response_type == 'service_assigned':
                agv_name = data.get('agv_name')
                destination = data.get('destination')
                position = data.get('position', {})
                
                self.current_agv = agv_name
                self.current_destination = destination
                self.service_state = "ASSIGNED"
                
                self.get_logger().info('')
                self.get_logger().info('=' * 70)
                self.get_logger().info(f'✓ SERVICE ASSIGNED')
                self.get_logger().info(f'  AGV: {agv_name}')
                self.get_logger().info(f'  Destination: {destination}')
                self.get_logger().info(f'  Position: ({position.get("x", 0):.1f}, {position.get("y", 0):.1f})')
                self.get_logger().info(f'  State: WAITING FOR LOAD')
                self.get_logger().info(f'  → Load part onto {agv_name}, then enter: 2')
                self.get_logger().info('=' * 70 + '\n')
            
            elif response_type == 'no_agv_available':
                online = data.get('online_count', 0)
                self.get_logger().warn('')
                self.get_logger().warn('=' * 70)
                self.get_logger().warn('✗ NO AGV AVAILABLE')
                self.get_logger().warn(f'  {online} robots online but none at position A')
                self.get_logger().warn('=' * 70 + '\n')
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in factory response')
    
    def display_fleet_status(self, data: dict):
        """Display formatted fleet status"""
        available = data.get('available_count', 0)
        online = data.get('online_count', 0)
        current_service = data.get('current_service')
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('FLEET STATUS')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Available: {available} | Online: {online} | In Service: {current_service or "None"}')
        self.get_logger().info('-' * 70)
        
        for agv in data.get('agvs', []):
            status_icon = '🟢' if agv['online'] else '🔴'
            avail_icon = '✓' if agv['available'] else '✗'
            
            self.get_logger().info(
                f"{status_icon} {agv['name']:<8} | "
                f"State: {agv['state']:<13} | "
                f"Pos: {agv['position']:<7} ({agv['x']:6.1f}, {agv['y']:6.1f}) | "
                f"Color: {agv['color']:<11} | Avail: {avail_icon}"
            )
        
        self.get_logger().info('=' * 70 + '\n')
    
    def send_command(self, command_dict: dict):
        """Send command to FactoryAGV"""
        msg = String()
        msg.data = json.dumps(command_dict)
        self.command_pub.publish(msg)
    
    def handle_command_0(self):
        """Command 0: Get fleet status"""
        self.get_logger().info('Requesting fleet status...')
        self.send_command({'command': 'get_status'})
    
    def handle_command_1(self, destination: str):
        """Command 1: Request service"""
        if destination not in ['D', 'E']:
            self.get_logger().error(f'Invalid destination: {destination}. Must be D or E')
            return
        
        self.get_logger().info(f'Requesting service to destination {destination}...')
        self.send_command({
            'command': 'request_service',
            'destination': destination
        })
    
    def handle_command_2(self):
        """Command 2: Ready to move (part loaded)"""
        if not self.current_agv:
            self.get_logger().error('No AGV in service. Request service first (command 1)')
            return
        
        self.get_logger().info(f'Part loaded. Sending {self.current_agv} to {self.current_destination}...')
        self.send_command({'command': 'ready_to_move'})
        self.service_state = "MOVING"
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'→ {self.current_agv} MOVING to {self.current_destination}')
        self.get_logger().info(f'  Queue will automatically advance (B→A, C→B)')
        self.get_logger().info(f'  Wait for arrival notification...')
        self.get_logger().info('=' * 70 + '\n')
    
    def handle_command_3(self):
        """Command 3: Ready to return (part unloaded)"""
        if not self.current_agv:
            self.get_logger().error('No AGV in service')
            return
        
        self.get_logger().info(f'Part unloaded. Sending {self.current_agv} back to queue...')
        self.send_command({'command': 'ready_to_return'})
        self.service_state = "RETURNING"
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'→ {self.current_agv} RETURNING to queue')
        self.get_logger().info(f'  Will stop at first available position (C or B)')
        self.get_logger().info(f'  Wait for service completion...')
        self.get_logger().info('=' * 70 + '\n')
        
        # Clear current service (will be reset when AGV returns)
        self.current_agv = None
        self.current_destination = None
    
    def command_input_loop(self):
        """Thread for handling keyboard input"""
        print("\n> ", end='', flush=True)
        
        while self.running:
            try:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline().strip()
                    if line:
                        self.process_command(line)
                        print("\n> ", end='', flush=True)
            except Exception as e:
                print(f"Input error: {e}")
                break
    
    def process_command(self, cmd_line: str):
        """Process a command from user input"""
        parts = cmd_line.upper().split()
        if not parts:
            return
        
        cmd = parts[0]
        
        try:
            if cmd == '0':
                self.handle_command_0()
            
            elif cmd == '1':
                if len(parts) < 2:
                    print("Usage: 1 D  or  1 E")
                    return
                destination = parts[1].upper()
                self.handle_command_1(destination)
            
            elif cmd == '2':
                self.handle_command_2()
            
            elif cmd == '3':
                self.handle_command_3()
            
            elif cmd in ['Q', 'QUIT', 'EXIT']:
                print("Shutting down supervisory control...")
                self.running = False
                rclpy.shutdown()
            
            else:
                print(f"Unknown command: {cmd}")
                print("Valid commands: 0, 1 D/E, 2, 3, q")
        
        except Exception as e:
            print(f"Error processing command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SupervisoryControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
