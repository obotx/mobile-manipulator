import mujoco
import mujoco.viewer
import pygame
import time
import signal
import sys
from .config import load_config
from .mobile_base import MobileBaseController
from .gripper import GripperController
from .interface import RobotInterface

class RobotControlSystem:
    # def __init__(self, model_file: str = "/home/newton/ros2_work_ws/DIY_Arm_ObotX/src/DIY-Arm-ObotX/arm_obotx_on_mobile_base/arm_obotx_with_mobile_base.xml"):
    def __init__(self, model_file: str = "arm_obotx_with_mobile_base.xml"):
        # Load configuration
        self.config = load_config()
        
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)
        
        # Initialize controllers
        self.mobile_base = MobileBaseController(
            self.model, self.data, self.config['mobile'])
        self.gripper = GripperController(
            self.model, self.data, self.config['gripper'])
        self.interface = RobotInterface(self.config['interface'])
        
        # System state
        self.control_mode = 'mobile'
        self.running = True
        self.emergency_stop = False
        self.speed_multiplier = 2.0
        
        # Continuous movement tracking
        self.keys_pressed = set()
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("Robot Control System initialized")
        print("Controls:")
        print("  TAB: Switch between mobile/gripper control")
        print("  Mobile mode: WASD for movement, QE for rotation")
        print("  Gripper mode: G to close, R to open")
        print("  ESC: Emergency stop")
        print("  +/-: Adjust speed multiplier")
    
    def _signal_handler(self, signum, frame):
        print("\nShutting down...")
        self.running = False
    
    def _handle_key_press(self, key):
        """Handle key press events"""
        self.keys_pressed.add(key)
        
        # Mode switching
        if key == pygame.K_TAB:
            self.control_mode = 'gripper' if self.control_mode == 'mobile' else 'mobile'
            print(f"Switched to {self.control_mode} mode")
        
        # Emergency stop
        elif key == pygame.K_ESCAPE:
            self.mobile_base.emergency_stop()
            self.emergency_stop = True
            print("Emergency stop activated!")
        
        # Speed control
        elif key == pygame.K_PLUS or key == pygame.K_EQUALS:
            self.speed_multiplier = min(3.0, self.speed_multiplier + 0.1)
            print(f"Speed multiplier: {self.speed_multiplier:.1f}")
        elif key == pygame.K_MINUS:
            self.speed_multiplier = max(0.1, self.speed_multiplier - 0.1)
            print(f"Speed multiplier: {self.speed_multiplier:.1f}")
        
        # Gripper controls (immediate response)
        elif self.control_mode == 'gripper':
            if key == pygame.K_g:
                self.gripper.set_state('closed')
                print("Gripper closing...")
            elif key == pygame.K_r:
                self.gripper.set_state('open')
                print("Gripper opening...")
    
    def _handle_key_release(self, key):
        """Handle key release events"""
        self.keys_pressed.discard(key)
    
    def _update_continuous_movement(self):
        """Update movement based on currently pressed keys"""
        if self.control_mode != 'mobile' or self.emergency_stop:
            return
        
        # Calculate current speeds with multiplier
        v_fwd = self.config['mobile']['v_forward'] * self.speed_multiplier
        v_str = self.config['mobile']['v_strafe'] * self.speed_multiplier
        v_rot = self.config['mobile']['v_rotate'] * self.speed_multiplier
        
        # Initialize velocities
        vx, vy, omega = 0.0, 0.0, 0.0
        
        # Check pressed keys for movement
        if pygame.K_w in self.keys_pressed:
            vx += v_fwd
        if pygame.K_s in self.keys_pressed:
            vx -= v_fwd
        if pygame.K_a in self.keys_pressed:
            vy += v_str
        if pygame.K_d in self.keys_pressed:
            vy -= v_str
        if pygame.K_q in self.keys_pressed:
            omega += v_rot
        if pygame.K_e in self.keys_pressed:
            omega -= v_rot

        print(f"Current velocities: vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")
        
        # Apply velocity command
        self.mobile_base.set_velocity_command(vx, vy, omega)
    
    def _reset_emergency_stop(self):
        """Reset emergency stop"""
        self.emergency_stop = False
        self.mobile_base.manual_stop_flag = False
        print("Emergency stop reset")
    
    def run(self):
        """Main control loop"""
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                prev_time = time.time()
                
                print("Starting control loop...")
                
                while viewer.is_running() and self.running:
                    # Calculate time step
                    current_time = time.time()
                    dt = current_time - prev_time
                    prev_time = current_time
                    
                    # Handle interface events
                    for event_type, event_data in self.interface.handle_events():
                        if event_type == 'quit':
                            self.running = False
                        elif event_type == 'keydown':
                            self._handle_key_press(event_data)
                        elif event_type == 'keyup':
                            self._handle_key_release(event_data)
                        elif event_type == 'reset_emergency' and self.emergency_stop:
                            self._reset_emergency_stop()
                    
                    # Handle continuous movement
                    self._update_continuous_movement()
                    
                    # Apply controls if not in emergency stop
                    if not self.emergency_stop:
                        self.mobile_base.apply_control(dt)
                        self.gripper.apply_control()
                    
                    # Step simulation
                    mujoco.mj_step(self.model, self.data)
                    
                    # Sync viewer
                    viewer.sync()
                    
                    # Update interface
                    mobile_status = self.mobile_base.get_status()
                    gripper_status = self.gripper.get_status()
                    
                    self.interface.update(mobile_status, gripper_status, self.control_mode)
                    
                    # Small delay to prevent excessive CPU usage
                    time.sleep(0.001)
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error in control loop: {e}")
        finally:
            self.interface.close()
            print("Control system shut down")

def main():
    try:
        control_system = RobotControlSystem()
        control_system.run()
    except Exception as e:
        print(f"Failed to start control system: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()