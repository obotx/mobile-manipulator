import pygame
import time
from typing import Dict

class RobotInterface:
    def __init__(self, config):
        pygame.init()
        self.screen = pygame.display.set_mode(config['window_size'])
        pygame.display.set_caption("Robot Control Interface")
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        self.large_font = pygame.font.Font(None, 32)
        self.last_update = time.time()
        self.update_interval = 1.0 / config['telemetry_update_rate']
        
    def update(self, mobile_state: Dict, gripper_state: Dict, control_mode: str):
        """Update the interface if enough time has passed"""
        current_time = time.time()
        if current_time - self.last_update < self.update_interval:
            return
            
        self.last_update = current_time
        
        # Clear screen
        self.screen.fill((30, 30, 30))
        
        # Draw interface elements
        self._draw_header(control_mode)
        self._draw_mobile_state(mobile_state)
        self._draw_gripper_state(gripper_state)
        self._draw_controls()
        
        pygame.display.flip()
    
    def _draw_header(self, control_mode: str):
        # Title
        title = self.large_font.render("Robot Control Interface", True, (255, 255, 255))
        self.screen.blit(title, (10, 10))
        
        # Current mode
        mode_color = (100, 255, 100) if control_mode == 'mobile' else (255, 100, 100)
        mode_text = self.font.render(f"Active Mode: {control_mode.upper()}", True, mode_color)
        self.screen.blit(mode_text, (10, 50))
    
    def _draw_mobile_state(self, state: Dict):
        # Draw mobile base status
        y_offset = 80
        
        # Target velocity
        target_vel = state.get('target_velocity', [0, 0, 0])
        vel_text = f"Target Vel: [x:{target_vel[0]:.2f}, y:{target_vel[1]:.2f}, ω:{target_vel[2]:.2f}]"
        text = self.small_font.render(vel_text, True, (180, 180, 180))
        self.screen.blit(text, (10, y_offset))
        
        # Wheel velocities
        wheel_vels = state.get('wheel_velocities', [0, 0, 0, 0])
        wheel_text = f"Wheel Vels: [FL:{wheel_vels[0]:.2f}, FR:{wheel_vels[1]:.2f}, BL:{wheel_vels[2]:.2f}, BR:{wheel_vels[3]:.2f}]"
        text = self.small_font.render(wheel_text, True, (180, 180, 180))
        self.screen.blit(text, (10, y_offset + 20))
        
        # Emergency stop status
        if state.get('emergency_stop', False) or state.get('manual_stop', False):
            stop_text = self.font.render("EMERGENCY STOP ACTIVE", True, (255, 0, 0))
            self.screen.blit(stop_text, (10, y_offset + 45))
    
    def _draw_gripper_state(self, state: Dict):
        # Draw gripper status
        y_offset = 160
        
        # Gripper state
        gripper_state = state.get('state', 'unknown')
        state_color = (100, 255, 100) if gripper_state == 'open' else (255, 100, 100)
        status_text = f"Gripper: {gripper_state.upper()}"
        text = self.font.render(status_text, True, state_color)
        self.screen.blit(text, (10, y_offset))
        
        # Movement progress
        if state.get('is_moving', False):
            progress = state.get('progress', 0.0)
            progress_text = f"Moving... {progress*100:.1f}%"
            text = self.small_font.render(progress_text, True, (255, 255, 0))
            self.screen.blit(text, (10, y_offset + 25))
    
    def _draw_controls(self):
        # Draw control instructions
        y_offset = 220
        
        controls = [
            "CONTROLS:",
            "TAB - Switch control mode",
            "ESC - Emergency stop",
            "",
            "MOBILE MODE:",
            "W/S - Forward/Backward",
            "A/D - Strafe Left/Right", 
            "Q/E - Rotate Left/Right",
            "+/- - Speed adjustment",
            "",
            "GRIPPER MODE:",
            "G - Close gripper",
            "R - Open gripper"
        ]
        
        for i, line in enumerate(controls):
            color = (255, 255, 255) if line.endswith(":") else (150, 150, 150)
            if line == "":
                continue
            text = self.small_font.render(line, True, color)
            self.screen.blit(text, (10, y_offset + i * 16))
    
    def handle_events(self):
        """Handle pygame events and return key events"""
        events = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                events.append(('quit', None))
            elif event.type == pygame.KEYDOWN:
                events.append(('keydown', event.key))
                # Special handling for emergency stop reset
                if event.key == pygame.K_SPACE:
                    events.append(('reset_emergency', None))
            elif event.type == pygame.KEYUP:
                events.append(('keyup', event.key))
        return events
    
    def close(self):
        pygame.quit()
    