import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
from dataclasses import dataclass
from typing import List, Tuple, Dict
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time

@dataclass
class WheelState:
    """State of individual wheel"""
    position: np.ndarray = None  # [x, y, z]
    velocity: np.ndarray = None  # [vx, vy, vz]
    suspension_compression: float = 0.0  # meters
    suspension_velocity: float = 0.0  # m/s
    steering_angle: float = 0.0  # radians
    motor_torque: float = 0.0  # Nm
    ground_force: np.ndarray = None  # [fx, fy, fz]
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.ground_force is None:
            self.ground_force = np.zeros(3)

@dataclass
class VehicleParams:
    """Advanced vehicle parameters"""
    # Body parameters
    mass: float = 1500.0  # kg
    length: float = 4.5  # meters
    width: float = 1.8   # meters
    height: float = 1.5  # meters
    wheelbase: float = 2.7  # meters
    track_width: float = 1.6  # meters
    
    # Inertia (kg⋅m²)
    Ixx: float = 500.0   # Roll inertia
    Iyy: float = 2000.0  # Pitch inertia
    Izz: float = 2500.0  # Yaw inertia
    
    # Center of gravity (from rear axle, meters)
    cg_x: float = 1.35   # Forward from rear axle
    cg_y: float = 0.0    # Lateral offset
    cg_z: float = 0.5    # Height above ground
    
    # Suspension parameters (per wheel)
    spring_rate: float = 25000.0  # N/m
    damping_rate: float = 3000.0  # N⋅s/m
    max_compression: float = 0.3  # meters
    unloaded_height: float = 0.4  # meters
    
    # Tire parameters
    tire_radius: float = 0.32  # meters
    tire_stiffness: float = 180000.0  # N/rad (lateral)
    rolling_resistance: float = 0.01
    friction_coefficient: float = 0.8
    
    # Motor parameters
    max_motor_torque: float = 300.0  # Nm per motor
    motor_efficiency: float = 0.9
    gear_ratio: float = 10.0
    
    # Aerodynamics
    drag_coefficient: float = 0.3
    frontal_area: float = 2.5  # m²
    air_density: float = 1.225  # kg/m³

@dataclass
class PayloadConfig:
    """Payload configuration affecting CG"""
    mass: float = 0.0  # kg
    position: np.ndarray = None  # [x, y, z] relative to vehicle center
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.array([0.0, 0.0, 0.0])

class AdvancedVehicle:
    """Advanced 4-wheel vehicle with dual motors and full suspension"""
    
    def __init__(self, params: VehicleParams):
        self.params = params
        
        # Vehicle state [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        self.state = np.zeros(12)
        
        # Wheel states (FL, FR, RL, RR)
        self.wheels = [WheelState() for _ in range(4)]
        self.wheel_names = ['FL', 'FR', 'RL', 'RR']
        
        # Payload configuration
        self.payload = PayloadConfig()
        
        # Control inputs
        self.front_steering = 0.0  # radians
        self.rear_steering = 0.0   # radians
        self.front_motor_torque = 0.0  # Nm
        self.rear_motor_torque = 0.0   # Nm
        
        # Store commanded torques for visualization
        self._commanded_front_torque = 0.0
        self._commanded_rear_torque = 0.0
        
        # Initialize wheel positions
        self._update_wheel_positions()
        
        # History for plotting
        self.time_history = []
        self.state_history = []
        self.wheel_history = []
        self.control_history = []
        
    def _update_wheel_positions(self):
        """Update wheel positions based on vehicle state"""
        x, y, z, roll, pitch, yaw = self.state[:6]
        
        # Rotation matrix from body to world frame
        R = self._rotation_matrix(roll, pitch, yaw)
        
        # Wheel positions in body frame
        wheel_positions_body = np.array([
            [self.params.wheelbase, self.params.track_width/2, -self.params.unloaded_height],   # FL
            [self.params.wheelbase, -self.params.track_width/2, -self.params.unloaded_height],  # FR
            [0, self.params.track_width/2, -self.params.unloaded_height],                       # RL
            [0, -self.params.track_width/2, -self.params.unloaded_height]                       # RR
        ])
        
        # Transform to world frame
        vehicle_position = np.array([x, y, z])
        for i, wheel_pos_body in enumerate(wheel_positions_body):
            wheel_pos_world = vehicle_position + R @ wheel_pos_body
            self.wheels[i].position = wheel_pos_world
            
    def _rotation_matrix(self, roll, pitch, yaw):
        """Create rotation matrix from Euler angles"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        return R
        
    def set_payload(self, mass: float, position: np.ndarray):
        """Set payload configuration"""
        self.payload.mass = mass
        self.payload.position = position.copy()
        
    def calculate_total_mass_and_cg(self):
        """Calculate total mass and center of gravity including payload"""
        total_mass = self.params.mass + self.payload.mass
        
        if self.payload.mass > 0:
            # Weighted average of CG positions
            vehicle_cg = np.array([self.params.cg_x, self.params.cg_y, self.params.cg_z])
            total_cg = (self.params.mass * vehicle_cg + self.payload.mass * self.payload.position) / total_mass
        else:
            total_cg = np.array([self.params.cg_x, self.params.cg_y, self.params.cg_z])
            
        return total_mass, total_cg
        
    def calculate_suspension_forces(self):
        """Calculate suspension forces for each wheel"""
        forces = []
        
        for i, wheel in enumerate(self.wheels):
            # Ground height (simplified as flat)
            ground_height = 0.0
            wheel_height = wheel.position[2]
            
            # Compression (positive = compressed)
            compression = self.params.unloaded_height - (wheel_height - ground_height)
            compression = np.clip(compression, 0, self.params.max_compression)
            
            # Spring force (upward when compressed)
            spring_force = self.params.spring_rate * compression
            
            # Damper force (opposes vertical velocity)
            damper_force = self.params.damping_rate * wheel.suspension_velocity
            
            # Total vertical force
            total_force = spring_force + damper_force
            
            # Update wheel state
            wheel.suspension_compression = compression
            
            # Force vector (vertical component only for now)
            force_vector = np.array([0, 0, total_force])
            wheel.ground_force = force_vector
            forces.append(force_vector)
            
        return forces
        
    def calculate_tire_forces(self, dt):
        """Calculate tire forces based on slip and steering"""
        tire_forces = []
        
        vx, vy, vz, wx, wy, wz = self.state[6:12]
        
        for i, wheel in enumerate(self.wheels):
            # Determine steering angle
            if i < 2:  # Front wheels
                steer_angle = self.front_steering
            else:  # Rear wheels
                steer_angle = self.rear_steering
                
            wheel.steering_angle = steer_angle
            
            # Calculate slip angle (simplified)
            if abs(vx) > 0.1:  # Avoid division by zero
                slip_angle = np.arctan2(vy, vx) - steer_angle
            else:
                slip_angle = 0
                
            # Lateral force (simplified Pacejka-like model)
            lateral_force = -self.params.tire_stiffness * slip_angle
            lateral_force = np.clip(lateral_force, -5000, 5000)  # Limit tire force
            
            # Longitudinal force from motor torque
            if i < 2:  # Front wheels
                motor_torque = self.front_motor_torque / 2  # Split between wheels
            else:  # Rear wheels
                motor_torque = self.rear_motor_torque / 2
                
            wheel.motor_torque = motor_torque
            
            # Convert motor torque to wheel force
            wheel_force = motor_torque * self.params.gear_ratio / self.params.tire_radius
            
            # Rolling resistance
            rolling_force = -self.params.rolling_resistance * wheel.ground_force[2] * np.sign(vx)
            
            # Total longitudinal force
            longitudinal_force = wheel_force + rolling_force
            
            # Combine forces in wheel coordinate system
            fx_wheel = longitudinal_force
            fy_wheel = lateral_force
            
            # Transform to vehicle coordinates
            cos_steer = np.cos(steer_angle)
            sin_steer = np.sin(steer_angle)
            
            fx_vehicle = fx_wheel * cos_steer - fy_wheel * sin_steer
            fy_vehicle = fx_wheel * sin_steer + fy_wheel * cos_steer
            
            tire_force = np.array([fx_vehicle, fy_vehicle, 0])
            tire_forces.append(tire_force)
            
        return tire_forces
        
    def calculate_aerodynamic_forces(self):
        """Calculate aerodynamic drag"""
        vx, vy, vz = self.state[6:9]
        velocity_magnitude = np.sqrt(vx**2 + vy**2)
        
        if velocity_magnitude > 0.1:
            # Drag force opposing motion
            drag_magnitude = 0.5 * self.params.air_density * self.params.frontal_area * \
                           self.params.drag_coefficient * velocity_magnitude**2
            
            # Drag direction opposite to velocity
            drag_direction = -np.array([vx, vy, 0]) / velocity_magnitude
            drag_force = drag_magnitude * drag_direction
        else:
            drag_force = np.zeros(3)
            
        return drag_force
        
    def update_dynamics(self, dt, current_time=None):
        """Update vehicle dynamics using simplified rigid body equations"""
        # Get current state
        x, y, z, roll, pitch, yaw = self.state[:6]
        vx, vy, vz, wx, wy, wz = self.state[6:12]
        
        # Calculate total mass and CG
        total_mass, total_cg = self.calculate_total_mass_and_cg()
        
        # Update wheel positions
        self._update_wheel_positions()
        
        # Calculate forces
        suspension_forces = self.calculate_suspension_forces()
        tire_forces = self.calculate_tire_forces(dt)
        aero_force = self.calculate_aerodynamic_forces()
        
        # Sum all forces
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        
        # Add tire and suspension forces
        for i, (tire_f, susp_f) in enumerate(zip(tire_forces, suspension_forces)):
            wheel_pos = self.wheels[i].position
            vehicle_center = np.array([x, y, z])
            
            # Force contribution
            total_force += tire_f + susp_f
            
            # Moment contribution (r × F)
            r = wheel_pos - vehicle_center
            total_moment += np.cross(r, tire_f + susp_f)
            
        # Add aerodynamic drag
        total_force += aero_force
        
        # Add gravity
        gravity_force = np.array([0, 0, -total_mass * 9.81])
        total_force += gravity_force
        
        # Calculate accelerations
        ax = total_force[0] / total_mass
        ay = total_force[1] / total_mass
        az = total_force[2] / total_mass
        
        # Angular accelerations (simplified)
        alpha_x = total_moment[0] / self.params.Ixx
        alpha_y = total_moment[1] / self.params.Iyy
        alpha_z = total_moment[2] / self.params.Izz
        
        # Integrate velocities
        self.state[6] += ax * dt  # vx
        self.state[7] += ay * dt  # vy
        self.state[8] += az * dt  # vz
        self.state[9] += alpha_x * dt  # wx
        self.state[10] += alpha_y * dt  # wy
        self.state[11] += alpha_z * dt  # wz
        
        # Integrate positions
        self.state[0] += self.state[6] * dt  # x
        self.state[1] += self.state[7] * dt  # y
        self.state[2] += self.state[8] * dt  # z
        self.state[3] += self.state[9] * dt  # roll
        self.state[4] += self.state[10] * dt  # pitch
        self.state[5] += self.state[11] * dt  # yaw
        
        # Store history with synchronized time
        if current_time is not None:
            self.time_history.append(current_time)
        else:
            # Fallback: calculate time from history length
            self.time_history.append(len(self.time_history) * dt)
            
        self.state_history.append(self.state.copy())
        self.wheel_history.append([
            {
                'compression': w.suspension_compression,
                'steering': w.steering_angle,
                'torque': w.motor_torque,
                'force': w.ground_force.copy()
            } for w in self.wheels
        ])
        self.control_history.append({
            'front_steering': self.front_steering,
            'rear_steering': self.rear_steering,
            'front_torque': self.front_motor_torque,
            'rear_torque': self.rear_motor_torque,
            'commanded_front_torque': getattr(self, '_commanded_front_torque', self.front_motor_torque),
            'commanded_rear_torque': getattr(self, '_commanded_rear_torque', self.rear_motor_torque)
        })
        
        # Ensure all history arrays have the same length
        min_length = min(len(self.time_history), len(self.state_history), 
                        len(self.wheel_history), len(self.control_history))
        
        if len(self.time_history) > min_length:
            self.time_history = self.time_history[:min_length]
        if len(self.state_history) > min_length:
            self.state_history = self.state_history[:min_length]
        if len(self.wheel_history) > min_length:
            self.wheel_history = self.wheel_history[:min_length]
        if len(self.control_history) > min_length:
            self.control_history = self.control_history[:min_length]
            
    def reset_vehicle(self):
        """Reset vehicle state and history"""
        self.state = np.zeros(12)
        self.time_history.clear()
        self.state_history.clear()
        self.wheel_history.clear()
        self.control_history.clear()
        
        # Reset wheel states
        for wheel in self.wheels:
            wheel.position = np.zeros(3)
            wheel.velocity = np.zeros(3)
            wheel.suspension_compression = 0.0
            wheel.suspension_velocity = 0.0
            wheel.steering_angle = 0.0
            wheel.motor_torque = 0.0
            wheel.ground_force = np.zeros(3)
            
        # Reset control inputs
        self.front_steering = 0.0
        self.rear_steering = 0.0
        self.front_motor_torque = 0.0
        self.rear_motor_torque = 0.0
        self._commanded_front_torque = 0.0
        self._commanded_rear_torque = 0.0

class VehicleController:
    """PID-based vehicle controller for path following"""
    
    def __init__(self, vehicle: AdvancedVehicle):
        self.vehicle = vehicle
        
        # PID parameters for different control loops (increased gains for better response)
        self.speed_pid = {'kp': 1000, 'ki': 100, 'kd': 200}  # More aggressive speed control
        self.heading_pid = {'kp': 5000, 'ki': 200, 'kd': 1000}  # More aggressive steering
        self.lateral_pid = {'kp': 1500, 'ki': 50, 'kd': 300}
        
        # Motor control parameters
        self.torque_split = 0.6  # 60% front, 40% rear by default
        self.motor_response_time = 0.1  # Motor response lag in seconds
        
        # Control state
        self.target_speed = 0.0
        self.target_heading = 0.0
        self.target_position = np.array([0.0, 0.0])
        
        # PID error accumulation
        self.speed_error_sum = 0.0
        self.heading_error_sum = 0.0
        self.lateral_error_sum = 0.0
        self.prev_speed_error = 0.0
        self.prev_heading_error = 0.0
        self.prev_lateral_error = 0.0
        
        # Motor command filtering (for realistic response)
        self.commanded_front_torque = 0.0
        self.commanded_rear_torque = 0.0
        self.actual_front_torque = 0.0
        self.actual_rear_torque = 0.0
        
    def set_targets(self, speed: float, heading: float, position: np.ndarray = None):
        """Set control targets"""
        self.target_speed = speed
        self.target_heading = heading
        if position is not None:
            self.target_position = position.copy()
            
    def update_control(self, dt):
        """Update control outputs based on current state"""
        # Current state
        x, y, z, roll, pitch, yaw = self.vehicle.state[:6]
        vx, vy, vz, wx, wy, wz = self.vehicle.state[6:12]
        
        current_speed = np.sqrt(vx**2 + vy**2)
        current_heading = yaw
        current_position = np.array([x, y])
        
        # Speed control (longitudinal)
        speed_error = self.target_speed - current_speed
        self.speed_error_sum += speed_error * dt
        speed_error_diff = (speed_error - self.prev_speed_error) / dt if dt > 0 else 0
        
        speed_output = (self.speed_pid['kp'] * speed_error + 
                       self.speed_pid['ki'] * self.speed_error_sum + 
                       self.speed_pid['kd'] * speed_error_diff)
        
        # Distribute torque between front and rear motors using GUI parameter
        total_torque = np.clip(speed_output, -self.vehicle.params.max_motor_torque * 2, 
                              self.vehicle.params.max_motor_torque * 2)
        
        # Apply torque split from GUI (front_split is 0.0 to 1.0)
        front_split = self.torque_split
        rear_split = 1.0 - front_split
        
        # Command new torques
        self.commanded_front_torque = total_torque * front_split
        self.commanded_rear_torque = total_torque * rear_split
        
        # Apply motor response lag (first-order filter)
        if self.motor_response_time > 0:
            alpha = dt / (self.motor_response_time + dt)  # Filter coefficient
            self.actual_front_torque += alpha * (self.commanded_front_torque - self.actual_front_torque)
            self.actual_rear_torque += alpha * (self.commanded_rear_torque - self.actual_rear_torque)
        else:
            self.actual_front_torque = self.commanded_front_torque
            self.actual_rear_torque = self.commanded_rear_torque
        
        # Apply efficiency losses
        efficiency = self.vehicle.params.motor_efficiency
        self.vehicle.front_motor_torque = self.actual_front_torque * efficiency
        self.vehicle.rear_motor_torque = self.actual_rear_torque * efficiency
        
        # Store commanded torques for visualization
        self.vehicle._commanded_front_torque = self.commanded_front_torque
        self.vehicle._commanded_rear_torque = self.commanded_rear_torque
        
        # Heading control
        heading_error = self.target_heading - current_heading
        # Wrap angle to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        self.heading_error_sum += heading_error * dt
        heading_error_diff = (heading_error - self.prev_heading_error) / dt if dt > 0 else 0
        
        heading_output = (self.heading_pid['kp'] * heading_error + 
                         self.heading_pid['ki'] * self.heading_error_sum + 
                         self.heading_pid['kd'] * heading_error_diff)
        
        # Convert to steering angles
        max_steer = np.radians(30)  # 30 degrees max
        front_steer = np.clip(heading_output / 10000, -max_steer, max_steer)
        
        # 4-wheel steering: rear steers opposite direction for better maneuverability
        rear_steer_ratio = 0.3  # Rear steers 30% of front angle in opposite direction
        rear_steer = -front_steer * rear_steer_ratio
        
        self.vehicle.front_steering = front_steer
        self.vehicle.rear_steering = rear_steer
        
        # Store previous errors
        self.prev_speed_error = speed_error
        self.prev_heading_error = heading_error

class VehicleSimulator:
    """Main simulator class"""
    
    def __init__(self):
        self.vehicle = None
        self.controller = None
        self.time = 0.0
        self.dt = 0.01
        self.running = False
        
    def create_vehicle(self, params: VehicleParams = None):
        """Create vehicle with given parameters"""
        if params is None:
            params = VehicleParams()
        self.vehicle = AdvancedVehicle(params)
        self.controller = VehicleController(self.vehicle)
        self.time = 0.0  # Reset time
        
    def reset_simulation(self):
        """Reset simulation state"""
        if self.vehicle:
            self.vehicle.reset_vehicle()
        if self.controller:
            self.controller.speed_error_sum = 0.0
            self.controller.heading_error_sum = 0.0
            self.controller.lateral_error_sum = 0.0
            self.controller.prev_speed_error = 0.0
            self.controller.prev_heading_error = 0.0
            self.controller.prev_lateral_error = 0.0
            
            # Reset motor states
            self.controller.commanded_front_torque = 0.0
            self.controller.commanded_rear_torque = 0.0
            self.controller.actual_front_torque = 0.0
            self.controller.actual_rear_torque = 0.0
        self.time = 0.0
        self.running = False
        
    def run_scenario(self, scenario_name: str, duration: float = 10.0):
        """Run predefined scenarios"""
        if self.vehicle is None:
            self.create_vehicle()
            
        # Reset vehicle and time before starting
        self.reset_simulation()
        
        scenarios = {
            'acceleration': self._acceleration_test,
            'cornering': self._cornering_test,
            'slalom': self._slalom_test,
            'payload_test': self._payload_test,
            'suspension_test': self._suspension_test
        }
        
        if scenario_name in scenarios:
            scenarios[scenario_name](duration)
        else:
            print(f"Unknown scenario: {scenario_name}")
            
    def _acceleration_test(self, duration):
        """Test acceleration performance"""
        self._initialize_vehicle_for_test(initial_speed=0.0)
        self.controller.set_targets(speed=20.0, heading=0.0)  # 20 m/s target
        self._run_simulation(duration)
        
    def _cornering_test(self, duration):
        """Test cornering at constant speed"""
        self._initialize_vehicle_for_test(initial_speed=10.0)
        self.controller.set_targets(speed=15.0, heading=0.0)
        
        # Gradually change heading during simulation
        for i in range(int(duration / self.dt)):
            current_time = i * self.dt
            
            if current_time > 2.0:  # Start turning after 2 seconds
                target_heading = np.pi/3 * np.sin(0.3 * current_time)  # Wider sinusoidal steering
                self.controller.set_targets(speed=15.0, heading=target_heading)
            
            self.controller.update_control(self.dt)
            self.vehicle.update_dynamics(self.dt, current_time)
            self.time += self.dt
            
    def _slalom_test(self, duration):
        """Test slalom maneuver"""
        self._initialize_vehicle_for_test(initial_speed=8.0)
        self.controller.set_targets(speed=12.0, heading=0.0)
        
        for i in range(int(duration / self.dt)):
            current_time = i * self.dt
            
            # More aggressive slalom pattern
            target_heading = np.pi/4 * np.sin(0.6 * current_time)  # ±45 degrees, faster oscillation
            self.controller.set_targets(speed=12.0, heading=target_heading)
            
            self.controller.update_control(self.dt)
            self.vehicle.update_dynamics(self.dt, current_time)
            self.time += self.dt
            
    def _payload_test(self, duration):
        """Test with different payload configurations"""
        self._initialize_vehicle_for_test(initial_speed=5.0)
        
        # Test with high payload affecting CG
        payload_mass = 500.0  # kg
        payload_position = np.array([1.0, 0.0, 1.5])  # High and forward
        self.vehicle.set_payload(payload_mass, payload_position)
        
        self.controller.set_targets(speed=15.0, heading=0.0)
        self._run_simulation(duration/2)
        
        # Change to low payload
        payload_position = np.array([1.0, 0.0, 0.2])  # Low
        self.vehicle.set_payload(payload_mass, payload_position)
        
        # Continue simulation
        self._run_simulation(duration/2)
        
    def _suspension_test(self, duration):
        """Test suspension response over rough terrain"""
        self._initialize_vehicle_for_test(initial_speed=5.0)
        self.controller.set_targets(speed=10.0, heading=0.0)
        
        # Simulate by adding vertical disturbances
        for i in range(int(duration / self.dt)):
            current_time = i * self.dt
            
            # Add random road disturbances
            road_disturbance = 0.05 * np.sin(5 * current_time) + 0.02 * np.random.randn()
            
            # Apply disturbance to vehicle
            self.vehicle.state[2] += road_disturbance * self.dt
            
            self.controller.update_control(self.dt)
            self.vehicle.update_dynamics(self.dt, current_time)
            self.time += self.dt
            
    def _run_simulation(self, duration):
        """Run basic simulation loop"""
        start_time = self.time
        for i in range(int(duration / self.dt)):
            current_time = start_time + i * self.dt
            self.controller.update_control(self.dt)
            self.vehicle.update_dynamics(self.dt, current_time)
            self.time += self.dt
            
    def _initialize_vehicle_for_test(self, initial_speed=0.0):
        """Initialize vehicle with proper starting conditions"""
        # Set vehicle on ground with slight height
        self.vehicle.state[2] = 0.5  # Z position (height above ground)
        
        # Set initial forward velocity if specified
        if initial_speed > 0:
            self.vehicle.state[6] = initial_speed  # Forward velocity (vx)
            
        # Ensure other states are reasonable
        self.vehicle.state[7] = 0.0  # Lateral velocity (vy)
        self.vehicle.state[8] = 0.0  # Vertical velocity (vz)
        
        # Reset orientations to level
        self.vehicle.state[3] = 0.0  # Roll
        self.vehicle.state[4] = 0.0  # Pitch  
        self.vehicle.state[5] = 0.0  # Yaw (heading)
        
        # Reset angular velocities
        self.vehicle.state[9] = 0.0   # Roll rate
        self.vehicle.state[10] = 0.0  # Pitch rate
        self.vehicle.state[11] = 0.0  # Yaw rate

class VehicleVisualizer:
    """Advanced visualization for vehicle simulation"""
    
    def __init__(self, simulator: VehicleSimulator):
        self.simulator = simulator
        self.fig = None
        self.axes = []
        self.vehicle_patch = None
        self.wheel_patches = []
        self.animation = None
        
    def create_plots(self):
        """Create visualization plots"""
        self.fig, self.axes = plt.subplots(2, 3, figsize=(18, 12))
        self.fig.suptitle('Advanced Vehicle Dynamics Simulation', fontsize=16)
        
        # Flatten axes for easier indexing
        self.axes = self.axes.flatten()
        
        return self.fig, self.axes
        
    def setup_vehicle_view(self, ax):
        """Setup top-down vehicle view"""
        ax.set_aspect('equal')
        ax.set_xlim(-10, 50)
        ax.set_ylim(-20, 20)
        ax.grid(True, alpha=0.3)
        ax.set_title('Vehicle Top View')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        
        # Create vehicle body patch
        self.vehicle_patch = Rectangle((0, 0), 4.5, 1.8, 
                                     fill=True, color='blue', alpha=0.7)
        ax.add_patch(self.vehicle_patch)
        
        # Create wheel patches
        self.wheel_patches = []
        for i in range(4):
            wheel = Circle((0, 0), 0.3, fill=True, color='black')
            ax.add_patch(wheel)
            self.wheel_patches.append(wheel)
            
        return ax
        
    def update_plots(self):
        """Update all plots with current simulation data"""
        if not self.simulator.vehicle or not self.simulator.vehicle.time_history:
            return
            
        vehicle = self.simulator.vehicle
        
        # Ensure all history arrays have the same length
        min_length = min(len(vehicle.time_history), len(vehicle.state_history),
                        len(vehicle.wheel_history), len(vehicle.control_history))
        
        if min_length == 0:
            return
            
        # Truncate arrays to same length
        time = np.array(vehicle.time_history[:min_length])
        states = np.array(vehicle.state_history[:min_length])
        wheel_history = vehicle.wheel_history[:min_length]
        control_history = vehicle.control_history[:min_length]
        
        # Clear all axes
        for ax in self.axes:
            ax.clear()
            
        # Plot 1: Vehicle trajectory
        ax = self.axes[0]
        if len(states) > 0:
            # Plot full trajectory
            ax.plot(states[:, 0], states[:, 1], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
            
            # Plot recent trajectory (last 50 points) in brighter color
            if len(states) > 50:
                recent_states = states[-50:]
                ax.plot(recent_states[:, 0], recent_states[:, 1], 'cyan', linewidth=3, alpha=0.8, label='Recent Path')
            
            # Current position
            current_x, current_y = states[-1, 0], states[-1, 1]
            ax.plot(current_x, current_y, 'ro', markersize=10, label='Current Position', zorder=5)
            
            # Starting position
            ax.plot(states[0, 0], states[0, 1], 'go', markersize=8, label='Start Position', zorder=5)
            
            # Draw vehicle orientation with larger arrow
            yaw = states[-1, 5]
            arrow_length = max(3.0, (ax.get_xlim()[1] - ax.get_xlim()[0]) * 0.05)
            dx = arrow_length * np.cos(yaw)
            dy = arrow_length * np.sin(yaw)
            ax.arrow(current_x, current_y, dx, dy, 
                    head_width=arrow_length*0.3, head_length=arrow_length*0.2, 
                    fc='red', ec='red', linewidth=2, zorder=6)
            
            # Draw vehicle body outline (rectangle representing vehicle)
            vehicle_length = 4.5
            vehicle_width = 1.8
            
            # Vehicle corners in body frame
            corners = np.array([
                [-vehicle_length/2, -vehicle_width/2],
                [vehicle_length/2, -vehicle_width/2],
                [vehicle_length/2, vehicle_width/2],
                [-vehicle_length/2, vehicle_width/2],
                [-vehicle_length/2, -vehicle_width/2]  # Close the rectangle
            ])
            
            # Rotate corners based on vehicle yaw
            cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            rotated_corners = corners @ rotation_matrix.T
            
            # Translate to vehicle position
            vehicle_outline_x = rotated_corners[:, 0] + current_x
            vehicle_outline_y = rotated_corners[:, 1] + current_y
            
            ax.plot(vehicle_outline_x, vehicle_outline_y, 'k-', linewidth=2, alpha=0.8, zorder=4)
            
            # Set appropriate axis limits with some padding
            x_min, x_max = np.min(states[:, 0]), np.max(states[:, 0])
            y_min, y_max = np.min(states[:, 1]), np.max(states[:, 1])
            
            # Add padding (at least 20% of range, minimum 10 meters)
            x_range = max(x_max - x_min, 10.0)
            y_range = max(y_max - y_min, 10.0)
            padding_x = max(x_range * 0.2, 5.0)
            padding_y = max(y_range * 0.2, 5.0)
            
            ax.set_xlim(x_min - padding_x, x_max + padding_x)
            ax.set_ylim(y_min - padding_y, y_max + padding_y)
            
            # Add distance markers every 10 meters
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            
            # Draw coordinate grid reference
            grid_spacing = 10  # meters
            x_grid_start = int((x_min - padding_x) // grid_spacing) * grid_spacing
            x_grid_end = int((x_max + padding_x) // grid_spacing + 1) * grid_spacing
            y_grid_start = int((y_min - padding_y) // grid_spacing) * grid_spacing
            y_grid_end = int((y_max + padding_y) // grid_spacing + 1) * grid_spacing
            
            for x_grid in range(int(x_grid_start), int(x_grid_end) + grid_spacing, grid_spacing):
                ax.axvline(x=x_grid, color='gray', alpha=0.3, linestyle='--', linewidth=0.5)
            for y_grid in range(int(y_grid_start), int(y_grid_end) + grid_spacing, grid_spacing):
                ax.axhline(y=y_grid, color='gray', alpha=0.3, linestyle='--', linewidth=0.5)
                
            # Add status information box
            if len(states) > 0:
                current_speed = np.sqrt(states[-1, 6]**2 + states[-1, 7]**2)
                total_distance = 0
                if len(states) > 1:
                    # Calculate total distance traveled
                    for i in range(1, len(states)):
                        dx = states[i, 0] - states[i-1, 0]
                        dy = states[i, 1] - states[i-1, 1]
                        total_distance += np.sqrt(dx**2 + dy**2)
                
                info_text = f"Speed: {current_speed:.1f} m/s\n"
                info_text += f"Position: ({current_x:.1f}, {current_y:.1f}) m\n"
                info_text += f"Heading: {np.degrees(yaw):.1f}°\n"
                info_text += f"Distance: {total_distance:.1f} m"
                
                # Add warning if vehicle hasn't moved much
                if len(states) > 100 and total_distance < 5.0:
                    info_text += "\n⚠️ Low movement detected"
                
                # Add text box with vehicle info
                ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                       verticalalignment='top', horizontalalignment='left',
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                       fontsize=9, family='monospace')
                
        else:
            # No data yet, set default view
            ax.set_xlim(-10, 50)
            ax.set_ylim(-25, 25)
            ax.text(0.5, 0.5, 'Waiting for simulation data...', 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=12, alpha=0.5)
            
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title('Vehicle Trajectory (Top View)')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.legend(loc='upper right')
        
        # Plot 2: Velocity and control
        ax = self.axes[1]
        if len(time) > 0 and len(states) > 0:
            speeds = np.sqrt(states[:, 6]**2 + states[:, 7]**2)
            ax.plot(time, speeds, 'g-', linewidth=2, label='Speed (m/s)')
            ax.plot(time, states[:, 6], 'b--', label='Vx (m/s)')
            ax.plot(time, states[:, 7], 'r--', label='Vy (m/s)')
        ax.grid(True, alpha=0.3)
        ax.set_title('Vehicle Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        
        # Plot 3: Orientation
        ax = self.axes[2]
        if len(time) > 0 and len(states) > 0:
            ax.plot(time, np.degrees(states[:, 3]), 'r-', label='Roll (deg)')
            ax.plot(time, np.degrees(states[:, 4]), 'g-', label='Pitch (deg)')
            ax.plot(time, np.degrees(states[:, 5]), 'b-', label='Yaw (deg)')
        ax.grid(True, alpha=0.3)
        ax.set_title('Vehicle Orientation')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
        # Plot 4: Suspension compression
        ax = self.axes[3]
        if len(time) > 0 and len(wheel_history) > 0:
            wheel_names = ['FL', 'FR', 'RL', 'RR']
            colors = ['red', 'blue', 'green', 'orange']
            
            for i, (name, color) in enumerate(zip(wheel_names, colors)):
                try:
                    compressions = [wh[i]['compression'] for wh in wheel_history]
                    if len(compressions) == len(time):
                        ax.plot(time, compressions, color=color, label=f'{name} Suspension')
                except (IndexError, KeyError):
                    continue
                
        ax.grid(True, alpha=0.3)
        ax.set_title('Suspension Compression')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Compression (m)')
        ax.legend()
        
        # Plot 5: Motor torques with improved visualization
        ax = self.axes[4]
        if len(time) > 0 and len(control_history) > 0:
            try:
                front_torques = [ch['front_torque'] for ch in control_history]
                rear_torques = [ch['rear_torque'] for ch in control_history]
                
                if len(front_torques) == len(time) and len(rear_torques) == len(time):
                    ax.plot(time, front_torques, 'b-', linewidth=2, label='Front Motor (Actual)')
                    ax.plot(time, rear_torques, 'r-', linewidth=2, label='Rear Motor (Actual)')
                    
                    # If we have controller data, show commanded vs actual
                    if hasattr(self.simulator, 'controller') and self.simulator.controller:
                        controller = self.simulator.controller
                        if hasattr(controller, 'commanded_front_torque'):
                            # Show commanded torques as dashed lines
                            commanded_front = [ch.get('commanded_front_torque', ch['front_torque']) for ch in control_history]
                            commanded_rear = [ch.get('commanded_rear_torque', ch['rear_torque']) for ch in control_history]
                            
                            if len(commanded_front) == len(time):
                                ax.plot(time, commanded_front, 'b--', linewidth=1, alpha=0.7, label='Front Commanded')
                                ax.plot(time, commanded_rear, 'r--', linewidth=1, alpha=0.7, label='Rear Commanded')
                    
                    # Add torque limits for reference
                    max_torque = 300.0  # From vehicle params
                    ax.axhline(y=max_torque, color='gray', linestyle='--', alpha=0.5, label='Max Torque')
                    ax.axhline(y=-max_torque, color='gray', linestyle='--', alpha=0.5)
                    
                    # Show current values and efficiency
                    if len(front_torques) > 0:
                        current_front = front_torques[-1]
                        current_rear = rear_torques[-1]
                        total_power = abs(current_front + current_rear)
                        
                        info_text = f'Front: {current_front:.1f} Nm\n'
                        info_text += f'Rear: {current_rear:.1f} Nm\n'
                        info_text += f'Total Power: {total_power:.1f} Nm\n'
                        
                        if hasattr(self.simulator, 'controller') and self.simulator.controller:
                            split = self.simulator.controller.torque_split * 100
                            info_text += f'F/R Split: {split:.0f}/{100-split:.0f}%'
                        
                        ax.text(0.02, 0.98, info_text, 
                               transform=ax.transAxes, verticalalignment='top',
                               bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                               fontsize=9)
            except KeyError:
                pass
            
        ax.grid(True, alpha=0.3)
        ax.set_title('Motor Torques')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.legend()
        
        # Plot 6: Steering angles
        ax = self.axes[5]
        if len(time) > 0 and len(control_history) > 0:
            try:
                front_steering = [np.degrees(ch['front_steering']) for ch in control_history]
                rear_steering = [np.degrees(ch['rear_steering']) for ch in control_history]
                
                if len(front_steering) == len(time) and len(rear_steering) == len(time):
                    ax.plot(time, front_steering, 'b-', linewidth=2, label='Front Steering')
                    ax.plot(time, rear_steering, 'r-', linewidth=2, label='Rear Steering')
            except KeyError:
                pass
            
        ax.grid(True, alpha=0.3)
        ax.set_title('Steering Angles')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
        plt.tight_layout()

class VehicleSimulatorGUI:
    """GUI for advanced vehicle simulator"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Vehicle Dynamics Simulator")
        self.root.geometry("1600x1000")
        
        self.simulator = VehicleSimulator()
        self.visualizer = VehicleVisualizer(self.simulator)
        self.simulation_thread = None
        self.running = False
        
        self.create_widgets()
        self.setup_plots()
        
    def create_widgets(self):
        """Create GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel for controls
        control_frame = ttk.Frame(main_frame, width=400) #Change for panel width
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        control_frame.pack_propagate(False)
        
        # Right panel for plots
        self.plot_frame = ttk.Frame(main_frame)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.create_vehicle_controls(control_frame)
        self.create_motor_controls(control_frame)
        self.create_payload_controls(control_frame)
        self.create_scenario_controls(control_frame)
        self.create_action_buttons(control_frame)
        
    def create_vehicle_controls(self, parent):
        """Create vehicle parameter controls"""
        frame = ttk.LabelFrame(parent, text="Vehicle Parameters", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # Mass
        ttk.Label(frame, text="Mass (kg):").grid(row=0, column=0, sticky=tk.W)
        self.mass_var = tk.DoubleVar(value=1500.0)
        ttk.Entry(frame, textvariable=self.mass_var, width=10).grid(row=0, column=1)
        
        # Wheelbase
        ttk.Label(frame, text="Wheelbase (m):").grid(row=1, column=0, sticky=tk.W)
        self.wheelbase_var = tk.DoubleVar(value=2.7)
        ttk.Entry(frame, textvariable=self.wheelbase_var, width=10).grid(row=1, column=1)
        
        # Spring rate
        ttk.Label(frame, text="Spring Rate (N/m):").grid(row=2, column=0, sticky=tk.W)
        self.spring_var = tk.DoubleVar(value=25000.0)
        ttk.Entry(frame, textvariable=self.spring_var, width=10).grid(row=2, column=1)
        
        # Damping rate
        ttk.Label(frame, text="Damping (N⋅s/m):").grid(row=3, column=0, sticky=tk.W)
        self.damping_var = tk.DoubleVar(value=3000.0)
        ttk.Entry(frame, textvariable=self.damping_var, width=10).grid(row=3, column=1)
        
        frame.columnconfigure(1, weight=1)
        
    def create_motor_controls(self, parent):
        """Create motor parameter controls"""
        frame = ttk.LabelFrame(parent, text="Motor Configuration", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # Max motor torque
        ttk.Label(frame, text="Max Torque (Nm):").grid(row=0, column=0, sticky=tk.W)
        self.max_torque_var = tk.DoubleVar(value=300.0)
        ttk.Entry(frame, textvariable=self.max_torque_var, width=10).grid(row=0, column=1)
        
        # Motor efficiency
        ttk.Label(frame, text="Efficiency (%):").grid(row=1, column=0, sticky=tk.W)
        self.motor_efficiency_var = tk.DoubleVar(value=90.0)
        ttk.Entry(frame, textvariable=self.motor_efficiency_var, width=10).grid(row=1, column=1)
        
        # Gear ratio
        ttk.Label(frame, text="Gear Ratio:").grid(row=2, column=0, sticky=tk.W)
        self.gear_ratio_var = tk.DoubleVar(value=10.0)
        ttk.Entry(frame, textvariable=self.gear_ratio_var, width=10).grid(row=2, column=1)
        
        # Torque distribution
        ttk.Label(frame, text="Front/Rear Split (%):").grid(row=3, column=0, sticky=tk.W)
        self.torque_split_var = tk.DoubleVar(value=60.0)  # 60% front, 40% rear
        torque_scale = ttk.Scale(frame, from_=30, to=80, variable=self.torque_split_var, 
                                orient=tk.HORIZONTAL, length=120)
        torque_scale.grid(row=3, column=1, sticky=tk.EW)
        
        # Torque split label
        ttk.Label(frame, text="(30=Rear-biased, 80=Front-biased)").grid(row=4, column=0, columnspan=2, sticky=tk.W)
        
        # Motor response time
        ttk.Label(frame, text="Response Time (s):").grid(row=5, column=0, sticky=tk.W)
        self.motor_response_var = tk.DoubleVar(value=0.1)
        ttk.Entry(frame, textvariable=self.motor_response_var, width=10).grid(row=5, column=1)
        
        # Motor presets
        ttk.Label(frame, text="Motor Presets:").grid(row=6, column=0, sticky=tk.W, pady=(10,0))
        
        preset_frame = ttk.Frame(frame)
        preset_frame.grid(row=7, column=0, columnspan=2, sticky=tk.EW, pady=(5,0))
        
        # Default motor presets with descriptions
        self.motor_presets = {
            "Sports Car": {"torque": 250, "efficiency": 95, "gear": 8, "split": 40, "response": 0.05},
            "Electric SUV": {"torque": 400, "efficiency": 92, "gear": 12, "split": 60, "response": 0.08},
            "Race Car": {"torque": 180, "efficiency": 98, "gear": 6, "split": 30, "response": 0.02},
            "Truck": {"torque": 600, "efficiency": 88, "gear": 15, "split": 70, "response": 0.15},
            "RC Brushless": {"torque": 35, "efficiency": 97, "gear": 3, "split": 20, "response": 0.01}
        }
        
        # Preset descriptions for tooltips
        self.preset_descriptions = {
            "Sports Car": "Balanced performance with rear-wheel bias\nQuick response, moderate torque",
            "Electric SUV": "High torque, front-wheel drive character\nSmooth power delivery, efficiency focused",
            "Race Car": "Ultra-responsive, lightweight setup\nRear-wheel drive, maximum performance",
            "Truck": "Heavy-duty, high torque for loads\nFront-biased, gradual power delivery",
            "RC Brushless": "High-performance RC racing setup\nVery low torque, instant response, rear-drive"
        }
        
        # Preset selection dropdown
        ttk.Label(preset_frame, text="Select Preset:").grid(row=0, column=0, sticky=tk.W, pady=(0,5))
        self.preset_var = tk.StringVar(value="Sports Car")
        self.preset_combo = ttk.Combobox(preset_frame, textvariable=self.preset_var, 
                                        values=list(self.motor_presets.keys()), 
                                        state="readonly", width=12)
        self.preset_combo.grid(row=0, column=1, sticky=tk.EW, padx=(5,0), pady=(0,5))
        self.preset_combo.bind('<<ComboboxSelected>>', self.on_preset_selected)
        
        # Preset info display
        self.preset_info_var = tk.StringVar()
        self.preset_info_label = ttk.Label(preset_frame, textvariable=self.preset_info_var,
                                          font=('TkDefaultFont', 8), foreground='gray')
        self.preset_info_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(2,5))
        
        # Update info for initial selection
        self.update_preset_info()
        
        # Preset control buttons
        preset_btn_frame = ttk.Frame(preset_frame)
        preset_btn_frame.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(5,0))
        
        ttk.Button(preset_btn_frame, text="Load", width=5,
                  command=self.load_selected_preset).grid(row=0, column=0, padx=(0,1))
        ttk.Button(preset_btn_frame, text="Save", width=5,
                  command=self.save_current_preset).grid(row=0, column=1, padx=1)
        ttk.Button(preset_btn_frame, text="Delete", width=5,
                  command=self.delete_selected_preset).grid(row=0, column=2, padx=1)
        ttk.Button(preset_btn_frame, text="New", width=5,
                  command=self.create_new_preset).grid(row=0, column=3, padx=1)
        ttk.Button(preset_btn_frame, text="Help", width=5,
                  command=self.show_motor_help).grid(row=0, column=4, padx=(1,0))
        
        # Configure grid weights
        preset_btn_frame.columnconfigure(0, weight=1)
        preset_btn_frame.columnconfigure(1, weight=1)
        preset_btn_frame.columnconfigure(2, weight=1)
        preset_btn_frame.columnconfigure(3, weight=1)
        preset_btn_frame.columnconfigure(4, weight=1)
        
        preset_frame.columnconfigure(1, weight=1)
        
        frame.columnconfigure(1, weight=1)
        
    def create_vehicle_controls(self, parent):
        """Create vehicle parameter controls"""
        frame = ttk.LabelFrame(parent, text="Vehicle Parameters", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # Mass
        ttk.Label(frame, text="Mass (kg):").grid(row=0, column=0, sticky=tk.W)
        self.mass_var = tk.DoubleVar(value=1500.0)
        ttk.Entry(frame, textvariable=self.mass_var, width=10).grid(row=0, column=1)
        
        # Wheelbase
        ttk.Label(frame, text="Wheelbase (m):").grid(row=1, column=0, sticky=tk.W)
        self.wheelbase_var = tk.DoubleVar(value=2.7)
        ttk.Entry(frame, textvariable=self.wheelbase_var, width=10).grid(row=1, column=1)
        
        # Spring rate
        ttk.Label(frame, text="Spring Rate (N/m):").grid(row=2, column=0, sticky=tk.W)
        self.spring_var = tk.DoubleVar(value=25000.0)
        ttk.Entry(frame, textvariable=self.spring_var, width=10).grid(row=2, column=1)
        
        # Damping rate
        ttk.Label(frame, text="Damping (N⋅s/m):").grid(row=3, column=0, sticky=tk.W)
        self.damping_var = tk.DoubleVar(value=3000.0)
        ttk.Entry(frame, textvariable=self.damping_var, width=10).grid(row=3, column=1)
        
    def create_payload_controls(self, parent):
        """Create payload controls"""
        frame = ttk.LabelFrame(parent, text="Payload Configuration", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # Payload mass
        ttk.Label(frame, text="Payload Mass (kg):").grid(row=0, column=0, sticky=tk.W)
        self.payload_mass_var = tk.DoubleVar(value=0.0)
        ttk.Entry(frame, textvariable=self.payload_mass_var, width=10).grid(row=0, column=1)
        
        # Payload position
        ttk.Label(frame, text="Position X (m):").grid(row=1, column=0, sticky=tk.W)
        self.payload_x_var = tk.DoubleVar(value=1.0)
        ttk.Entry(frame, textvariable=self.payload_x_var, width=10).grid(row=1, column=1)
        
        ttk.Label(frame, text="Position Z (m):").grid(row=2, column=0, sticky=tk.W)
        self.payload_z_var = tk.DoubleVar(value=0.5)
        ttk.Entry(frame, textvariable=self.payload_z_var, width=10).grid(row=2, column=1)
        
    def create_scenario_controls(self, parent):
        """Create scenario selection"""
        frame = ttk.LabelFrame(parent, text="Test Scenarios", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        self.scenario_var = tk.StringVar(value="acceleration")
        scenarios = [
            ("Acceleration Test", "acceleration"),
            ("Cornering Test", "cornering"),
            ("Slalom Maneuver", "slalom"),
            ("Payload Test", "payload_test"),
            ("Suspension Test", "suspension_test")
        ]
        
        for i, (name, value) in enumerate(scenarios):
            ttk.Radiobutton(frame, text=name, variable=self.scenario_var, 
                          value=value).grid(row=i, column=0, sticky=tk.W)
            
        # Duration
        ttk.Label(frame, text="Duration (s):").grid(row=len(scenarios), column=0, sticky=tk.W)
        self.duration_var = tk.DoubleVar(value=10.0)
        ttk.Entry(frame, textvariable=self.duration_var, width=10).grid(row=len(scenarios), column=1)
        
    def create_action_buttons(self, parent):
        """Create action buttons"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=(10, 0))
        
        self.run_btn = ttk.Button(frame, text="Run Simulation", command=self.run_simulation)
        self.run_btn.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(frame, text="Stop", command=self.stop_simulation).pack(fill=tk.X, pady=(0, 5))
        ttk.Button(frame, text="Reset", command=self.reset_simulation).pack(fill=tk.X)
        
    def setup_plots(self):
        """Setup matplotlib plots"""
        self.fig, self.axes = self.visualizer.create_plots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    def update_vehicle_params(self):
        """Update vehicle parameters from GUI"""
        params = VehicleParams()
        params.mass = self.mass_var.get()
        params.wheelbase = self.wheelbase_var.get()
        params.spring_rate = self.spring_var.get()
        params.damping_rate = self.damping_var.get()
        
        # Motor parameters
        params.max_motor_torque = self.max_torque_var.get()
        params.motor_efficiency = self.motor_efficiency_var.get() / 100.0  # Convert percentage
        params.gear_ratio = self.gear_ratio_var.get()
        
        self.simulator.create_vehicle(params)
        
        # Set payload
        payload_mass = self.payload_mass_var.get()
        payload_pos = np.array([self.payload_x_var.get(), 0.0, self.payload_z_var.get()])
        self.simulator.vehicle.set_payload(payload_mass, payload_pos)
        
        # Update controller with motor settings
        if self.simulator.controller:
            self.simulator.controller.torque_split = self.torque_split_var.get() / 100.0
            self.simulator.controller.motor_response_time = self.motor_response_var.get()
        
        

            
    def load_motor_preset(self, preset_params):
        """Load motor preset configuration (legacy method for compatibility)"""
        self.max_torque_var.set(preset_params["torque"])
        self.motor_efficiency_var.set(preset_params["efficiency"])
        self.gear_ratio_var.set(preset_params["gear"])
        self.torque_split_var.set(preset_params["split"])
        self.motor_response_var.set(preset_params["response"])
        
    def load_selected_preset(self):
        """Load the currently selected preset"""
        preset_name = self.preset_var.get()
        if preset_name in self.motor_presets:
            self.load_motor_preset(self.motor_presets[preset_name])
            self.update_preset_info()
            
            # Show confirmation
            import tkinter.messagebox as messagebox
            messagebox.showinfo("Preset Loaded", f"Motor preset '{preset_name}' loaded successfully!")
            
    def save_current_preset(self):
        """Save current motor settings to selected preset or create new one"""
        import tkinter.simpledialog as simpledialog
        import tkinter.messagebox as messagebox
        
        current_preset = self.preset_var.get()
        
        # Ask if user wants to overwrite existing or create new
        if current_preset in self.motor_presets:
            response = messagebox.askyesnocancel(
                "Save Preset", 
                f"Overwrite existing preset '{current_preset}'?\n\n"
                f"Yes: Overwrite '{current_preset}'\n"
                f"No: Create new preset\n"
                f"Cancel: Don't save"
            )
            
            if response is None:  # Cancel
                return
            elif response:  # Yes - overwrite
                preset_name = current_preset
            else:  # No - create new
                preset_name = simpledialog.askstring(
                    "New Preset Name", 
                    "Enter name for new preset:",
                    initialvalue=f"{current_preset}_Modified"
                )
                if not preset_name:
                    return
        else:
            # Ask for name for new preset
            preset_name = simpledialog.askstring(
                "New Preset Name", 
                "Enter name for new preset:",
                initialvalue="Custom_Preset"
            )
            if not preset_name:
                return
        
        # Save current settings
        self.motor_presets[preset_name] = {
            "torque": self.max_torque_var.get(),
            "efficiency": self.motor_efficiency_var.get(),
            "gear": self.gear_ratio_var.get(),
            "split": self.torque_split_var.get(),
            "response": self.motor_response_var.get()
        }
        
        # Update description for custom preset
        params = self.motor_presets[preset_name]
        self.preset_descriptions[preset_name] = f"Custom preset: {params['torque']}Nm, {params['efficiency']}% eff, {params['gear']}:1 gear"
        
        # Update combobox and select new preset
        self.update_preset_combo()
        self.preset_var.set(preset_name)
        self.update_preset_info()
        
        messagebox.showinfo("Preset Saved", f"Motor preset '{preset_name}' saved successfully!")
        
    def delete_selected_preset(self):
        """Delete the currently selected preset"""
        import tkinter.messagebox as messagebox
        
        preset_name = self.preset_var.get()
        
        if preset_name not in self.motor_presets:
            messagebox.showwarning("Delete Preset", "No preset selected to delete.")
            return
            
        # Prevent deletion of default presets (optional - remove if you want to allow)
        default_presets = ["Sports Car", "Electric SUV", "Race Car", "Truck", "RC Brushless"]
        if preset_name in default_presets:
            response = messagebox.askyesno(
                "Delete Default Preset", 
                f"'{preset_name}' is a default preset.\n\n"
                f"Are you sure you want to delete it?\n"
                f"(It will be restored when you restart the application)"
            )
            if not response:
                return
        
        # Confirm deletion
        response = messagebox.askyesno(
            "Confirm Delete", 
            f"Are you sure you want to delete preset '{preset_name}'?\n\n"
            f"This action cannot be undone."
        )
        
        if response:
            del self.motor_presets[preset_name]
            # Also remove from descriptions if it exists
            if preset_name in self.preset_descriptions:
                del self.preset_descriptions[preset_name]
                
            self.update_preset_combo()
            
            # Select first available preset
            if self.motor_presets:
                first_preset = list(self.motor_presets.keys())[0]
                self.preset_var.set(first_preset)
                self.update_preset_info()
            
            messagebox.showinfo("Preset Deleted", f"Motor preset '{preset_name}' deleted successfully!")
            
    def create_new_preset(self):
        """Create a new preset with current settings"""
        import tkinter.simpledialog as simpledialog
        
        preset_name = simpledialog.askstring(
            "New Preset Name", 
            "Enter name for new preset:",
            initialvalue="Custom_Preset"
        )
        
        if preset_name:
            if preset_name in self.motor_presets:
                import tkinter.messagebox as messagebox
                response = messagebox.askyesno(
                    "Preset Exists", 
                    f"Preset '{preset_name}' already exists.\n\n"
                    f"Do you want to overwrite it?"
                )
                if not response:
                    return
            
            # Save current settings as new preset
            self.motor_presets[preset_name] = {
                "torque": self.max_torque_var.get(),
                "efficiency": self.motor_efficiency_var.get(),
                "gear": self.gear_ratio_var.get(),
                "split": self.torque_split_var.get(),
                "response": self.motor_response_var.get()
            }
            
            # Update description for new preset
            params = self.motor_presets[preset_name]
            self.preset_descriptions[preset_name] = f"Custom preset: {params['torque']}Nm, {params['efficiency']}% eff, {params['gear']}:1 gear"
            
            # Update combobox and select new preset
            self.update_preset_combo()
            self.preset_var.set(preset_name)
            self.update_preset_info()
            
            import tkinter.messagebox as messagebox
            messagebox.showinfo("Preset Created", f"New motor preset '{preset_name}' created successfully!")
            
    def show_motor_help(self):
        """Show help information about motor parameters"""
        import tkinter.messagebox as messagebox
        
        help_text = """Motor Parameter Guide:

Max Torque (Nm):
• Maximum torque output per motor
• Higher = faster acceleration
• Typical: 35-600 Nm

Efficiency (%):
• Motor efficiency (power out/power in)
• Electric motors: 90-98%
• Affects heat generation and range

Gear Ratio:
• Transmission reduction ratio
• Higher = more torque, lower top speed
• Lower = less torque, higher top speed
• Typical: 3-15:1

Front/Rear Split (%):
• Torque distribution between axles
• 30% = rear-wheel drive character
• 50% = balanced all-wheel drive
• 80% = front-wheel drive character

Response Time (s):
• Motor lag/response delay
• Racing: 0.01-0.05s (instant)
• Normal: 0.05-0.15s (quick)
• Heavy duty: 0.15-0.5s (gradual)

Preset Descriptions:
• Sports Car: Balanced performance
• Electric SUV: High torque, efficiency
• Race Car: Maximum performance
• Truck: Heavy-duty applications
• RC Brushless: High-performance RC racing"""

        messagebox.showinfo("Motor Parameters Help", help_text)
            
    def update_preset_combo(self):
        """Update the preset combobox with current presets"""
        self.preset_combo['values'] = list(self.motor_presets.keys())
        
    def on_preset_selected(self, event=None):
        """Handle preset selection change"""
        self.update_preset_info()
        
    def update_preset_info(self):
        """Update the preset information display"""
        preset_name = self.preset_var.get()
        if preset_name in self.preset_descriptions:
            self.preset_info_var.set(self.preset_descriptions[preset_name])
        elif preset_name in self.motor_presets:
            # For custom presets, show parameter summary
            params = self.motor_presets[preset_name]
            info = f"Custom: {params['torque']}Nm, {params['efficiency']}% eff, {params['gear']}:1 gear"
            self.preset_info_var.set(info)
        else:
            self.preset_info_var.set("")
        
    def get_preset_info(self, preset_name):
        """Get detailed information about a preset for display"""
        if preset_name not in self.motor_presets:
            return "Preset not found"
            
        params = self.motor_presets[preset_name]
        info = f"Torque: {params['torque']} Nm\n"
        info += f"Efficiency: {params['efficiency']}%\n"
        info += f"Gear Ratio: {params['gear']}:1\n"
        info += f"F/R Split: {params['split']}%\n"
        info += f"Response: {params['response']}s"
        
        return info
        
    def run_simulation(self):
        """Run simulation in separate thread"""
        if self.running:
            return
            
        self.running = True
        self.run_btn.config(state="disabled")
        
        self.simulation_thread = threading.Thread(target=self._simulation_worker)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
        
        # Start plot update timer
        self.update_plots()
        
    def _simulation_worker(self):
        """Simulation worker thread"""
        try:
            self.update_vehicle_params()
            scenario = self.scenario_var.get()
            duration = self.duration_var.get()
            
            self.simulator.run_scenario(scenario, duration)
            
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            self.running = False
            self.root.after(0, lambda: self.run_btn.config(state="normal"))
            
    def update_plots(self):
        """Update plots periodically"""
        if self.running:
            self.visualizer.update_plots()
            self.canvas.draw()
            self.root.after(100, self.update_plots)  # Update every 100ms
        else:
            # Final update
            self.visualizer.update_plots()
            self.canvas.draw()
            
    def stop_simulation(self):
        """Stop current simulation"""
        self.running = False
        
    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.simulator.reset_simulation()
            
        # Clear plots
        for ax in self.axes:
            ax.clear()
            ax.text(0.5, 0.5, 'Click "Run Simulation" to start', 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=12, alpha=0.5)
        self.canvas.draw()

def main():
    """Main function"""
    root = tk.Tk()
    app = VehicleSimulatorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
