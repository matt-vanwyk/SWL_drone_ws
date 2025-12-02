#!/usr/bin/env python3
"""
Gimbal controller with position tracking and relative movement support
Prevents wrapping issues when panning continuously in one direction
"""

import asyncio
from mavsdk import System


def angle_to_pwm(angle_deg, min_angle=-90, max_angle=90):
    """
    Convert angle in degrees to PWM value (-1 to 1) with proper linear mapping
    
    Args:
        angle_deg: Desired angle in degrees
        min_angle: Minimum angle capability of gimbal axis
        max_angle: Maximum angle capability of gimbal axis
    
    Returns:
        PWM value from -1 to 1
    """
    # Clamp angle to valid range
    angle_deg = max(min_angle, min(max_angle, angle_deg))
    
    # Linear mapping from [min_angle, max_angle] to [-1, 1]
    angle_range = max_angle - min_angle
    
    if angle_range == 0:
        return 0.0
    
    # Map angle to PWM range
    pwm_value = 2.0 * (angle_deg - min_angle) / angle_range - 1.0
    
    # Clamp to [-1, 1] range (safety)
    pwm_value = max(-1.0, min(1.0, pwm_value))
    
    return pwm_value


class GimbalController:
    """Gimbal controller with position tracking"""
    
    # Actuator indices for AUX outputs
    YAW_ACTUATOR = 2    # AUX 2 - Yaw (pan left/right)
    PITCH_ACTUATOR = 3  # AUX 3 - Pitch (tilt up/down)
    
    # Gimbal angle limits based on PWM configuration
    YAW_MIN = -60   # Degrees (left)
    YAW_MAX = 65    # Degrees (right)
    PITCH_MIN = -30  # Degrees (down)
    PITCH_MAX = 30   # Degrees (up)
    
    def __init__(self, drone):
        self.drone = drone
        
        # Track current gimbal position
        self._current_yaw = 0.0
        self._current_pitch = 0.0
    
    @property
    def current_yaw(self):
        """Get current yaw angle"""
        return self._current_yaw
    
    @property
    def current_pitch(self):
        """Get current pitch angle"""
        return self._current_pitch
    
    async def set_pitch(self, angle_deg):
        """
        Set gimbal pitch to absolute angle
        
        Args:
            angle_deg: Pitch angle in degrees (negative = down, positive = up)
                      Range: -30° to +30°
        """
        # Clamp to limits
        angle_deg = max(self.PITCH_MIN, min(self.PITCH_MAX, angle_deg))
        
        # Update tracked position
        self._current_pitch = angle_deg
        
        pwm = angle_to_pwm(angle_deg, self.PITCH_MIN, self.PITCH_MAX)
        await self.drone.action.set_actuator(self.PITCH_ACTUATOR, pwm)
    
    async def set_yaw(self, angle_deg):
        """
        Set gimbal yaw to absolute angle
        
        Args:
            angle_deg: Yaw angle in degrees (negative = left, positive = right)
                      Range: -60° to +65°
        """
        # Clamp to limits
        angle_deg = max(self.YAW_MIN, min(self.YAW_MAX, angle_deg))
        
        # Update tracked position
        self._current_yaw = angle_deg
        
        pwm = angle_to_pwm(angle_deg, self.YAW_MIN, self.YAW_MAX)
        await self.drone.action.set_actuator(self.YAW_ACTUATOR, pwm)
    
    async def pan_relative(self, delta_yaw_deg):
        """
        Pan gimbal by a relative amount from current position
        
        Args:
            delta_yaw_deg: Amount to pan (positive = right, negative = left)
        
        Returns:
            tuple: (success: bool, new_angle: float, clamped: bool)
        """
        # Calculate new absolute angle
        new_yaw = self._current_yaw + delta_yaw_deg
        
        # Check if we hit limits
        clamped = False
        if new_yaw > self.YAW_MAX:
            new_yaw = self.YAW_MAX
            clamped = True
        elif new_yaw < self.YAW_MIN:
            new_yaw = self.YAW_MIN
            clamped = True
        
        # Set to new position
        await self.set_yaw(new_yaw)
        
        return (True, new_yaw, clamped)
    
    async def tilt_relative(self, delta_pitch_deg):
        """
        Tilt gimbal by a relative amount from current position
        
        Args:
            delta_pitch_deg: Amount to tilt (positive = up, negative = down)
        
        Returns:
            tuple: (success: bool, new_angle: float, clamped: bool)
        """
        # Calculate new absolute angle
        new_pitch = self._current_pitch + delta_pitch_deg
        
        # Check if we hit limits
        clamped = False
        if new_pitch > self.PITCH_MAX:
            new_pitch = self.PITCH_MAX
            clamped = True
        elif new_pitch < self.PITCH_MIN:
            new_pitch = self.PITCH_MIN
            clamped = True
        
        # Set to new position
        await self.set_pitch(new_pitch)
        
        return (True, new_pitch, clamped)
    
    async def set_pitch_and_yaw(self, pitch_deg, yaw_deg):
        """
        Set both gimbal pitch and yaw to absolute angles simultaneously
        
        Args:
            pitch_deg: Pitch angle in degrees (-30° to +30°)
            yaw_deg: Yaw angle in degrees (-60° to +65°)
        """
        await self.set_pitch(pitch_deg)
        await self.set_yaw(yaw_deg)
    
    async def center(self):
        """Return gimbal to center position (0, 0)"""
        await self.set_pitch_and_yaw(0, 0)
    
    def get_position(self):
        """
        Get current gimbal position
        
        Returns:
            tuple: (pitch, yaw) in degrees
        """
        return (self._current_pitch, self._current_yaw)
    
    def get_remaining_range(self):
        """
        Get how much range is left in each direction
        
        Returns:
            dict with remaining range in each direction
        """
        return {
            'yaw_left_remaining': self._current_yaw - self.YAW_MIN,
            'yaw_right_remaining': self.YAW_MAX - self._current_yaw,
            'pitch_down_remaining': self._current_pitch - self.PITCH_MIN,
            'pitch_up_remaining': self.PITCH_MAX - self._current_pitch,
        }


async def demo():
    """Demo showing position tracking and relative movements"""
    
    drone = System()
    
    print("Connecting to flight controller...")
    await drone.connect(system_address="serial:///dev/drone:115200")
    
    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected!\n")
            break
    
    # Create gimbal controller
    gimbal = GimbalController(drone)
    
    print("=" * 60)
    print("GIMBAL POSITION TRACKING DEMO")
    print("=" * 60)
    
    # Home gimbal
    print("\n→ Homing gimbal...")
    await gimbal.center()
    await asyncio.sleep(1)
    print(f"   Position: {gimbal.get_position()}")
    
    # Relative movements
    print("\n→ Pan right 20° (relative)...")
    success, new_angle, clamped = await gimbal.pan_relative(20)
    print(f"   New yaw: {new_angle}°, Clamped: {clamped}")
    await asyncio.sleep(2)
    
    print("\n→ Pan right 30° more (relative)...")
    success, new_angle, clamped = await gimbal.pan_relative(30)
    print(f"   New yaw: {new_angle}°, Clamped: {clamped}")
    await asyncio.sleep(2)
    
    print("\n→ Pan right 50° more (will hit limit!)...")
    success, new_angle, clamped = await gimbal.pan_relative(50)
    print(f"   New yaw: {new_angle}°, Clamped: {clamped}")
    if clamped:
        print("   ⚠ Hit right limit!")
    await asyncio.sleep(2)
    
    # Check remaining range
    remaining = gimbal.get_remaining_range()
    print(f"\n→ Remaining range:")
    print(f"   Left: {remaining['yaw_left_remaining']:.1f}°")
    print(f"   Right: {remaining['yaw_right_remaining']:.1f}°")
    
    # Return to center
    print("\n→ Returning to center...")
    await gimbal.center()
    await asyncio.sleep(1)
    print(f"   Position: {gimbal.get_position()}")
    
    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        asyncio.run(demo())
    except KeyboardInterrupt:
        print("\n\nDemo stopped by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()