#!/usr/bin/env python3
"""
Gimbal control using PWM actuators directly
Bypasses PX4 gimbal system and directly controls AUX outputs
"""

import asyncio
import time
from mavsdk import System


def angle_to_pwm(angle_deg, min_angle=-90, max_angle=90):
    """
    Convert angle in degrees to PWM value (-1 to 1)
    
    Args:
        angle_deg: Desired angle in degrees
        min_angle: Minimum angle capability of gimbal axis
        max_angle: Maximum angle capability of gimbal axis
    
    Returns:
        PWM value from -1 to 1
    """
    # Clamp angle to valid range
    angle_deg = max(min_angle, min(max_angle, angle_deg))
    
    # Normalize to -1 to 1 range
    # 0 degrees = 0.0 PWM (center)
    # max_angle = 1.0 PWM
    # min_angle = -1.0 PWM
    pwm_value = angle_deg / max(abs(min_angle), abs(max_angle))
    
    return pwm_value


async def run():
    drone = System()
    
    print("Connecting to flight controller...")
    await drone.connect(system_address="serial:///dev/drone:115200")
    
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected!")
            break
    
    await asyncio.sleep(1)
    
    # Actuator indices (starting from 1)
    YAW_ACTUATOR = 2   # AUX 2
    PITCH_ACTUATOR = 3  # AUX 3
    
    print("\n" + "="*60)
    print("GIMBAL PWM CONTROL TEST")
    print("="*60)
    print(f"Yaw control: AUX {YAW_ACTUATOR}")
    print(f"Pitch control: AUX {PITCH_ACTUATOR}")
    print("="*60)
    
    # Arm the drone to enable PWM outputs
    # print("\nArming vehicle to enable actuator outputs...")
    # try:
    #     await drone.action.arm()
    #     print("✓ Vehicle armed")
    #     await asyncio.sleep(1)
    # except Exception as e:
    #     print(f"✗ Failed to arm: {e}")
    #     print("Continuing anyway (outputs might not work)...")
    
    print("\nTesting WITHOUT arming (to see if it's necessary)...")
    print("If gimbal doesn't move, we know arming is required.")
    
    # Test pitch movements
    print("\n" + "="*50)
    print("TESTING PITCH (TILT)")
    print("="*50)
    
    pitch_tests = [
        (0, "Level (0°)"),
        (-30, "Down 30°"),
        (-60, "Down 60°"),
        (-90, "Down 90° (max)"),
        (0, "Back to level"),
        (30, "Up 30°"),
        (0, "Back to level"),
    ]
    
    for angle, desc in pitch_tests:
        print(f"\n→ {desc}")
        try:
            pwm_value = angle_to_pwm(angle, min_angle=-90, max_angle=90)
            print(f"  Sending PWM value: {pwm_value:.3f}")
            
            await drone.action.set_actuator(PITCH_ACTUATOR, pwm_value)
            print("  ✓ Command sent")
            await asyncio.sleep(3)  # Wait for gimbal to move
            
        except Exception as e:
            print(f"  ✗ Failed: {e}")
    
    # Test yaw movements  
    print("\n" + "="*50)
    print("TESTING YAW (PAN)")
    print("="*50)
    
    yaw_tests = [
        (0, "Center (0°)"),
        (45, "Right 45°"),
        (90, "Right 90° (max)"),
        (0, "Back to center"),
        (-45, "Left 45°"),
        (-90, "Left 90° (max)"),
        (0, "Back to center"),
    ]
    
    for angle, desc in yaw_tests:
        print(f"\n→ {desc}")
        try:
            pwm_value = angle_to_pwm(angle, min_angle=-180, max_angle=180)
            print(f"  Sending PWM value: {pwm_value:.3f}")
            
            await drone.action.set_actuator(YAW_ACTUATOR, pwm_value)
            print("  ✓ Command sent")
            await asyncio.sleep(3)
            
        except Exception as e:
            print(f"  ✗ Failed: {e}")
    
    # Test combined movement
    print("\n" + "="*50)
    print("TESTING COMBINED MOVEMENTS")
    print("="*50)
    
    combined_tests = [
        (0, 0, "Center position"),
        (-45, 45, "Down 45°, Right 45°"),
        (-60, -60, "Down 60°, Left 60°"),
        (0, 90, "Level, Right 90°"),
        (0, 0, "Back to center"),
    ]
    
    for pitch, yaw, desc in combined_tests:
        print(f"\n→ {desc}")
        try:
            pitch_pwm = angle_to_pwm(pitch, min_angle=-90, max_angle=90)
            yaw_pwm = angle_to_pwm(yaw, min_angle=-180, max_angle=180)
            
            print(f"  Pitch PWM: {pitch_pwm:.3f}, Yaw PWM: {yaw_pwm:.3f}")
            
            # Set both axes simultaneously
            await drone.action.set_actuator(PITCH_ACTUATOR, pitch_pwm)
            await drone.action.set_actuator(YAW_ACTUATOR, yaw_pwm)
            
            print("  ✓ Commands sent")
            await asyncio.sleep(4)
            
        except Exception as e:
            print(f"  ✗ Failed: {e}")
    
    # Return to neutral
    print("\n" + "="*50)
    print("RETURNING TO NEUTRAL")
    print("="*50)
    try:
        await drone.action.set_actuator(PITCH_ACTUATOR, 0.0)
        await drone.action.set_actuator(YAW_ACTUATOR, 0.0)
        print("✓ Returned to neutral (0.0, 0.0)")
        await asyncio.sleep(2)
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Disarm the vehicle
    # print("\nDisarming vehicle...")
    # try:
    #     await drone.action.disarm()
    #     print("✓ Vehicle disarmed")
    # except Exception as e:
    #     print(f"✗ Failed to disarm: {e}")
    
    print("\n(Skipped disarm since we didn't arm)")
    
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("\nDid the gimbal move physically?")
    print("  ✓ YES → Great! PWM control is working")
    print("  ✗ NO  → Check:")
    print("         1. Actuator assignments in QGC")
    print("         2. Gimbal power")
    print("         3. PWM cable connections")


if __name__ == "__main__":
    print("\n⚠️  SAFETY NOTICE:")
    print("- Ensure gimbal has clear movement range")
    print("- Verify PWM cables connected to AUX 2 (yaw) and AUX 3 (pitch)")
    print("- Press Ctrl+C to stop anytime\n")
    print("- Vehicle will be ARMED during test (actuators enabled)")
    print("- Vehicle will be DISARMED at end of test\n")
    
    time.sleep(3)
    
    drone = None
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        print("Attempting emergency disarm...")
        
        # Emergency disarm if Ctrl+C pressed
        async def emergency_disarm():
            temp_drone = System()
            await temp_drone.connect(system_address="serial:///dev/drone:115200")
            await asyncio.sleep(1)
            try:
                await temp_drone.action.disarm()
                print("✓ Emergency disarm successful")
            except:
                print("✗ Could not disarm - please disarm manually via QGC")
        
        asyncio.run(emergency_disarm())
        
    except Exception as e:
        print(f"\n\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()