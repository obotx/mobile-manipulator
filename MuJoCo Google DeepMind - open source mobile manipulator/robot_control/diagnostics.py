import mujoco
import numpy as np
import sys

def diagnose_robot_model(model_file: str):
    """Diagnose robot model to identify actuators and joints"""
    
    try:
        model = mujoco.MjModel.from_xml_path(model_file)
        data = mujoco.MjData(model)
        
        print("=" * 60)
        print(f"ROBOT MODEL DIAGNOSTICS")
        print("=" * 60)
        print(f"Model file: {model_file}")
        print(f"Number of actuators: {model.nu}")
        print(f"Number of joints: {model.nq}")
        print(f"Number of bodies: {model.nbody}")
        print()
        
        # List all actuators
        print("ACTUATORS:")
        print("-" * 40)
        for i in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            actuator_type = model.actuator_dyntype[i]
            trntype = model.actuator_trntype[i]
            
            # Get associated joint/tendon
            if trntype == mujoco.mjtTrn.mjTRN_JOINT:
                joint_id = model.actuator_trnid[i, 0]
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                target = f"joint: {joint_name}"
            elif trntype == mujoco.mjtTrn.mjTRN_TENDON:
                tendon_id = model.actuator_trnid[i, 0]
                tendon_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id)
                target = f"tendon: {tendon_name}"
            else:
                target = "unknown"
            
            print(f"  [{i:2d}] {actuator_name or 'unnamed'} -> {target}")
        
        print()
        
        # List all joints
        print("JOINTS:")
        print("-" * 40)
        for i in range(model.nq):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_type = model.jnt_type[i]
            
            # Joint type names
            joint_types = {
                0: "free",
                1: "ball",
                2: "slide",
                3: "hinge"
            }
            
            type_name = joint_types.get(joint_type, "unknown")
            print(f"  [{i:2d}] {joint_name or 'unnamed'} (type: {type_name})")
        
        print()
        
        # Suggest wheel actuator mapping
        print("SUGGESTED WHEEL ACTUATOR MAPPING:")
        print("-" * 40)
        wheel_keywords = ['wheel', 'FL', 'FR', 'BL', 'BR', 'front', 'back', 'left', 'right']
        potential_wheels = []
        
        for i in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if actuator_name:
                for keyword in wheel_keywords:
                    if keyword.lower() in actuator_name.lower():
                        potential_wheels.append((i, actuator_name))
                        break
        
        if potential_wheels:
            print("Potential wheel actuators found:")
            for i, name in potential_wheels:
                print(f"  [{i}] {name}")
        else:
            print("No obvious wheel actuators found. First 4 actuators:")
            for i in range(min(4, model.nu)):
                actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                print(f"  [{i}] {actuator_name or 'unnamed'}")
        
        print()
        
        # Suggest gripper actuator mapping
        print("SUGGESTED GRIPPER ACTUATOR MAPPING:")
        print("-" * 40)
        gripper_keywords = ['finger', 'gripper', 'palm', 'grasp']
        potential_grippers = []
        
        for i in range(model.nu):
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if actuator_name:
                for keyword in gripper_keywords:
                    if keyword.lower() in actuator_name.lower():
                        potential_grippers.append((i, actuator_name))
                        break
        
        if potential_grippers:
            print("Potential gripper actuators found:")
            for i, name in potential_grippers:
                print(f"  [{i}] {name}")
        else:
            print("No obvious gripper actuators found.")
        
        print()
        
        # Configuration suggestions
        print("CONFIGURATION SUGGESTIONS:")
        print("-" * 40)
        
        if potential_wheels:
            print("For mobile_base.py, try updating wheel_actuator_ids:")
            print("self.wheel_actuator_ids = {")
            for i, name in potential_wheels[:4]:  # Take first 4
                # Try to guess wheel position from name
                if 'FL' in name or ('front' in name.lower() and 'left' in name.lower()):
                    pos = 'FL'
                elif 'FR' in name or ('front' in name.lower() and 'right' in name.lower()):
                    pos = 'FR'
                elif 'BL' in name or ('back' in name.lower() and 'left' in name.lower()):
                    pos = 'BL'
                elif 'BR' in name or ('back' in name.lower() and 'right' in name.lower()):
                    pos = 'BR'
                else:
                    pos = f"WHEEL_{i}"
                print(f"    '{pos}': {i},  # {name}")
            print("}")
        
        if potential_grippers:
            print("\nFor gripper config, update joint names in open_positions and closed_positions:")
            for i, name in potential_grippers:
                # Get joint name if it's a joint actuator
                if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                    joint_id = model.actuator_trnid[i, 0]
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                    print(f"    '{joint_name}': 0.0,  # Open position")
        
        print("\n" + "=" * 60)
        print("Run this script to understand your robot's actuator setup!")
        print("=" * 60)
        
    except Exception as e:
        print(f"Error loading model: {e}")
        return False
    
    return True

def test_actuator_control(model_file: str, actuator_id: int, force: float = 10.0):
    """Test a specific actuator"""
    try:
        model = mujoco.MjModel.from_xml_path(model_file)
        data = mujoco.MjData(model)
        
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        print(f"Testing actuator [{actuator_id}] {actuator_name} with force {force}")
        
        # Apply force for a few steps
        for i in range(100):
            data.ctrl[actuator_id] = force
            mujoco.mj_step(model, data)
            
            if i % 20 == 0:
                print(f"Step {i}: ctrl = {data.ctrl[actuator_id]}")
        
        print("Test complete")
        
    except Exception as e:
        print(f"Error testing actuator: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python diagnostics.py <model_file.xml> [test_actuator_id] [force]")
        print("Example: python diagnostics.py robot.xml")
        print("Example: python diagnostics.py robot.xml 0 10.0")
        sys.exit(1)
    
    model_file = sys.argv[1]
    
    if len(sys.argv) >= 3:
        # Test mode
        actuator_id = int(sys.argv[2])
        force = float(sys.argv[3]) if len(sys.argv) >= 4 else 10.0
        test_actuator_control(model_file, actuator_id, force)
    else:
        # Diagnostic mode
        diagnose_robot_model(model_file)