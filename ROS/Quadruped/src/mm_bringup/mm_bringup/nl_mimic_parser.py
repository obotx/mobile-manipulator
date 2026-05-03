#!/usr/bin/env python3
"""
Non-linear mimic parser for URDF custom namespace.
Reads <nl:mimic> tags and applies formulas to joint states.
"""
import rclpy
import numpy as np
import numexpr as ne
import xml.etree.ElementTree as ET
import subprocess
import os
import re
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


class NLMimicParser(Node):
    # XML namespace for custom mimic tags
    NL_NS_URI = 'http://ros.org/nonlinear_mimic'
    
    def __init__(self):
        super().__init__('nl_mimic_parser')
        
        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        
        if not urdf_path or not os.path.isfile(urdf_path):
            self.get_logger().fatal(f'URDF path invalid or not provided: {urdf_path}')
            rclpy.shutdown()
            return
            
        self.get_logger().info(f'Loading mimic rules from: {urdf_path}')
        
        # ✅ Load and process URDF content (handles XACRO)
        urdf_content = self._load_urdf_content(urdf_path)
        self.mimic_rules = self._parse_nl_mimics_from_string(urdf_content)
        
        self.sub = self.create_subscription(
            JointState, '/joint_states_raw', self._callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self._joint_cache = {}
        
        self.get_logger().info(f'Loaded {len(self.mimic_rules)} non-linear mimic rules')
        for target, rule in self.mimic_rules.items():
            self.get_logger().info(f'  {target}: {rule["formula"]}')

    def _load_urdf_content(self, urdf_path: str) -> str:
        """Load URDF content, processing XACRO if needed."""
        try:
            if urdf_path.endswith('.xacro'):
                self.get_logger().debug(f'Processing XACRO: {urdf_path}')
                result = subprocess.run(
                    ['xacro', urdf_path],
                    capture_output=True, text=True, check=True, timeout=30
                )
                return result.stdout
            else:
                with open(urdf_path, 'r', encoding='utf-8') as f:
                    return f.read()
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'XACRO processing failed: {e.stderr}')
            raise
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {urdf_path}')
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            raise
    
    def _parse_nl_mimics_from_string(self, xml_content: str) -> dict:
        """Parse <nl:mimic> tags from XML string using robust Clark notation."""
        rules = {}
        
        try:
            root = ET.fromstring(xml_content)
        except ET.ParseError as e:
            self.get_logger().error(f'Failed to parse URDF XML: {e}')
            return rules
        
        # Extract namespace URI from <robot> tag using regex (more reliable)
        ns_match = re.search(r'xmlns:nl=["\']([^"\']+)["\']', xml_content)
        ns_uri = ns_match.group(1) if ns_match else self.NL_NS_URI
        self.get_logger().debug(f'Using namespace URI: {ns_uri}')
        
        # Clark notation: {namespace_uri}tag (100% reliable across Python versions)
        mimic_xpath = f'.//{{{ns_uri}}}mimic'
        
        for nl_mimic in root.findall(mimic_xpath):
            target = nl_mimic.get('target')
            if not target:
                self.get_logger().warn('nl:mimic missing "target" attribute, skipping')
                continue
            
            # Use Clark notation for child lookups
            formula_el = nl_mimic.find(f'{{{ns_uri}}}formula')
            formula = formula_el.text.strip() if formula_el is not None and formula_el.text else ''
            if not formula:
                self.get_logger().warn(f'nl:mimic for "{target}" missing <nl:formula>, skipping')
                continue
            
            # Parse input mappings: alias -> joint_name
            inputs = {}
            inputs_el = nl_mimic.find(f'{{{ns_uri}}}inputs')
            if inputs_el is not None:
                for inp in inputs_el.findall(f'{{{ns_uri}}}input'):
                    alias = inp.get('name')
                    joint = inp.get('joint')
                    if alias and joint:
                        inputs[alias] = joint
            
            # Parse output limits
            limits_el = nl_mimic.find(f'{{{ns_uri}}}limits')
            limits = None
            if limits_el is not None:
                try:
                    limits = {
                        'min': float(limits_el.get('min', '-inf')),
                        'max': float(limits_el.get('max', 'inf'))
                    }
                except (ValueError, TypeError):
                    self.get_logger().warn(f'Invalid limits for {target}, ignoring')
            
            # Parse safety clamps for inputs
            safety = {}
            safety_el = nl_mimic.find(f'{{{ns_uri}}}safety')
            if safety_el is not None:
                for clamp in safety_el.findall(f'{{{ns_uri}}}clamp'):
                    joint = clamp.get('joint')
                    if joint:
                        try:
                            safety[joint] = {
                                'min': float(clamp.get('min', '-inf')),
                                'max': float(clamp.get('max', 'inf'))
                            }
                        except (ValueError, TypeError):
                            self.get_logger().warn(f'Invalid clamp values for {joint}, ignoring')
            
            rules[target] = {
                'formula': formula,
                'inputs': inputs,
                'limits': limits,
                'safety': safety
            }
            self.get_logger().info(f'Parsed mimic: {target} = {formula}')
        
        return rules

    def _callback(self, msg: JointState):
        """Process incoming joint states and apply non-linear mimics ONLY to target joints."""
        # Update cache with latest values
        for name, pos in zip(msg.name, msg.position):
            self._joint_cache[name] = pos
        
        # Build output message - start with a copy of incoming
        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.name = list(msg.name)
        out_msg.position = list(msg.position)
        out_msg.velocity = list(msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * len(msg.name)
        out_msg.effort = list(msg.effort) if len(msg.effort) == len(msg.name) else [0.0] * len(msg.name)
        
        # Process ONLY joints that have non-linear mimic rules
        for target_joint, rule in self.mimic_rules.items():
            # Gather input values for formula evaluation
            eval_vars = {}
            inputs_ready = True
            
            for alias, joint_name in rule['inputs'].items():
                if joint_name not in self._joint_cache:
                    inputs_ready = False
                    break
                
                val = self._joint_cache[joint_name]
                
                # Apply safety clamps if defined
                if joint_name in rule['safety']:
                    clamp = rule['safety'][joint_name]
                    val = np.clip(val, clamp['min'], clamp['max'])
                
                eval_vars[alias] = val
            
            if not inputs_ready:
                self.get_logger().debug(f'Inputs not ready for {target_joint}, skipping mimic')
                continue
            
            # Evaluate formula using numexpr
            try:
                # numexpr works with arrays, so wrap scalars
                result = ne.evaluate(rule['formula'], local_dict=eval_vars)
                # Convert to scalar float (handles both numpy scalars and plain floats)
                if hasattr(result, 'item'):
                    result = float(result.item())
                else:
                    result = float(result)
            except Exception as e:
                self.get_logger().error(
                    f'Failed to evaluate formula "{rule["formula"]}" for {target_joint}: {e}')
                continue
            
            # Apply output limits if defined
            if rule['limits'] is not None:
                result = np.clip(result, rule['limits']['min'], rule['limits']['max'])
            
            # ✅ ONLY assign new position for the target joint
            if target_joint in out_msg.name:
                idx = out_msg.name.index(target_joint)
                out_msg.position[idx] = result
                out_msg.velocity[idx] = 0.0
                out_msg.effort[idx] = 0.0
            else:
                # Target joint not in incoming message - add it
                out_msg.name.append(target_joint)
                out_msg.position.append(result)
                out_msg.velocity.append(0.0)
                out_msg.effort.append(0.0)
            
            self.get_logger().debug(f'{target_joint} = {result:.4f} (formula: {rule["formula"]})')
        
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NLMimicParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()