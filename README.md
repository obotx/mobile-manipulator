# Universal Mobile Manipulator

https://github.com/user-attachments/assets/ace3e387-373c-48c0-a6cd-64db6422f6d3

## Table Of Contents

- [Introduction](#introduction)
- [Mobile Manipulator Type](#mobile-manipulator-type)
  1. Quadruped Mobile Manipulator
      - MuJoCo
        - Morph - I : Dual Independent Parallel Manipulators
        - Morph - II : Single Closed-Chain Parallel Manipulator
      - ROS
        - Morph - I : Dual Independent Parallel Manipulators
        - Morph - I : MoveIt Motion Planning
        - Morph - I : MoveIt Pick and Place
  2. Tripod Mobile Manipulator
      - MuJoCo
      - ROS
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

## Introduction
This repository contains the implementation of the ObotX Universal Mobile Manipulator, featuring two distinct simulation environments:

- ROS 2 (Jazzy) with Gazebo Harmonic
- MuJoCo

The project focuses on mobile manipulation systems integrating locomotion and robotic arms for complex tasks.

## Mobile Manipulator Type
1. Quadruped Mobile Manipulator
    - [MuJoco](./MUJOCO/Quadruped) (🚀 **Click to explore more**)
      - **Morph - I** : Dual Independent Parallel Manipulators
      <img width="608" height="452" alt="basic_glfw_morph_i_trajectory" src="https://github.com/user-attachments/assets/aeaa1eab-2f9d-456a-9728-123f56364058" />

      - **Morph - II**  : Single Closed-Chain Parallel Manipulator
      <img width="608" height="452" alt="basic_glfw_morph_ii_free_move" src="https://github.com/user-attachments/assets/ef014d18-7159-4b32-a7e8-b8f082fec64b" />

    - [ROS](./ROS/Quadruped) (🚀 **Click to explore more**)
      - **Morph - I** : Dual Independent Parallel Manipulators
        
        https://github.com/user-attachments/assets/9d1917a3-4d70-42b7-870c-debb7992be44
        
      - **Morph - I** : MoveIt Motion Planning
        
        https://github.com/user-attachments/assets/06b96eab-91f4-4531-ab77-8572dfe3c3a9

      - **Morph - I** : MoveIt Pick and Place
        
        https://github.com/user-attachments/assets/e6f80654-5056-4998-a8c1-618290b149bd

2. Tripod Mobile Manipulator
    - [MuJoco](./MUJOCO/Tripod) (🚀 **Click to explore more**)
      
      https://github.com/user-attachments/assets/5699a624-a83a-4c6a-859a-ceb459501354
      
    - [ROS](./ROS/Tripod) (***Coming Soon***)

## Acknowledgements

- **Kitchen assets** from [furniture_sim by vikashplus](https://github.com/vikashplus/furniture_sim)  
- **Market product assets** from [Scanned Objects MuJoCo Models by kevinzakka](https://github.com/kevinzakka/mujoco_scanned_objects)   
- **Mecanum wheel mobile base implementation** from [Mecanum Drive in MuJoCo by JunHeonYoon](https://github.com/JunHeonYoon/mujoco_mecanum)
- **Reference for base platform Summit XL ROS package** from [summit_xl_common by RobotnikAutomation](https://github.com/RobotnikAutomation/summit_xl_common)
- **Gripper for 4 fingers** from [DELTO_M_ROS2 by tesollodelto](https://github.com/tesollodelto/delto_m_ros2/tree/jazzy-dev)
- **Robotiq 3F Urdf Reference** from [Robotiq](https://github.com/ros-industrial-attic/robotiq)

## Contact

Project Homepage: [obotx.com](https://obotx.com/)
