# Mobile Manipulator

## Table of Contents
- [MORPH Series Overview](#morph-series--mobile-omni-robotic-platform-with-hands)
- [MORPH-I (Type I)](#morph-i-type-i)
  - [Joint Configuration](#joint-configuration)
  - [End-effectors](#end-effectors)
  - [Control Characteristics](#control-characteristics)
- [MORPH-II (Type II)](#morph-ii-type-ii)
  - [Joint Configuration](#joint-configuration-1)
  - [End-effector](#end-effector)
  - [Control Characteristics](#control-characteristics-1)
- [Mobile Base](#mobile-base)
- [Control Interface](#control-interface)
- [How to Use](#how-to-use)
  - [Installation](#installation)
  - [MORPH-I Examples](#morph-i-examples)
  - [MORPH-II Examples](#morph-ii-examples)

---

## MORPH Series – `Mobile Omni Robotic Platform with Hands`

The **MORPH** series implements two distinct parallel manipulator configurations mounted on an omnidirectional mobile base. Both variants use chained mechanisms but differ fundamentally in their kinematic architecture.

---

## MORPH-I (Type I)  
*Dual Independent Parallel Manipulators*
- Two completely separate manipulators mounted side-by-side on the mobile base
- Each manipulator consists of:
  - Two vertical prismatic columns (left and right)
  - One horizontal bar connecting the tops of both columns
  - A base revolute joint rotating the entire column assembly
- The manipulators operate independently with no mechanical coupling between left and right sides

### Joint Configuration
| Joint Name                     | Manipulator | Type        | Function |
|--------------------------------|-------------|-------------|----------|
| ColumnLeftBearingJoint_1       | Left        | Prismatic   | Height of left vertical column |
| ColumnRightBearingJoint_1      | Left        | Prismatic   | Height of right vertical column |
| ArmLeftJoint_1                 | Left        | Prismatic   | Extension of horizontal bar |
| BaseJoint_1                    | Left        | Revolute    | Yaw rotation of entire left assembly |
| ColumnLeftBearingJoint_2       | Right       | Prismatic   | Height of left vertical column |
| ColumnRightBearingJoint_2      | Right       | Prismatic   | Height of right vertical column |
| ArmLeftJoint_2                 | Right       | Prismatic   | Extension of horizontal bar |
| BaseJoint_2                    | Right       | Revolute    | Yaw rotation of entire right assembly |

### End-effectors
- Two independent grippers:
  - Left manipulator: 3-fingers gripper
  - Right manipulator: 3-fingers gripper

### Control Characteristics
- Eight-dimensional command vector required
- Each manipulator has independent vertical motion, horizontal extension, and rotation
- End-effector positions are controlled separately
- No shared actuators between left and right manipulators

---

## MORPH-II (Type II)  
*Single Closed-Chain Parallel Manipulator*

- One integrated manipulator mounted on the mobile base
- Two vertical prismatic columns (left and right) that move independently
- Left horizontal bar: attached to the left vertical column. Can slide vertically along the column and extend horizontally away from the column
- Right horizontal bar: attached to the right vertical column. Can slide vertically along the column and extend horizontally away from the column
- Movement of either vertical column or horizontal bar directly affects the position of the end-effector

### Joint Configuration
| Joint Name                     | Type        | Function |
|--------------------------------|-------------|----------|
| ColumnLeftBearingJoint         | Prismatic   | Vertical position of left column attachment point |
| ColumnRightBearingJoint        | Prismatic   | Vertical position of right column attachment point |
| ArmLeftJoint                   | Prismatic   | Horizontal extension length of left bar |
| ArmRightJoint                  | Prismatic   | Horizontal extension length of right bar |
| BaseJoint                      | Revolute    | Yaw rotation of entire assembly |

### End-effector
- Single 3-fingers gripper mounted at the end of the two horizontal bars
- Position and orientation are determined by the four joint positions and base rotation

### Control Characteristics
- Five-dimensional command vector required for full control
- End-effector positions are controlled separately

---

## Mobile Base
- Four-wheel omnidirectional drive system
- Capable of holonomic motion (movement in any direction without reorientation)
- Position and orientation tracked through base footprint reference

## Control Interface
- Command channels:
  - Base position and orientation targets
  - End-effector position targets
  - Mode selection between direct joint control and inverse kinematics
  - Raw joint command inputs
- Input methods:
  - Keyboard controls for arm and base movement
  - Mouse controls for camera manipulation in GLFW mode
  - Dedicated commands for gripper operation

---

## HOW TO USE

```bash
git clone https://github.com/obotx/mobile-manipulator.git
cd mobile-manipulator
```


```bash
python -m venv obotx_manip

source obotx_manip/bin/activate   # Linux
obotx_manip\Scripts\Activate.ps1  # Windows via PowerShell
obotx_manip\Scripts\activate.bat  # Windows via cmd.exe

pip install -r requirements.txt
```

### MORPH - I
Plain environtment with free movement (GUI) :
```bash
python src/gui/play.py
```
![gui_morph_i_free_move](https://github.com/obotx/mobile-manipulator/blob/dev/assets/gui_morph_i_free_move.png)

Plain environtment with free movement :
```bash
python src/simulations/morph_i_free_move.py
```
![morph_i_free_move](https://github.com/obotx/mobile-manipulator/blob/dev/assets/basic_glfw_morph_i_free_move.png)

Market Environtment with Trajectory :
```bash
python src/simulations/morph_i_market_trajectory.py
```
![morph_i_market_trajectory](https://github.com/obotx/mobile-manipulator/blob/dev/assets/basic_glfw_morph_i_trajectory.png)

### MORPH - II
Plain environtment with free movement :
```bash
python src/simulations/morph_ii_free_move.py
```
![morph_ii_free_move](https://github.com/obotx/mobile-manipulator/blob/dev/assets/basic_glfw_morph_ii_free_move.png)

Market Environtment with Trajectory :
```bash
python src/simulations/morph_ii_kitchen_trajectory.py
```
![morph_ii_kitchen_trajectory](https://github.com/obotx/mobile-manipulator/blob/dev/assets/basic_glfw_morph_ii_trajectory.png)

## Acknowledgements

- **Kitchen assets** from [furniture_sim by vikashplus](https://github.com/vikashplus/furniture_sim)  
- **Market product assets** from [Scanned Objects MuJoCo Models by kevinzakka](https://github.com/kevinzakka/mujoco_scanned_objects)   
- **Mecanum wheel mobile base implementation** from [Mecanum Drive in MuJoCo by JunHeonYoon](https://github.com/JunHeonYoon/mujoco_mecanum)