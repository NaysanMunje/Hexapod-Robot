import math
import pathlib

# (x, y, yaw0): coxa default — middle horizontal; corners ⊥ to 79.893 sides
hips = {
    "LF": (0.073282, 0.061608, 1.161125),
    "LM": (0.0, 0.093430, math.pi / 2),
    "LR": (-0.073282, 0.061608, 1.980468),
    "RF": (0.073282, -0.061608, -1.161125),
    "RM": (0.0, -0.093430, -math.pi / 2),
    "RR": (-0.073282, -0.061608, -1.980468),
}

parts = [
    """<?xml version="1.0"?>
<!-- Full hexapod stick model: body + 6 legs at vertices. meters, +X fwd +Y left +Z up -->
<robot name="hexapod">
  <link name="base_link">
    <visual>
      <geometry><box size="0.146564 0.186859 0.008"/></geometry>
      <material name="plate"><color rgba="0.35 0.55 0.85 0.4"/></material>
    </visual>
  </link>
"""
]


def leg_block(p, x, y, yaw):
    return f"""
  <!-- ========== LEG {p} ========== -->
  <joint name="{p}_attach" type="fixed">
    <parent link="base_link"/>
    <child link="{p}_hip"/>
    <origin xyz="{x:.6f} {y:.6f} 0" rpy="0 0 {yaw:.6f}"/>
  </joint>
  <link name="{p}_hip"/>

  <joint name="{p}_coxa_joint" type="revolute">
    <parent link="{p}_hip"/>
    <child link="{p}_coxa"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="3"/>
  </joint>
  <link name="{p}_coxa">
    <visual>
      <origin xyz="0.0265 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder length="0.053" radius="0.0035"/></geometry>
      <material name="{p}_c"><color rgba="0.8 0.25 0.25 1"/></material>
    </visual>
  </link>

  <joint name="{p}_femur_joint" type="revolute">
    <parent link="{p}_coxa"/>
    <child link="{p}_femur"/>
    <origin xyz="0.053 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.789" upper="0" effort="10" velocity="3"/>
  </joint>
  <link name="{p}_femur">
    <visual>
      <origin xyz="0.03868 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder length="0.07736" radius="0.0035"/></geometry>
      <material name="{p}_f"><color rgba="0.2 0.55 0.9 1"/></material>
    </visual>
  </link>

  <joint name="{p}_knee_joint" type="revolute">
    <parent link="{p}_femur"/>
    <child link="{p}_shin1"/>
    <origin xyz="0.07736 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.136" effort="10" velocity="3"/>
  </joint>
  <link name="{p}_shin1">
    <visual>
      <origin xyz="0.011179 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder length="0.022358" radius="0.003"/></geometry>
      <material name="{p}_s1"><color rgba="0.2 0.75 0.4 1"/></material>
    </visual>
  </link>

  <joint name="{p}_shin1_shin2" type="fixed">
    <parent link="{p}_shin1"/>
    <child link="{p}_shin2"/>
    <origin xyz="0.022358 0 0" rpy="0 -0.2234 0"/>
  </joint>
  <link name="{p}_shin2">
    <visual>
      <origin xyz="0.035759 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder length="0.071518" radius="0.003"/></geometry>
      <material name="{p}_s2"><color rgba="0.15 0.65 0.35 1"/></material>
    </visual>
  </link>

  <joint name="{p}_shin2_shin3" type="fixed">
    <parent link="{p}_shin2"/>
    <child link="{p}_shin3"/>
    <origin xyz="0.071518 0 0" rpy="0 -0.3508 0"/>
  </joint>
  <link name="{p}_shin3">
    <visual>
      <origin xyz="0.01557 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder length="0.03114" radius="0.003"/></geometry>
      <material name="{p}_s3"><color rgba="0.1 0.55 0.3 1"/></material>
    </visual>
  </link>

  <joint name="{p}_foot_fixed" type="fixed">
    <parent link="{p}_shin3"/>
    <child link="{p}_foot"/>
    <origin xyz="0.03114 0 0" rpy="0 0 0"/>
  </joint>
  <link name="{p}_foot">
    <visual>
      <geometry><sphere radius="0.005"/></geometry>
      <material name="{p}_ft"><color rgba="0.95 0.85 0.2 1"/></material>
    </visual>
  </link>
"""


for name, (x, y, yaw) in hips.items():
    parts.append(leg_block(name, x, y, yaw))

parts.append("</robot>\n")

path = pathlib.Path(r"c:\Users\naysa\Desktop\projects\summer 26\hexapod\hexapod_description\urdf\hexapod.urdf")
path.write_text("".join(parts), encoding="utf-8")
print("wrote", path, path.stat().st_size)
