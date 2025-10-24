warthog
==

Common packages for the Warthog platform, including messages and robot description. These are packages relevant to all workspaces, whether simulation, desktop, or on the robot's own headless PC.

## Installation

Copy the packages under the colcon_ws/src and build the workspace.

```bash
git clone https://github.com/kyavuzkurt/warthog
mv warthog/* <path_to_your_colcon_ws>/src/.
cd <path_to_your_colcon_ws>
colcon build warthog_gazebo 
```

## Usage Instructions

To launch gazebo and rviz 

```bash
source install/setup.bash
ros2 launch warthog_gazebo empty_world.launch.py
```

To only launch gazebo
```bash
source install/setup.bash
ros2 launch warthog_gazebo empty_world.launch.py rviz:=false
```


## Troubleshooting

If you get errors while loading the models from gazebo:

```bash
[ign gazebo-1] [GUI] [Err] [SceneManager.cc:404] Failed to load geometry for visual: right_diff_unit_link_fixed_joint_lump__right_diff_unit_taillight_link_visual_6
[ign gazebo-1] [GUI] [Err] [SystemPaths.cc:378] Unable to find file with URI [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [SystemPaths.cc:473] Could not resolve file [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [MeshManager.cc:173] Unable to find file[model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [MeshDescriptor.cc:56] Mesh manager can't find mesh named [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [Ogre2MeshFactory.cc:562] Cannot load null mesh [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [Ogre2MeshFactory.cc:125] Failed to get Ogre item for [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [SceneManager.cc:404] Failed to load geometry for visual: front_right_wheel_link_visual
[ign gazebo-1] [GUI] [Err] [SystemPaths.cc:378] Unable to find file with URI [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [SystemPaths.cc:473] Could not resolve file [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [MeshManager.cc:173] Unable to find file[model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [MeshDescriptor.cc:56] Mesh manager can't find mesh named [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [Ogre2MeshFactory.cc:562] Cannot load null mesh [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [Ogre2MeshFactory.cc:125] Failed to get Ogre item for [model://warthog_description/meshes/wheel.stl]
[ign gazebo-1] [GUI] [Err] [SceneManager.cc:404] Failed to load geometry for visual: rear_right_wheel_link_visual
```
Try linking the model files manually to the models direction

```bash
mkdir -p <path_to_your_colcon_ws>/install/warthog_description/share/warthog_description/models/warthog_description
cd <path_to_your_colcon_ws>/install/warthog_description/share/warthog_description/models/warthog_description
ln -s ../../meshes meshes
cp ../../model.config .
```
You should see the model loaded correctly when launching gazebo

