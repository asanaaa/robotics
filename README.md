# robotics
Чтобы запустить упражнение 2:  
```bash
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo_launch.py
```
```bash
rviz2 -d carrot.rviz
```  
Чтобы настроить параметр задержки для упражнения 4:  
```bash
ros2 param set /listener delay 5.0
```  
Чтобы запустить упражнение 3:
```bash
ros2 launch turtle_multi_target multi_target_launch.py  
ros2 run turtle_multi_target target_switcher
```
