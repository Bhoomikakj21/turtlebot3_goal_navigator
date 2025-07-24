# TurtleBot3 Goal Navigator

This ROS package moves **TurtleBot3** through waypoints using proportional control — with and without obstacle avoidance.

---

## Launch Options

### Basic Navigation (no avoidance)

```bash
roslaunch turtlebot3_goal_navigator goal_nav_basic.launch
```

### Navigation + Obstacle Avoidance

```bash
roslaunch turtlebot3_goal_navigator goal_nav_with_avoidance.launch
```

---

## Scripts Included

| Script | Description |
|--------|-------------|
| `goal_nav_basic.py` | Moves robot through waypoints using odometry and proportional control |
| `goal_nav_with_avoidance.py` | Same as above, but with reactive obstacle avoidance using `/scan` |

---

## Dependencies

- `rospy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `turtlebot3_gazebo`
