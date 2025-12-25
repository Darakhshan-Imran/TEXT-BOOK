---
sidebar_position: 13
title: "باب 13: Nav2 کے ساتھ Navigation"
---

# باب 13: Nav2 کے ساتھ Navigation

## تعارف

Nav2 (Navigation2) ROS 2 کا معیاری navigation stack ہے جو موبائل روبوٹس کے لیے خودمختار navigation فراہم کرتا ہے۔ یہ path planning، obstacle avoidance، اور localization شامل کرتا ہے۔

## بنیادی تصور 1: Nav2 Architecture

Nav2 کے اہم اجزاء:
- **Planner Server**: Global path planning
- **Controller Server**: Local trajectory following
- **Behavior Server**: Recovery behaviors
- **BT Navigator**: Behavior tree based navigation

## بنیادی تصور 2: Navigation کا استعمال

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()
navigator.waitUntilNav2Active()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)

while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f"Distance remaining: {feedback.distance_remaining}")
```

## بنیادی تصور 3: Costmaps

Costmaps navigation کے لیے ماحول کی نمائندگی کرتے ہیں:
- **Global Costmap**: پوری map کی نمائندگی
- **Local Costmap**: روبوٹ کے ارد گرد کا علاقہ
- **Inflation Layer**: رکاوٹوں کے گرد safety zone

## اہم نکات

- Nav2 ROS 2 کا معیاری navigation stack ہے
- Behavior trees flexible navigation ممکن بناتے ہیں
- Costmaps obstacle avoidance فراہم کرتے ہیں
- متعدد planners اور controllers دستیاب ہیں

## اگلے باب کا پیش نظارہ

باب 14 humanoid kinematics اور dynamics کو تلاش کرے گا۔
