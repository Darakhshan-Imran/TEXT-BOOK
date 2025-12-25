---
sidebar_position: 15
title: "باب 15: Manipulation اور Grasping"
---

# باب 15: Manipulation اور Grasping

## تعارف

Manipulation اور grasping روبوٹکس کے اہم موضوعات ہیں جو روبوٹس کو اشیاء کو اٹھانے، منتقل کرنے، اور ہیرا پھیری کرنے کے قابل بناتے ہیں۔

## بنیادی تصور 1: Grasp Planning

Grasp planning میں اشیاء کو پکڑنے کا بہترین طریقہ تلاش کرنا شامل ہے:

```python
class GraspPlanner:
    def __init__(self, gripper_width):
        self.gripper_width = gripper_width

    def plan_grasp(self, object_pose, object_dimensions):
        """Plan a grasp for a simple box object"""
        grasp_poses = []

        # Top grasp
        top_grasp = object_pose.copy()
        top_grasp[2] += object_dimensions[2] / 2
        grasp_poses.append(('top', top_grasp))

        # Side grasps
        for angle in [0, 90, 180, 270]:
            side_grasp = self.compute_side_grasp(
                object_pose, object_dimensions, angle
            )
            grasp_poses.append((f'side_{angle}', side_grasp))

        return grasp_poses
```

## بنیادی تصور 2: Motion Planning

Motion planning روبوٹ بازو کو رکاوٹوں سے بچتے ہوئے منزل تک لے جاتا ہے:
- **RRT (Rapidly-exploring Random Trees)**
- **PRM (Probabilistic Roadmaps)**
- **OMPL (Open Motion Planning Library)**

## بنیادی تصور 3: Force Control

Grasping میں force control اہم ہے:
- Impedance control
- Admittance control
- Hybrid position/force control

## اہم نکات

- Grasp planning اشیاء کو پکڑنے کا طریقہ تلاش کرتا ہے
- Motion planning collision-free راستے بناتا ہے
- Force control نازک اشیاء کو سنبھالنے کے لیے اہم ہے
- Sensor feedback grasp کامیابی کے لیے ضروری ہے

## اگلے باب کا پیش نظارہ

باب 16 voice control اور Whisper کو تلاش کرے گا۔
