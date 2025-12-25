---
sidebar_position: 14
title: "باب 14: ہیومنائیڈ Kinematics اور Dynamics"
---

# باب 14: ہیومنائیڈ Kinematics اور Dynamics

## تعارف

ہیومنائیڈ روبوٹس کی kinematics اور dynamics سمجھنا ان کی حرکت کو کنٹرول کرنے کے لیے ضروری ہے۔ یہ باب forward/inverse kinematics، dynamics، اور balance control کو تلاش کرتا ہے۔

## بنیادی تصور 1: Forward Kinematics

Forward kinematics joint angles سے end-effector position حساب کرتا ہے:

```python
import numpy as np

def forward_kinematics(joint_angles, link_lengths):
    """Forward kinematics for a simple 2-link arm"""
    theta1, theta2 = joint_angles
    l1, l2 = link_lengths

    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    return np.array([x, y])
```

## بنیادی تصور 2: Inverse Kinematics

Inverse kinematics target position سے joint angles حساب کرتا ہے:

```python
def inverse_kinematics(target, link_lengths):
    """Inverse kinematics for a 2-link arm"""
    x, y = target
    l1, l2 = link_lengths

    # Cosine law
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(cos_theta2)

    theta1 = np.arctan2(y, x) - np.arctan2(
        l2 * np.sin(theta2),
        l1 + l2 * np.cos(theta2)
    )

    return np.array([theta1, theta2])
```

## بنیادی تصور 3: Balance Control

ہیومنائیڈ روبوٹس کو چلتے وقت توازن برقرار رکھنا ہوتا ہے:
- **ZMP (Zero Moment Point)**: توازن کی حالت
- **CoM (Center of Mass)**: کمیت کا مرکز
- **Inverted Pendulum Model**: سادہ ماڈل

## اہم نکات

- Kinematics joint space اور task space کو جوڑتی ہے
- Forward kinematics تجزیاتی حل رکھتی ہے
- Inverse kinematics میں متعدد حل ہو سکتے ہیں
- Balance control ہیومنائیڈز کے لیے اہم ہے

## اگلے باب کا پیش نظارہ

باب 15 manipulation اور grasping کو تلاش کرے گا۔
