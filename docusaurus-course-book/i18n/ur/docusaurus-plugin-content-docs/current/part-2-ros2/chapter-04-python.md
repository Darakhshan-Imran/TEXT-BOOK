---
sidebar_position: 4
title: "باب 4: ROS 2 اور Python کے ساتھ Building"
---

# باب 4: ROS 2 اور Python کے ساتھ Building

## تعارف

Python اپنی سادگی، پڑھنے کی آسانی، اور سائنٹیفک کمپیوٹنگ لائبریریز کے بھرپور ایکو سسٹم کی وجہ سے روبوٹکس ڈیولپمنٹ کے لیے سب سے مقبول زبانوں میں سے ایک بن گئی ہے۔ ROS 2 Python کلائنٹ لائبریری (rclpy) ROS 2 مڈل ویئر کے لیے ایک طاقتور انٹرفیس فراہم کرتی ہے، جو ڈویلپرز کو کم سے کم boilerplate کوڈ کے ساتھ جدید روبوٹک ایپلیکیشنز بنانے کے قابل بناتی ہے۔

Python کی interpreted نوعیت اور dynamic typing اسے روبوٹکس میں تیز prototyping اور iterative ڈیولپمنٹ کے لیے مثالی بناتی ہے۔ Python کے بھرپور ایکو سسٹم اور ROS 2 کے مضبوط کمیونیکیشن انفراسٹرکچر کا امتزاج سادہ سینسر نوڈز سے لے کر پیچیدہ AI سے چلنے والے روبوٹک سسٹمز تک سب کچھ تیار کرنے کے لیے ایک زبردست پلیٹ فارم فراہم کرتا ہے۔

## بنیادی تصور 1: Python میں Custom Message Types

ROS 2 میں custom message types بنانا ڈویلپرز کو ڈومین کے مخصوص ڈیٹا سٹرکچرز متعین کرنے کی اجازت دیتا ہے جو ان کی روبوٹک ایپلیکیشنز کی ضروریات سے بالکل میل کھاتے ہیں۔ Custom messages `.msg` انٹرفیس ڈیفینیشن لینگویج کا استعمال کرتے ہوئے متعین کیے جاتے ہیں۔

```python
# مثال 1: Custom Message Types کے ساتھ Advanced Node Implementation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from typing import Optional, Dict, Any


class RobotState:
    """Type hints اور validation کے ساتھ Custom robot state representation۔"""

    def __init__(self,
                 position: Vector3,
                 orientation_quat: Vector3,
                 timestamp: Time,
                 frame_id: str = 'base_link'):
        self.position = position
        self.orientation_quat = orientation_quat
        self.timestamp = timestamp
        self.frame_id = frame_id

        # Inputs کی تصدیق
        if not isinstance(position, Vector3):
            raise ValueError("Position ایک Vector3 میسج ہونا چاہیے")


class CustomMessageNode(Node):
    """
    Custom message types اور validation کا استعمال کرتے ہوئے Advanced node implementation۔
    """

    def __init__(self):
        super().__init__('custom_message_node')

        # قابل اعتماد کمیونیکیشن کے لیے QoS پروفائل
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # State updates کے لیے Timer
        self.state_timer = self.create_timer(0.1, self.update_robot_state)
        self.get_logger().info("Custom message node شروع ہو گیا")

    def update_robot_state(self) -> None:
        """متواتر robot state update اور publish کریں۔"""
        self.get_logger().debug("Robot state update ہو رہا ہے")


def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("صارف نے روک دیا")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## بنیادی تصور 2: Asynchronous Programming Patterns

ROS 2 Python میں Asynchronous programming main execution thread کو block کیے بغیر concurrent آپریشنز کی موثر handling ممکن بناتی ہے۔ یہ خاص طور پر روبوٹک ایپلیکیشنز کے لیے اہم ہے جنہیں بیک وقت متعدد sensor streams پروسیس کرنا، service requests handle کرنا، اور real-time performance برقرار رکھنا ہوتا ہے۔

```python
# مثال 2: ROS 2 Python کے ساتھ Async Programming
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import asyncio
import threading


class AsyncRobotController(Node):
    """
    ROS 2 Python نوڈز میں asynchronous programming patterns کا مظاہرہ۔
    """

    def __init__(self):
        super().__init__('async_robot_controller')

        # Concurrent execution کے لیے callback groups بنائیں
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()

        # کنٹرول کمانڈز کے لیے Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # سینسر ڈیٹا کے لیے Subscribers
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10,
            callback_group=self.sensor_cb_group
        )

        # Timestamps کے ساتھ سینسر ڈیٹا محفوظ کریں
        self.sensor_data = {
            'laser': {'data': None, 'timestamp': None}
        }

        self.get_logger().info("Async robot controller شروع ہو گیا")

    def laser_callback(self, msg: LaserScan) -> None:
        """Laser scan ڈیٹا asynchronously handle کریں۔"""
        self.sensor_data['laser']['data'] = msg
        self.sensor_data['laser']['timestamp'] = self.get_clock().now()
        self.get_logger().debug(f"Laser ڈیٹا موصول: {len(msg.ranges)} ranges")


def main(args=None):
    rclpy.init(args=args)
    node = AsyncRobotController()

    # Async callbacks handle کرنے کے لیے multi-threaded executor استعمال کریں
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("صارف نے روک دیا")
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## بنیادی تصور 3: Testing اور Quality Assurance

Testing مضبوط روبوٹک سسٹمز تیار کرنے کا ایک اہم پہلو ہے، اور ROS 2 Python نوڈز کی testing کے لیے جامع ٹولز فراہم کرتا ہے۔ موثر testing strategies میں unit testing، integration testing، اور system-level testing شامل ہیں۔

```python
# مثال 3: ROS 2 Python میں Testing اور Mocking
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SimpleNavigationNode(Node):
    """Testing مقاصد کے لیے سادہ navigation node۔"""

    def __init__(self):
        super().__init__('simple_navigation_node')

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command_subscription = self.create_subscription(
            String, 'nav_commands', self.command_callback, 10)

        # اندرونی حالت
        self.current_goal = None
        self.is_moving = False
        self.position = {'x': 0.0, 'y': 0.0}

    def command_callback(self, msg: String) -> None:
        """Navigation commands handle کریں۔"""
        command = msg.data.strip().lower()

        if command.startswith('goto:'):
            coords_str = command.split(':', 1)[1]
            x, y = [float(c.strip()) for c in coords_str.split(',')]
            self.current_goal = {'x': x, 'y': y}
            self.get_logger().info(f"نیا گول سیٹ: ({x}, {y})")
        elif command == 'stop':
            self.current_goal = None
            self.is_moving = False


class TestSimpleNavigationNode(unittest.TestCase):
    """SimpleNavigationNode کے لیے Unit tests۔"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SimpleNavigationNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_state(self):
        """نوڈ صحیح حالت کے ساتھ شروع ہوا۔"""
        self.assertIsNone(self.node.current_goal)
        self.assertFalse(self.node.is_moving)

    def test_command_parsing(self):
        """Command parsing functionality test کریں۔"""
        msg = String()
        msg.data = "goto:1.0,2.0"
        self.node.command_callback(msg)

        self.assertIsNotNone(self.node.current_goal)
        self.assertEqual(self.node.current_goal['x'], 1.0)
        self.assertEqual(self.node.current_goal['y'], 2.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
```

## بنیادی تصور 4: Python Nodes میں Performance Optimization

ROS 2 Python nodes میں performance optimization کئی strategies پر مشتمل ہے بشمول موثر message handling، مناسب resource management، اور high-performance computing کے لیے Python کی صلاحیتوں کا فائدہ اٹھانا۔

اہم optimization strategies میں شامل ہیں:
- عددی حسابات کے لیے NumPy استعمال کرنا
- Tight loops میں object creation کو کم سے کم کرنا
- موثر data structures استعمال کرنا
- I/O آپریشنز کو optimize کرنا

```python
# مثال 4: Python Nodes میں Performance Optimization
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from collections import deque
import numpy as np
import time


class OptimizedPointCloudProcessor(Node):
    """
    Performance optimization techniques کا مظاہرہ کرنے والا Optimized point cloud processor۔
    """

    def __init__(self):
        super().__init__('optimized_point_cloud_processor')

        self.input_subscription = self.create_subscription(
            PointCloud2, 'input_pointcloud', self.pointcloud_callback, 10)
        self.output_publisher = self.create_publisher(
            PointCloud2, 'processed_pointcloud', 10)

        # Optimization: موثر appends/pops کے لیے deque استعمال کریں
        self.processing_buffer = deque(maxlen=10)

        # Optimization: بار بار allocations سے بچنے کے لیے arrays پہلے سے allocate کریں
        self.working_array = np.empty((10000, 3), dtype=np.float32)

        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.message_count = 0

        self.get_logger().info("Optimized point cloud processor شروع ہو گیا")

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """Point cloud messages پروسیس کرنے کے لیے Optimized callback۔"""
        start_time = time.perf_counter()

        try:
            # NumPy کے ساتھ موثر پروسیسنگ
            points = self._fast_pointcloud_to_numpy(msg)
            processed_points = self._process_points_vectorized(points)

            # Performance monitor کریں
            processing_time = time.perf_counter() - start_time
            self.processing_times.append(processing_time)
            self.message_count += 1

            if self.message_count % 50 == 0:
                avg_time = sum(self.processing_times) / len(self.processing_times)
                self.get_logger().info(
                    f"{self.message_count} پیغامات پروسیس ہوئے، "
                    f"اوسط وقت: {avg_time*1000:.2f}ms"
                )

        except Exception as e:
            self.get_logger().error(f"Point cloud پروسیسنگ میں خرابی: {e}")

    def _fast_pointcloud_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """PointCloud2 میسج کو numpy array میں موثر طریقے سے تبدیل کریں۔"""
        points = np.frombuffer(msg.data, dtype=np.float32)
        return points.reshape(-1, 3)

    def _process_points_vectorized(self, points: np.ndarray) -> np.ndarray:
        """Vectorized NumPy operations استعمال کرتے ہوئے points پروسیس کریں۔"""
        # Optimization: Loops کی بجائے boolean indexing استعمال کریں
        distances = np.linalg.norm(points, axis=1)
        valid_mask = (distances >= 0.1) & (distances <= 10.0)
        return points[valid_mask]


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedPointCloudProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stats = node.get_performance_stats() if hasattr(node, 'get_performance_stats') else {}
        node.get_logger().info(f"آخری performance stats: {stats}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## بنیادی تصور 5: ROS 2 Projects کے لیے Python Package Structure

مناسب package structure maintainable اور distributable ROS 2 Python projects بنانے کے لیے ضروری ہے۔ ایک منظم package structure ROS 2 conventions کی پیروی کرتی ہے جبکہ code organization، testing، اور distribution کے لیے Python best practices کو شامل کرتی ہے۔

```
my_robot_package/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_package/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── navigation_node.py
│   │   └── perception_node.py
│   ├── lib/
│   │   ├── __init__.py
│   │   ├── navigation/
│   │   │   ├── __init__.py
│   │   │   └── planner.py
│   │   └── utils/
│   │       ├── __init__.py
│   │       └── math_helpers.py
├── launch/
│   └── robot.launch.py
├── config/
│   └── robot_params.yaml
└── test/
    └── test_navigation.py
```

## عملدرآمد کا نقطہ نظر

ROS 2 اور Python کے ساتھ building کرتے وقت، real-time systems میں Python کی حدود کو ذہن میں رکھتے ہوئے اس کی طاقتوں سے فائدہ اٹھانا اہم ہے۔ اہم بات یہ ہے کہ اپنے کوڈ کو اس طرح ترتیب دیں کہ Python کی expressiveness اور بھرپور ecosystem کا فائدہ اٹھایا جا سکے جبکہ ضرورت کے مطابق performance optimizations نافذ کی جائیں۔

## عام غلطیاں

1. **Memory Leaks**: طویل عرصے تک چلنے والے نوڈز میں object lifecycle کا صحیح انتظام نہ کرنا وقت کے ساتھ میموری جمع ہونے کا باعث بن سکتا ہے۔

2. **Threading مسائل**: ROS 2 threading model کو Python کی threading کے ساتھ غلط طریقے سے ملانا race conditions اور deadlocks کا سبب بن سکتا ہے۔

3. **Performance Bottlenecks**: اہم کوڈ paths کو optimize نہ کرنا خراب real-time performance کا نتیجہ بن سکتا ہے۔

4. **Import مسائل**: غلط package structure اور imports runtime errors کا سبب بن سکتے ہیں۔

## حقیقی دنیا کی ایپلیکیشنز

Python پر مبنی ROS 2 سسٹمز تحقیق، prototyping، اور production ایپلیکیشنز میں بڑے پیمانے پر استعمال ہوتے ہیں۔ تحقیقی اداروں میں، Python تیز experimentation اور algorithm development ممکن بناتی ہے۔ Production سسٹمز میں، Python اکثر high-level control، planning، اور user interfaces کے لیے استعمال ہوتی ہے۔

## مشقیں

1. ایک Python package بنائیں جو custom message type نافذ کرے اور مناسب structure کا مظاہرہ کرے۔
2. ایک asynchronous node نافذ کریں جو متعدد concurrent sensor streams handle کرے۔
3. ROS 2 testing frameworks استعمال کرتے ہوئے navigation system کے لیے ایک جامع test suite تیار کریں۔

## اہم نکات

- Python کا بھرپور ecosystem اسے روبوٹکس ڈیولپمنٹ کے لیے بہترین بناتا ہے
- Maintainable کوڈ کے لیے مناسب package structure ضروری ہے
- Asynchronous programming patterns concurrent آپریشنز کے لیے performance بہتر بناتے ہیں
- جامع testing روبوٹک ایپلیکیشنز میں reliability یقینی بناتی ہے
- Real-time systems کے لیے Performance optimization اہم ہے

## مزید پڑھائی

- ROS 2 Python Client Library Documentation: https://docs.ros.org/en/humble/p/rclpy/
- Python Packaging Guide: https://packaging.python.org/en/latest/
- Python میں Async Programming: Real Python کی asyncio گائیڈ

## اگلے باب کا پیش نظارہ

باب 5 URDF کے ساتھ روبوٹس کی تفصیل میں جائے گا، یہ تلاش کرے گا کہ links، joints، اور visual properties کے ساتھ تفصیلی روبوٹ ماڈلز کیسے بنائے جائیں جو ROS 2 پر مبنی روبوٹک سسٹمز میں simulation اور control کے لیے استعمال ہو سکیں۔
