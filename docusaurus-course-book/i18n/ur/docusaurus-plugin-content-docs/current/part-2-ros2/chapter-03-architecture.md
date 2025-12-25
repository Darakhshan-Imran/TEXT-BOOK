---
sidebar_position: 3
title: "باب 3: ROS 2 آرکیٹیکچر کے بنیادی اصول"
---

# باب 3: ROS 2 آرکیٹیکچر کے بنیادی اصول

## تعارف

ROS 2 آرکیٹیکچر اپنے پیشرو سے ایک اہم ارتقاء کی نمائندگی کرتا ہے، جس میں روبوٹکس ڈیولپمنٹ کے سالوں سے سیکھے گئے اسباق کو شامل کیا گیا ہے اور جدید روبوٹک ایپلیکیشنز کی ضروریات کو پورا کیا گیا ہے۔ یہ باب بنیادی آرکیٹیکچرل تصورات کو تلاش کرتا ہے جو ROS 2 کو روبوٹکس ڈیولپمنٹ کے لیے ایک طاقتور فریم ورک بناتے ہیں، خاص طور پر نوڈ lifecycle مینجمنٹ، Quality of Service (QoS) پالیسیز، کلائنٹ لائبریری نفاذ، اور کمپوننٹ پر مبنی آرکیٹیکچرز پر توجہ کے ساتھ۔

ROS 2 کا آرکیٹیکچر Data Distribution Service (DDS) معیار کے گرد بنایا گیا ہے، جو بنیادی کمیونیکیشن مڈل ویئر فراہم کرتا ہے۔ یہ ڈیزائن انتخاب ROS 2 کو ریئل ٹائم سسٹمز کی حمایت کرنے، سیکیورٹی فیچرز کو بہتر بنانے، اور ملٹی روبوٹ سسٹمز کے لیے scalability کو بہتر بنانے کے قابل بناتا ہے۔

آرکیٹیکچر کو سمجھنا مضبوط روبوٹک سسٹمز تیار کرنے کے لیے اہم ہے۔ سسٹم سطح پر کیے گئے آرکیٹیکچرل فیصلے براہ راست روبوٹ ایپلیکیشنز کی کارکردگی، قابل اعتمادی، اور دیکھ بھال پر اثر ڈالتے ہیں۔

## بنیادی تصور 1: نوڈ Lifecycle مینجمنٹ

نوڈ lifecycle مینجمنٹ ROS 2 کی ایک بنیادی خصوصیت ہے جو نوڈز کی آپریشنل زندگی بھر ان کی حالت کو کنٹرول کرنے کے لیے منظم طریقے فراہم کرتی ہے۔ lifecycle سسٹم مختلف آپریشنل حالتوں کے درمیان محفوظ اور قابل پیشگوئی منتقلی کی ضرورت کو پورا کرتا ہے، جو خاص طور پر حفاظت کے لحاظ سے اہم روبوٹک ایپلیکیشنز میں اہم ہے۔

lifecycle سسٹم واضح طور پر بیان کردہ حالتوں کا ایک سیٹ متعین کرتا ہے جو ایک نوڈ رکھ سکتا ہے:

- **Unconfigured**: ابتدائی حالت جہاں نوڈ بنایا گیا ہے لیکن ابھی تک configure نہیں ہوا
- **Inactive**: نوڈ configure ہے لیکن فعال طور پر ڈیٹا پروسیس نہیں کر رہا
- **Active**: نوڈ مکمل طور پر آپریشنل ہے اور ڈیٹا پروسیس کر رہا ہے
- **Finalized**: نوڈ بند ہو گیا ہے اور صاف ہو گیا ہے

```python
# مثال 1: نوڈ Lifecycle مینجمنٹ
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String


class LifecycleTalker(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_talker')
        self.pub = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو configure کرنے کے لیے Callback۔"""
        self.get_logger().info(f'نوڈ configure ہو رہا ہے: {state.id}')

        # configuration کے دوران publisher بنائیں
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)

        # دیگر ریسورسز initialize کریں
        self.counter = 0

        # کامیاب configuration کی نشاندہی کے لیے SUCCESS واپس کریں
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو activate کرنے کے لیے Callback۔"""
        self.get_logger().info(f'نوڈ activate ہو رہا ہے: {state.id}')

        # publisher کو activate کریں
        self.pub.on_activate()

        # متواتر publishing کے لیے timer بنائیں
        self.timer = self.create_timer(1.0, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """نوڈ کو deactivate کرنے کے لیے Callback۔"""
        self.get_logger().info(f'نوڈ deactivate ہو رہا ہے: {state.id}')

        # publisher کو deactivate کریں
        self.pub.on_deactivate()

        # timer ختم کریں
        self.destroy_timer(self.timer)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ریسورسز صاف کرنے کے لیے Callback۔"""
        self.get_logger().info(f'نوڈ صاف ہو رہا ہے: {state.id}')

        # publisher صاف کریں
        self.destroy_publisher(self.pub)
        self.pub = None

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """پیغامات شائع کرنے کے لیے Timer callback۔"""
        if self.pub is not None and self.pub.handle is not None:
            msg = String()
            msg.data = f'Lifecycle پیغام #{self.counter}'
            self.pub.publish(msg)
            self.counter += 1
            self.get_logger().info(f'شائع ہوا: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # lifecycle نوڈ بنائیں
    lifecycle_node = LifecycleTalker()

    # مظاہرے کے لیے lifecycle transitions کو دستی طور پر trigger کریں
    # عملی طور پر، یہ lifecycle services کے ذریعے trigger ہوں گے

    # نوڈ کو configure کریں
    lifecycle_node.trigger_configure()

    # نوڈ کو activate کریں
    lifecycle_node.trigger_activate()

    # کچھ دیر چلائیں
    try:
        rclpy.spin(lifecycle_node)
    except KeyboardInterrupt:
        pass
    finally:
        # deactivate اور cleanup
        lifecycle_node.trigger_deactivate()
        lifecycle_node.trigger_cleanup()

        lifecycle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ مثال ROS 2 میں lifecycle مینجمنٹ سسٹم کا مظاہرہ کرتی ہے۔ نوڈ واضح طور پر بیان کردہ حالتوں (configure، activate، deactivate، cleanup) سے گزرتا ہے جس میں ہر منتقلی کے لیے مخصوص callbacks ہیں۔

## بنیادی تصور 2: Quality of Service (QoS) پالیسیز

ROS 2 میں Quality of Service (QoS) پالیسیز نوڈز کے درمیان کمیونیکیشن کے رویے کو کنٹرول کرنے کے لیے ایک طاقتور میکانزم فراہم کرتی ہیں۔ QoS ڈویلپرز کو reliability، durability، history، اور میسج ڈیلیوری کے دیگر پہلوؤں کے لیے ضروریات واضح کرنے کی اجازت دیتا ہے۔

چار بنیادی QoS پالیسیز ہیں:

- **Reliability**: تعین کرتی ہے کہ پیغامات قابل اعتماد طریقے سے ڈیلیور ہوں یا best-effort
- **Durability**: کنٹرول کرتی ہے کہ دیر سے شامل ہونے والے subscribers کو پرانے پیغامات ملیں یا نہیں
- **History**: واضح کرتی ہے کہ دیر سے شامل ہونے والوں کے لیے کتنے پیغامات محفوظ کریں
- **Deadline**: لگاتار پیغامات کے درمیان زیادہ سے زیادہ وقت مقرر کرتی ہے

```python
# مثال 2: QoS پالیسی Configuration
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')

        # مختلف استعمال کے معاملات کے لیے مختلف QoS پروفائلز متعین کریں

        # سینسر ڈیٹا کے لیے: reliable، volatile، آخری 10 رکھیں
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # configuration ڈیٹا کے لیے: reliable، transient-local، سب رکھیں
        config_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # best-effort ڈیٹا کے لیے: best-effort، volatile، آخری 1 رکھیں
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # مختلف QoS پروفائلز کے ساتھ publishers بنائیں
        self.sensor_pub = self.create_publisher(String, 'sensor_data', sensor_qos)
        self.config_pub = self.create_publisher(String, 'config_data', config_qos)
        self.best_effort_pub = self.create_publisher(String, 'best_effort_data', best_effort_qos)

        # پیغامات شائع کرنے کے لیے Timer
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """مختلف QoS پروفائلز کے ساتھ پیغامات شائع کریں۔"""
        sensor_msg = String()
        sensor_msg.data = f'سینسر ریڈنگ #{self.counter}'
        self.sensor_pub.publish(sensor_msg)

        config_msg = String()
        config_msg.data = f'Configuration قدر #{self.counter}'
        self.config_pub.publish(config_msg)

        best_effort_msg = String()
        best_effort_msg.data = f'Best effort ڈیٹا #{self.counter}'
        self.best_effort_pub.publish(best_effort_msg)

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    qos_publisher = QoSPublisher()

    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ مثال ظاہر کرتی ہے کہ مختلف ضروریات کے ساتھ مختلف قسم کے ڈیٹا کے لیے QoS پالیسیز کیسے configure کی جا سکتی ہیں۔

## بنیادی تصور 3: کلائنٹ لائبریریز اور مڈل ویئر آرکیٹیکچر

ROS 2 متعدد کلائنٹ لائبریریز کی حمایت کرتا ہے جو بنیادی مڈل ویئر کے لیے زبان کے مخصوص انٹرفیسز فراہم کرتی ہیں۔ آرکیٹیکچر کلائنٹ لائبریری لیئر (rclcpp، rclpy، وغیرہ) کو مڈل ویئر لیئر (rcl) اور DDS نفاذ سے الگ کرتا ہے۔

مڈل ویئر آرکیٹیکچر کئی layers پر مشتمل ہے:

- **Application Layer**: کلائنٹ لائبریریز (rclcpp، rclpy) استعمال کرنے والا یوزر کوڈ
- **Client Library Layer**: زبان کے مخصوص ROS 2 انٹرفیسز
- **ROS Client Library (rcl)**: عام فعالیت فراہم کرنے والا پتلا C wrapper
- **ROS Middleware (rmw)**: DDS نفاذ کا تجریدی انٹرفیس
- **DDS Implementation**: مخصوص DDS vendor نفاذ (Fast DDS، Cyclone DDS، وغیرہ)

## بنیادی تصور 4: کمپوننٹ پر مبنی آرکیٹیکچر

ROS 2 میں کمپوننٹ پر مبنی آرکیٹیکچر متعدد نوڈز کو ایک ہی پروسیس میں چلانے کی اجازت دیتا ہے، inter-process کمیونیکیشن overhead کو ختم کرکے کارکردگی بہتر بناتا ہے جبکہ نوڈ پر مبنی سسٹم کے ماڈیولیرٹی فوائد کو برقرار رکھتا ہے۔

کمپوننٹ آرکیٹیکچر کئی فوائد فراہم کرتا ہے:

- قریب سے جڑے اجزاء کے درمیان کم کمیونیکیشن لیٹنسی
- الگ پروسیسز کے مقابلے میں کم میموری overhead
- زیادہ فریکوئنسی آپریشنز کے لیے بہتر کارکردگی
- بہتر ریسورس استعمال

```python
# مثال 4: فیڈ بیک کے ساتھ Action Server
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        """کلائنٹ کی درخواست کو قبول یا مسترد کریں۔"""
        self.get_logger().info('گول درخواست موصول ہوئی')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """کلائنٹ کی منسوخی درخواست کو قبول یا مسترد کریں۔"""
        self.get_logger().info('منسوخی درخواست موصول ہوئی')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """گول کو execute کریں۔"""
        self.get_logger().info('گول execute ہو رہا ہے...')

        # فیڈ بیک اور نتیجہ پیغامات بنائیں
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        result = Fibonacci.Result()

        # فیڈ بیک کے ساتھ پروسیسنگ simulate کریں
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('گول منسوخ ہوا')
                goal_handle.canceled()
                result.sequence = feedback_msg.sequence
                return result

            # اگلا Fibonacci نمبر حساب کریں
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # فیڈ بیک شائع کریں
            self.get_logger().info(f'فیڈ بیک شائع ہو رہا ہے: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        # نتیجہ populate کریں
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'گول کامیاب ہوا نتیجہ کے ساتھ: {result.sequence}')
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ action server مثال فیڈ بیک کے ساتھ طویل چلنے والے کام کا مظاہرہ کرتی ہے، جو روبوٹکس ایپلیکیشنز میں ایک عام پیٹرن ہے۔

## بنیادی تصور 5: سسٹم انضمام کے پیٹرنز

ROS 2 سسٹم انضمام کے لیے کئی آرکیٹیکچرل پیٹرنز فراہم کرتا ہے جو پیچیدہ، ملٹی کمپوننٹ روبوٹک سسٹمز بنانے کے قابل بناتے ہیں۔ ان پیٹرنز میں متعدد نوڈز کو مربوط کرنے کے لیے launch فائلز، configuration کے لیے پیرامیٹر مینجمنٹ، اور پیچیدہ رویوں کو مربوط کرنے کے لیے service orchestration شامل ہیں۔

```python
# مثال 5: متعدد اجزاء کے ساتھ Launch فائل
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """کمپوننٹ پر مبنی سسٹم کے لیے launch description تیار کریں۔"""

    # launch arguments کا اعلان کریں
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    debug_mode = LaunchConfiguration('debug_mode', default='false')

    # اجزاء کے لیے container بنائیں
    container = ComposableNodeContainer(
        name='robot_system_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # ملٹی تھریڈڈ container
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='اگر true ہو تو simulation clock استعمال کریں'),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='اگر true ہو تو debug logging فعال کریں'),
        container,
    ])
```

## عملدرآمد کا نقطہ نظر

ROS 2 کا آرکیٹیکچر روبوٹکس ڈیولپمنٹ کے حقیقی دنیا کے چیلنجز سے نمٹنے کے لیے احتیاط سے ڈیزائن کیا گیا ہے۔ layered نقطہ نظر مختلف نفاذ میں مستقل مزاجی برقرار رکھتے ہوئے لچک فراہم کرتا ہے۔ lifecycle مینجمنٹ سسٹم حفاظت کے لحاظ سے اہم ایپلیکیشنز میں محفوظ آپریشن کو یقینی بناتا ہے، جبکہ QoS پالیسیز مخصوص ضروریات سے ملنے کے لیے کمیونیکیشن رویے کو fine-tune کرنے کی اجازت دیتی ہیں۔

## عام غلطیاں

1. **QoS Mismatch**: publishers اور subscribers کے درمیان QoS پالیسیز کو ملانے میں ناکامی کمیونیکیشن ناکامیوں یا غیر متوقع رویے کا نتیجہ بن سکتی ہے۔

2. **ریسورس مینجمنٹ**: نوڈ lifecycles کا صحیح انتظام نہ کرنا ریسورس leaks اور غیر متوقع سسٹم رویے کا باعث بن سکتا ہے۔

3. **Threading مسائل**: ملٹی تھریڈڈ executors میں threading کو غلط طریقے سے handle کرنا race conditions اور crashes کا باعث بن سکتا ہے۔

4. **نیٹ ورک Configuration**: غلط طریقے سے configure کی گئی DDS سیٹنگز distributed سسٹمز میں discovery ناکامیوں اور کمیونیکیشن مسائل کا سبب بن سکتی ہیں۔

## حقیقی دنیا کی ایپلیکیشنز

ROS 2 کی آرکیٹیکچرل خصوصیات حقیقی دنیا کی روبوٹک ایپلیکیشنز کے لیے ضروری ہیں۔ خودمختار گاڑیوں میں، QoS پالیسیز حفاظت کے لحاظ سے اہم پیغامات کی قابل اعتماد ڈیلیوری کو یقینی بناتی ہیں۔ مینوفیکچرنگ میں، lifecycle مینجمنٹ پیچیدہ روبوٹک سیلز کے مربوط startup اور shutdown کو ممکن بناتی ہے۔

## مشقیں

1. ایک launch فائل بنائیں جو ماحولیاتی متغیرات کی بنیاد پر مشروط طور پر مختلف نوڈز کے سیٹ شروع کرے۔
2. ایک نوڈ نافذ کریں جو میسج criticality کی بنیاد پر مختلف ٹاپکس کے لیے مختلف QoS پالیسیز استعمال کرے۔
3. ایک کمپوننٹ پر مبنی سسٹم ڈیزائن کریں جو متعدد ذرائع سے ڈیٹا کو ضم کرے۔

## اہم نکات

- ROS 2 کا آرکیٹیکچر نوڈ lifecycle مینجمنٹ کے لیے منظم طریقے فراہم کرتا ہے
- QoS پالیسیز مخصوص ضروریات کے لیے کمیونیکیشن رویے کو fine-tune کرنے کے قابل بناتی ہیں
- layered مڈل ویئر آرکیٹیکچر لچک اور زبان کی آزادی فراہم کرتا ہے
- کمپوننٹ پر مبنی آرکیٹیکچر قریب سے جڑے سسٹمز کے لیے کارکردگی کے فوائد پیش کرتا ہے
- Launch فائلز سسٹم composition پر programmatic کنٹرول فراہم کرتی ہیں

## مزید پڑھائی

- ROS 2 Design Documents: https://design.ros2.org/
- DDS Specification: https://www.omg.org/spec/DDS/
- ROS 2 میں Quality of Service: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html

## اگلے باب کا پیش نظارہ

باب 4 ROS 2 اور Python کے ساتھ building کو تلاش کرے گا، جدید پروگرامنگ تکنیکوں، میسج customization، اور مضبوط Python پر مبنی روبوٹک ایپلیکیشنز تیار کرنے کے بہترین طریقوں میں غوطہ لگائے گا۔
