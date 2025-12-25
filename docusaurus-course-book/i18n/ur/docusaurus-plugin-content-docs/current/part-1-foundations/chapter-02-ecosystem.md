---
sidebar_position: 2
title: "باب 2: فزیکل AI ایکو سسٹم"
---

# باب 2: فزیکل AI ایکو سسٹم

## تعارف

فزیکل AI ایکو سسٹم مصنوعی ذہانت، روبوٹکس، اور حقیقی دنیا کے تعامل کے انقلابی ملاپ کی نمائندگی کرتا ہے۔ روایتی AI سسٹمز کے برعکس جو خالصتاً ڈیجیٹل ڈومینز میں کام کرتے ہیں، فزیکل AI ذہانت کو ٹھوس ماحول میں لاتی ہے، مشینوں کو جسمانی جگہوں میں سمجھنے، استدلال کرنے، اور عمل کرنے کے قابل بناتی ہے۔ یہ باب جدید فزیکل AI سسٹمز کی بنیادی تصورات کا تعارف کراتا ہے، خاص طور پر Robot Operating System 2 (ROS 2) پر توجہ کے ساتھ۔

فزیکل AI کی اہمیت کو آج کے تکنیکی منظر نامے میں بڑھا چڑھا کر بیان نہیں کیا جا سکتا۔ جیسے جیسے ہم ایسے دور کی طرف بڑھ رہے ہیں جہاں روبوٹس ہماری روزمرہ زندگی میں بغیر کسی رکاوٹ کے ضم ہو جائیں—گودام آٹومیشن سے لے کر گھریلو اسسٹنٹس تک—ہمیں ایسے مضبوط فریم ورکس کی ضرورت ہے جو محفوظ، قابل اعتماد، اور موثر روبوٹ آپریشن کو ممکن بنائیں۔ ROS 2 ایسے سسٹمز بنانے کے لیے ڈی فیکٹو معیار کے طور پر ابھرا ہے۔

اس باب کے دوران، ہم فزیکل AI ایکو سسٹم کے بنیادی تصورات کو تلاش کریں گے، بشمول کمیونیکیشن پیٹرنز، مڈل ویئر آرکیٹیکچر، اور پرسیپشن، پلاننگ، اور کنٹرول سسٹمز کا انضمام۔

## بنیادی تصور 1: ROS 2 کے بنیادی تصورات

ROS 2 (Robot Operating System 2) روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے جو پیچیدہ روبوٹک سسٹمز کی ترقی کو آسان بنانے کے لیے ٹولز، لائبریریز، اور کنونشنز کا مجموعہ فراہم کرتا ہے۔ اس کے مرکز میں، ROS 2 روبوٹ سافٹ ویئر اجزاء کے درمیان رابطے کی سہولت فراہم کرنے، distributed کمپیوٹیشن کا انتظام کرنے، اور روبوٹ ایپلیکیشنز کے لیے عام فعالیت فراہم کرنے کے لیے ڈیزائن کیا گیا ہے۔

ROS 2 کا آرکیٹیکچر اپنے پیشرو، ROS 1 سے بنیادی طور پر مختلف ہے۔ سب سے اہم تبدیلی Data Distribution Service (DDS) کو بنیادی کمیونیکیشن مڈل ویئر کے طور پر اپنانا ہے۔ یہ تبدیلی بہتر ریئل ٹائم کارکردگی، بہتر سیکیورٹی، اور ملٹی روبوٹ سسٹمز کے لیے بہتر scalability فراہم کرتی ہے۔

اہم آرکیٹیکچرل عناصر میں شامل ہیں:

- **نوڈز (Nodes)**: وہ پروسیسز جو کمپیوٹیشن انجام دیتے ہیں۔ نوڈز ROS 2 میں کمپیوٹیشن کی بنیادی اکائیاں ہیں۔
- **ٹاپکس (Topics)**: نامزد بسیں جن پر نوڈز پیغامات کا تبادلہ کرتے ہیں۔ ٹاپکس publish/subscribe کمیونیکیشن پیٹرن نافذ کرتے ہیں۔
- **سروسز (Services)**: نوڈز کے درمیان synchronous request/reply کمیونیکیشن۔ سروسز client/server پیٹرن نافذ کرتی ہیں۔
- **ایکشنز (Actions)**: فیڈ بیک اور گول مینجمنٹ کے ساتھ طویل چلنے والے کام۔

## بنیادی تصور 2: کمیونیکیشن پیٹرنز

ROS 2 میں کمیونیکیشن ایک distributed publish/subscribe آرکیٹیکچر پر مبنی ہے جو نوڈز کے درمیان ڈھیلے کپلنگ کو ممکن بناتا ہے۔ یہ ڈیزائن ماڈیولیرٹی اور لچک کو فروغ دیتا ہے، جو ڈویلپرز کو دوبارہ قابل استعمال اجزاء بنانے کی اجازت دیتا ہے۔

publish/subscribe پیٹرن ٹاپکس کے ذریعے نافذ کیا جاتا ہے۔ Publishers پیغامات ٹاپکس کو بھیجتے ہیں بغیر یہ جانے کہ کون سے subscribers انہیں وصول کریں گے۔ اسی طرح، subscribers ٹاپکس سنتے ہیں بغیر یہ جانے کہ کون سے publishers پیغامات بھیج رہے ہیں۔

```python
# مثال 1: بنیادی ROS 2 Publisher/Subscriber پیٹرن
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # سیکنڈز
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'سلام دنیا: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'شائع کر رہا ہے: "{msg.data}"')
        self.i += 1


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # غیر استعمال شدہ متغیر کی وارننگ روکیں

    def listener_callback(self, msg):
        self.get_logger().info(f'میں نے سنا: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_publisher)
    rclpy.spin_once(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ مثال ROS 2 میں بنیادی publisher/subscriber پیٹرن کا مظاہرہ کرتی ہے۔ publisher 2 Hz کی شرح سے "topic" نامی ٹاپک پر پیغامات بھیجتا ہے، جبکہ subscriber اسی ٹاپک کو سنتا ہے اور موصول ہونے والے پیغامات کو لاگ کرتا ہے۔

## بنیادی تصور 3: ہارڈویئر انضمام

ROS 2 سسٹمز کے ساتھ ہارڈویئر کو ضم کرنے کے لیے ٹائمنگ، ریئل ٹائم رکاوٹوں، اور حفاظت پر محتاط غور کی ضرورت ہوتی ہے۔ ROS 2 ایکو سسٹم اعلیٰ سطحی پلاننگ اور نچلی سطح کے ہارڈویئر کنٹرول کے درمیان فاصلے کو ختم کرنے کے لیے کئی میکانزم فراہم کرتا ہے۔

ROS 2 میں Hardware Abstraction Layer (HAL) عام طور پر ros2_control کا استعمال کرتے ہوئے نافذ کیا جاتا ہے، جو ROS 2 کو ہارڈویئر سے جوڑنے کے لیے ایک لچکدار فریم ورک ہے۔

```python
# مثال 2: سادہ Service/Client عملدرآمد
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'واپس کر رہا ہے: {response.sum}')
        return response


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('سروس دستیاب نہیں، دوبارہ انتظار کر رہا ہے...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_service = MinimalService()
    minimal_client = MinimalClient()

    # ایک درخواست بھیجیں
    response = minimal_client.send_request(1, 2)
    minimal_service.get_logger().info(f'add_two_ints کا نتیجہ: {response.sum}')

    minimal_service.destroy_node()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ service/client مثال دکھاتی ہے کہ ROS 2 کیسے synchronous request/reply کمیونیکیشن کو سنبھالتا ہے، جو ہارڈویئر کنٹرول کے منظرناموں کے لیے ضروری ہے۔

## عملدرآمد کا نقطہ نظر

فزیکل AI ایکو سسٹم ROS 2 سے آگے بڑھ کر simulation ماحول، perception سسٹمز، planning الگورتھمز، اور control فریم ورکس کو شامل کرتا ہے۔ جدید روبوٹک سسٹمز اکثر پیچیدہ رویوں کو حاصل کرنے کے لیے متعدد ٹیکنالوجیز کو ملاتے ہیں۔

```python
# مثال 3: ROS 2 میں پیرامیٹر مینجمنٹ
import rclpy
from rclpy.node import Node


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # ڈیفالٹ ویلیوز اور وضاحتوں کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('robot_name', 'turtlebot4')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('safety_distance', 0.3)
        self.declare_parameter('control_frequency', 50)

        # پیرامیٹر ویلیوز حاصل کریں
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # پیرامیٹر callback سیٹ اپ کریں
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'روبوٹ شروع ہوا: {self.robot_name}')
        self.get_logger().info(f'زیادہ سے زیادہ رفتار: {self.max_velocity} m/s')
        self.get_logger().info(f'حفاظتی فاصلہ: {self.safety_distance} m')
        self.get_logger().info(f'کنٹرول فریکوئنسی: {self.control_frequency} Hz')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value > 2.0:
                self.get_logger().warn(f'رفتار کی حد سے تجاوز: {param.value}')
                return False  # پیرامیٹر تبدیلی مسترد کریں
        return True  # پیرامیٹر تبدیلی قبول کریں


def main():
    rclpy.init()
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

یہ پیرامیٹر مینجمنٹ مثال ظاہر کرتی ہے کہ ROS 2 کیسے روبوٹ سسٹمز کی runtime configuration کو ممکن بناتا ہے۔

## عام غلطیاں

1. **ٹائمنگ کے مسائل**: نیٹ ورک لیٹنسی اور میسج پروسیسنگ تاخیر کا حساب نہ رکھنا ریئل ٹائم سسٹمز میں synchronization مسائل کا باعث بن سکتا ہے۔

2. **ریسورس مینجمنٹ**: میموری اور CPU ریسورسز کا صحیح انتظام نہ کرنا پیچیدہ روبوٹک سسٹمز میں کارکردگی میں کمی کا باعث بن سکتا ہے۔

3. **سیکیورٹی کی نظراندازی**: ROS 2 کمیونیکیشنز میں مناسب تصدیق اور encryption نافذ کرنے میں کوتاہی سسٹمز کو ممکنہ حملوں کے سامنے لا سکتی ہے۔

4. **نوڈ لائف سائیکل**: نوڈ startup اور shutdown کا غلط انتظام غیر مستحکم سسٹم حالتوں کا باعث بن سکتا ہے۔

## حقیقی دنیا کی ایپلیکیشنز

فزیکل AI ایکو سسٹم پہلے سے ہی متعدد صنعتوں کو تبدیل کر رہا ہے۔ گودام آٹومیشن میں، Amazon جیسی کمپنیاں پیکج ترتیب اور ڈیلیوری کے لیے ہزاروں روبوٹس کو مربوط کرنے کے لیے ROS پر مبنی سسٹمز استعمال کرتی ہیں۔ زراعت میں، ROS پر مبنی perception سسٹمز سے لیس خودمختار ٹریکٹرز کم سے کم انسانی مداخلت کے ساتھ فصلیں لگا سکتے، نگرانی کر سکتے اور کاٹ سکتے ہیں۔ صحت کی دیکھ بھال میں، ROS پر مبنی سرجیکل روبوٹس بہتر حفاظت اور درستگی کے ساتھ عین مطابق آپریشنز کو ممکن بناتے ہیں۔

## مشقیں

1. نیٹ ورک disconnections کے لیے error handling شامل کرنے کے لیے publisher/subscriber مثال میں ترمیم کریں۔
2. روبوٹ کے لیے پیرامیٹر configuration فائل بنائیں اور ایک نوڈ نافذ کریں جو startup پر یہ پیرامیٹرز لوڈ کرے۔

### حل

1. Error handling حل میں مضبوط نیٹ ورک کمیونیکیشن کے لیے connection callbacks اور retry میکانزم نافذ کرنا شامل ہوگا۔
2. پیرامیٹر configuration میں روبوٹ پیرامیٹرز کے ساتھ YAML فائل بنانا اور ROS 2 کی پیرامیٹر لوڈنگ صلاحیتوں کا استعمال کرنا شامل ہوگا۔

## اہم نکات

- ROS 2 روبوٹ کمیونیکیشن اور کوآرڈینیشن کے لیے ایک مضبوط middleware فراہم کرتا ہے
- distributed روبوٹ سسٹمز کے لیے مناسب کمیونیکیشن پیٹرنز ضروری ہیں
- ہارڈویئر انضمام کے لیے ٹائمنگ اور ریئل ٹائم رکاوٹوں پر محتاط توجہ کی ضرورت ہے
- پیرامیٹر مینجمنٹ لچکدار اور قابل ترتیب روبوٹ سسٹمز کو ممکن بناتا ہے
- فزیکل AI سسٹمز میں سیکیورٹی اور حفاظت کے تحفظات سب سے اہم ہیں

## مزید پڑھائی

- ROS 2 دستاویزات: https://docs.ros.org/en/humble/
- DDS Specification: https://www.omg.org/spec/DDS/
- روبوٹکس کے لیے ریئل ٹائم سسٹمز: ریئل ٹائم ROS نفاذ پر تحقیقی مقالے

## اگلے باب کا پیش نظارہ

باب 3 ROS 2 آرکیٹیکچر میں گہرائی میں جائے گا، نوڈ lifecycle مینجمنٹ، Quality of Service پالیسیز، اور جدید کمیونیکیشن پیٹرنز کو تلاش کرے گا جو مضبوط روبوٹک سسٹمز کو ممکن بناتے ہیں۔
