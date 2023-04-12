import rospy
from std_msgs.msg import String
from lora import LoRa

# Initialize ROS2 Node
rospy.init_node('ros2_lora_topic_bridge')

# Set LoRa parameters
frequency = 915.0
spreading_factor = 7
coding_rate = 5
bw = 125000
sync_word = 0x34
tx_power = 17
gpio_pins = {'dio_0': 17, 'reset': 11, 'spi': {'mosi': 10, 'miso': 9, 'sclk': 11, 'cs': 8}}

# Initialize LoRa
lora = LoRa(frequency=frequency, spreading_factor=spreading_factor, coding_rate=coding_rate,
            bw=bw, sync_word=sync_word, tx_power=tx_power, gpio_pins=gpio_pins)

# ROS2 Publisher Callback
def ros2_topic_callback(msg):
    # Convert ROS2 message to string
    data = msg.data
    # Send data over LoRa
    lora.send(data)

# ROS2 Subscriber
ros2_subscriber = rospy.Subscriber('/ros2_topic', String, ros2_topic_callback)

# LoRa Receive Callback
def lora_receive_callback(data):
    # Convert LoRa data to string
    data = data.decode('utf-8')
    # Publish data on ROS2 topic
    ros2_publisher.publish(String(data=data))

# LoRa Receiver
lora.set_receive_callback(lora_receive_callback)

# ROS2 Publisher
ros2_publisher = rospy.Publisher('/lora_topic', String, queue_size=10)

# ROS2 Spin
rospy.spin()
