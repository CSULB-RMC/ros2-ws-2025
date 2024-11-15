import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client
import paho.mqtt.publish as publish
import time
from std_msgs.msg import UInt8, String, Int16, Int32, Int32MultiArray
from dotenv import find_dotenv, load_dotenv
import os
import json
import math
load_dotenv(find_dotenv())
class DB_Broker(Node):
    
    def __init__(self):
        super().__init__('db_broker_pub')
        self.can_id = 0
        self.declare_parameter('can_id', self.can_id)
        self.can_id_timer = self.create_timer(1, self.can_timer_callback)

        self.unacked_publish = set() 
        self.status_timer = self.create_timer(0.05, self.timer_callback)
        self.vesc1_sub = self.create_subscription(Int32MultiArray, 'vesc_pub', self.sensor_update, 10)
        #self.sensorData = {2319: [0, 0, 0], 3599: [0, 0, 0, 0], 3855: [0, 0, 0, 0], 4111: [0, 0, 0, 0], 6927: [0, 0, 0, 0]} 
        self.sensorData = dict()
        self.temp = 0

        self.broker = os.getenv("DATANIZ_IP")
        self.port = 1883
        self.topic = 'python/mqtt/real'
        self.username = os.getenv("DATANIZ_EMAIL")
        self.password = os.getenv("DATANIZ_KEY") 

        self.FIRST_RECONNECT_DELAY = 1
        self.RECONNECT_RATE = 2
        self.MAX_RECONNECT_COUNT = 12
        self.MAX_RECONNECT_DELAY = 60
    
    def can_timer_callback(self):
        self.can_id  = self.get_parameter('can_id').get_parameter_value().integer_value


    # def on_connect(client, userdata, flags, rc):
    # For paho-mqtt 2.0.0, you need to add the properties parameter.
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info('connecting...')
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT Broker!')
        else:
            self.get_logger().info(f'Failed to connect, return code {rc}\n')

    def on_publish(self, client, userdata, mid):
        print("mid: "+str(userdata))

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info("Disconnected with result code: %s" % rc)
        reconnect_count, reconnect_delay = 0, self.FIRST_RECONNECT_DELAY
        while reconnect_count < self.MAX_RECONNECT_COUNT:
            self.get_logger().info("Reconnecting in %d seconds..." % reconnect_delay)
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                self.get_logger().info("Reconnected successfully!")
                return
            except Exception as err:
                self.get_logger().info("%s. Reconnect failed. Retrying..." % err)

            reconnect_delay *= self.RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, self.MAX_RECONNECT_DELAY)
            reconnect_count += 1
        self.get_logger().info("Reconnect failed after %s attempts. Exiting..." % reconnect_count)
    
    def connect_mqtt(self):
        # For paho-mqtt 2.0.0, you need to set callback_api_version.
#        client = mqtt_client.Client(transport="websockets")
        client = mqtt_client.Client()
        # client = mqtt_client.Client(callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

        client.username_pw_set(self.username, self.password)
        client.on_connect = self.on_connect
        client.on_publish = self.on_publish
        client.connect(self.broker, self.port)
        return client 

    def sensor_update(self, msg):
        temp_id = "0x%0.2X" % msg.data[0]
        if int(temp_id[len(temp_id) - 2::], 16) != self.can_id:
            return
        if msg.data[0] not in self.sensorData.keys():
            self.sensorData.update({msg.data[0]: [0 for i in range(10)]})
        self.sensorData[msg.data[0]][msg.data[1]] = msg.data[2]

    def publish(self, client, topic, message, timeout):
        self.get_logger().info(f"publishing {message}")
        msg_info = client.publish(topic, json.dumps(message), qos=1)
        self.unacked_publish.add(msg_info.mid)

        # Due to race-condition described above, the following way to wait for all publish is safer
        msg_info.wait_for_publish(timeout=timeout)

    def timer_callback(self):
        #self.get_logger().info(f'{self.broker}')
        #self.get_logger().info(f'{self.sensorData}')
        client = self.connect_mqtt()
        client.loop_start()
        ASSET_UID = self.can_id
        
        message = {
                "topic": self.topic,
                "device_asset_uid": ASSET_UID,
                "rpm": self.sensorData[2304 + self.can_id][0],
                "motor current": self.sensorData[2304 + self.can_id][1]/10,
                "duty cycle": self.sensorData[2304 + self.can_id][2]/1000,
                "amp hours": self.sensorData[3584 + self.can_id][0]/(math.e**4),
                "amp hours charged": self.sensorData[3584 + self.can_id][1]/(math.e**4),
                "watt hours": self.sensorData[3840 + self.can_id][0]/(math.e**4),
                "watt hours charged": self.sensorData[3840 + self.can_id][1]/(math.e**4),
                "FET temp": self.sensorData[4096 + self.can_id][0]/10,
                "motor temp": self.sensorData[4096 + self.can_id][1]/10,
                "input current": self.sensorData[4096 + self.can_id][2]/10,
                "PID position": self.sensorData[4096 + self.can_id][3]/50,
                "tachometer": self.sensorData[6912 + self.can_id][0],
                "input voltage": self.sensorData[6912 + self.can_id][1]/(math.e**4),
            }
        self.publish(client, self.topic, message, timeout=15) 
        client.loop_stop() 

def main(args=None):
    print("DB!!!")

    rclpy.init(args=args)
    db_node = DB_Broker()


    rclpy.spin(db_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    db_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

