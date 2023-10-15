#!/usr/bin/env python
import paho.mqtt.client as mqtt
import time

broker_url = "broker.mqttdashboard.com"
broker_port = 1883
pub_message = ""
pub_topic = "/eyrc/vb/time"

def on_publish(client, userdata, mid):
    print("--- Publisher ---")
    print("[INFO] Topic: {}".format(pub_topic))
    print("[INFO] Message Published: {}".format(pub_message))
    print("------------")

pub_client = mqtt.Client()
pub_client.on_publish = on_publish
pub_client.connect(broker_url, broker_port)

while True:
    pub_message = time.asctime( time.localtime(time.time()) )
    pub_client.publish(topic=pub_topic, payload=pub_message, qos=0, retain=False)
    time.sleep(1)

print("Out of Loop. Exiting..")

