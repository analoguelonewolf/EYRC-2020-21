#!/usr/bin/env python
import paho.mqtt.client as mqtt

broker_url = "broker.mqttdashboard.com"
broker_port = 1883

def on_connect(client, userdata, flags, rc):
    print("[INFO] Connected With Result Code: " + str(rc))

def on_message(client, userdata, message):
    print("--- Subscriber ---")
    print("[INFO] Topic: {}".format(message.topic) )
    print("[INFO] Message Recieved: {}".format(message.payload.decode()))
    print("------------")

sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)

sub_client.subscribe("/eyrc/vb/time", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")

