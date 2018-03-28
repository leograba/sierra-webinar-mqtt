#!/usr/bin/python

import time
import serial
import pynmea2
import paho.mqtt.client as mqtt
import json
import random
import sys

# Add flag to the MQTT Client class
mqtt.Client.connected_flag = False

# Constants and configuration for AirVantage
device_serial = "TORADEXBRSYSTEM1"
host = "eu.airvantage.net"
pub_topic = "TORADEXBRSYSTEM1/messages/json"
passwd = "123456"
# Constants and configuration for GPS
ATCmdPort = "/dev/ttyUSB2"
gpsPort = "/dev/ttyUSB1"
baud = 115200
# Path for the sensors
lum_path = "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
temp_path = "/sys/devices/virtual/thermal/thermal_zone0/temp"

# Get GPS status
def getGPSStatus():
    atser.write("AT!GPSSTATUS?\r")
    atser.readline()
    atser.readline()
    atser.readline()
    str1 = atser.readline() # 4th line is last fix
    str2 = atser.readline() # 5th line is fix session active
    while "OK" not in atser.readline():
        pass # read remaining lines but do nothing

    if "Last Fix Status" in str1 and "SUCCES" in str1:
        print str1
        print str2
        return 0 # GPS ready
    else:
        if "Fix Session Status" in str2 and "ACTIVE" in str2:
            return 1 # GPS not ready
        elif "Fix Session Status" in str2 and "NONE" in str2:
            return 1 # GPS cold start
        elif "Fix Session Status" in str2 and "FAIL" in str2:
            print "Fix Session failed"
            return 2 # GPS coordinates failed

# Parse GPS latitude and longitude coordinates
def parseGPS(message, payload):
	if hasattr(message, "sentence_type"):
		if message.sentence_type == "GGA":
			#print message.sentence_type + " - lat=" + str(message.latitude) \
			#	+ " lon=" + str(message.longitude)
			payload['_LATITUDE'] = message.latitude
			payload['_LONGITUDE'] = message.longitude
			return 0 # Success
		elif message.sentence_type == "RMC" :
			#print message.sentence_type + " - lat=" + str(message.latitude) \
			#	+ " lon=" + str(message.longitude)
			payload['_LATITUDE'] = message.latitude
			payload['_LONGITUDE'] = message.longitude
			return 0 # Success
		elif message.sentence_type == "GNS" :
			#print message.sentence_type + " - lat=" + str(message.latitude) \
			#	+ " lon=" + str(message.longitude)
			payload['_LATITUDE'] = message.latitude
			payload['_LONGITUDE'] = message.longitude
			return 0 # Success
		elif message.sentence_type == "GSA" :
			return 1 # Fail
		elif message.sentence_type == "GSV" :
			return 1 # Fail
		elif message.sentence_type == "VTG" :
			return 1 # Fail
		else:
			#print message.sentence_type
			return 1 # Fail
	else:
		return 1 # Fail

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + mqtt.connack_string(rc))

    if rc == 0:
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(pub_topic)
        client.connected_flag=True

    else :
        client.connected_flag=False

# The callback for when a PUBLISH message is received from the broker.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

# The callback for when a PUBLISH message is sent to the broker.
def on_publish(client, userdata, mid):
    #print("Message sent: " + str(mid))
    pass

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed: " +str(mid))

# GPS initialization

# Open AT command serial port and configure GPS
atser = serial.Serial(ATCmdPort, baud, timeout=1)

try:
    if getGPSStatus(): # if GPS not ready
        atser.write("AT!GPSEND\r")
        while "OK" not in atser.readline():
            pass
        while "OK" not in atser.read(4096):
            print "Starting first GPS fix"
            atser.write("AT!GPSFIX=1,200,10\r")
            #time.sleep(0.5)

        count = 0
        st = getGPSStatus()
        while st:
            if st == 1:
                print "Waiting for GPS to be ready " + str(count) + "s"
                count += 5
                time.sleep(5)
            if st == 2: # fix expired
                while "OK" not in atser.read(4096):
                    print "Starting GPS fix"
                    count = 0
                    atser.write("AT!GPSFIX=1,200,10\r")
            st = getGPSStatus()
                
    print "GPS good to go!"
    atser.close()

except KeyboardInterrupt:
    print "Closing now!"
    atser.close()
    sys.exit(0)

# Open GPS serial port and start GPS
ser = serial.Serial(gpsPort, baud)
ser.write("$GPS_START\n")

streamreader = pynmea2.NMEAStreamReader(ser)

# MQTT AirVantage Initialization

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.on_subscribe = on_subscribe

client.username_pw_set(device_serial, passwd)

client.connect(host)
client.loop_start()

while not client.connected_flag: #wait in loop
    print "Waiting to connect"
    time.sleep(1)

# Main loop

payload = dict() # put data to be sent here

while True:
    try:
        # Get data from system peripherals
        isempty = []
        with open(lum_path) as lm: # Scale goes from 0 to 10
            payload['machine.luminosity'] = 10 - (10 * float(lm.readline()) / 4096)
        with open(temp_path) as tp: # CPU temperature in degree Celsius
            payload['machine.temperature'] = float(tp.readline()) / 1000
        for i in range(0,50): # discard old data
            for msg in streamreader.next():
                parseGPS(msg, payload)
        #print "The current payload is: "
        #print payload

        # Turn into a JSON encoded string
        pload_str = json.dumps(payload)

        # Publish data
        client.publish(pub_topic, pload_str, qos=2)
        time.sleep(10)

    except KeyboardInterrupt:
        print "\nDisconnecting..."
        break # Exit application

    except:
        print "Unexpected error! Keep trying!"
        print sys.exc_info()[0]
        sys.exc_clear()
        continue # Keep trying

print "Good bye!"
client.loop_stop()
client.disconnect()
ser.write("$GPS_STOP\n")
ser.close()
