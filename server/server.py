import time
import datetime

import paho.mqtt.client as mqtt 

phase_epoch = datetime.datetime.now()

def phase_offset() -> datetime.timedelta:
  return (datetime.datetime.now() - phase_epoch)


def phase_offset_millis() -> int:
  return int(phase_offset().total_seconds() * 1000)



def main():
  power_high = 0
  power_low = 15
  glitch = 64

  last_heartbeat: float = 0.0
  heartbeat_interval: float = 3

  def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that
    # if we lose the connection and reconnect
    # then subscriptions will be renewed.
    client.subscribe("#")

  def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

  sleep_time: float = 0.5

  client = mqtt.Client()

  client.on_connect = on_connect
  client.on_message = on_message

  client.connect("localhost", 1883)

  while True:
    time.sleep(sleep_time)
    now = time.time()

    client.loop()

    if last_heartbeat + heartbeat_interval < now:
      last_heartbeat = now

      power_high = int(2 * (time.time() % 125))
      power_low = int(power_high / 4.0)
      
      phase = phase_offset_millis()
      print(phase)
   
      client.publish(
        topic="automat/environment",
        payload=(
          '{"phase":%d, "power_high":%d, "power_low":%d, "glitch":%d}'
          % (phase, power_high, power_low, glitch)))


if __name__ == '__main__':
  main()

