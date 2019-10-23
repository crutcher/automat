#!/usr/bin/env python3

import sys
import time
import socket

UDP_IP = "192.168.86.21"
UDP_PORT = 5005

def main(argv):
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((UDP_IP, UDP_PORT))

  num_frames = 10000
  print("sampling num_frames:", num_frames)

  start_time = time.time()
  for i in range(num_frames):
    data, addr = sock.recvfrom(1024)

  end_time = time.time()

  delay = end_time - start_time
  seconds_per_frame = delay / float(num_frames)
  frames_per_second = 1.0 / seconds_per_frame

  print("delay secs:", delay)
  print("secs per frame:", seconds_per_frame)
  print("frames per second:", frames_per_second)


if __name__ == '__main__':
  main(sys.argv)

