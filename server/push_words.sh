#!/bin/bash

DELAY=2.4

while true; do
  shuf /usr/share/dict/words | while read word; do
    echo Pushing $word
    mosquitto_pub -t 'automat/cell/all/message' -m "{\"ascii\": \"$word\"}"
    sleep $DELAY
  done
done
