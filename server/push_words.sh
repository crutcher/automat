#!/bin/bash

shuf /usr/share/dict/words | while read word; do
  echo Pushing $word
  mosquitto_pub -t 'automat/cell/all/message' -m "{\"ascii\": \"$word\"}"
  sleep 3.5
done
