#!/bin/bash

path=$1
shift

password=$1
shift

NODES=($( avahi-browse _arduino._tcp --resolve --parsable --terminate 2>/dev/null \
  | grep -F "=;" | cut -d';' -f8 | sort ))

for ip in ${NODES[@]}; do
  echo "pushing to $ip"
  python2 /home/$USER/.arduino15/packages/esp8266/hardware/esp8266/*/tools/espota.py \
    -i $ip -f "$path" 2> /dev/null && echo -e "Success:\t$ip" || echo -e "Fail:   \t$ip" &
done
wait

