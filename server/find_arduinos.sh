#!/bin/bash

avahi-browse _arduino._tcp --resolve --parsable --terminate 2>/dev/null \
  | grep -F "=;" | cut -d';' -f8 | sort

