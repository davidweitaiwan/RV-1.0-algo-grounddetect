#!/bin/bash
reg='[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
printf "%s" "Network connecting..."
while ! ip addr show dev $1 | grep -P $reg &> /dev/null
do
    printf "%c" "."
done
printf "\n%s\n" "Network detected."
# Process here
