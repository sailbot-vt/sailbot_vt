#!/bin/bash

for n in {1..120};
do 
    sleep 0.5
    sudo chmod 777 /dev/tty* 
    echo hello >> /tmp/test.txt
done
