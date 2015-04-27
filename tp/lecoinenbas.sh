#!/bin/bash

IFS=$'\n'

for l in $(cat $1 | tr -s \  )
do
    i=$(echo $l | cut -d \  -f 1)
    u1=$(echo $l | cut -d \  -f 2)
    v1=$(echo $l | cut -d \  -f 3)
    u2=$(echo $l | cut -d \  -f 4)
    v2=$(echo $l | cut -d \  -f 5)

    echo -e "p1[$i].set_u($u1);\tp1[$i].set_v($v1);\tp2[$i].set_u($u2);\tp2[$i].set_v($v2);" 
done
