#!/bin/bash     #Explicitly tell shell this is a bash script

while [ true ]  #Loop forever, or until ^c
do
  tail -n 2500 capture.csv >capturetail.csv
  sleep 1         #Pause 1 second
done