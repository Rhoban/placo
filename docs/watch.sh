#!/bin/bash

make html
echo "Starting watching..."
killall php
cd _build/html
php -S localhost:8080 &
cd ../..

while [ true ]
do
    inotifywait *.rst dynamics/ kinematics/ basics/ --recursive 
    make html
    sleep 0.5
done
