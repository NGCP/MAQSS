#!/bin/bash
DATE=$(echo `date +%y-%m-%d`)
TIME=$(echo `date +%H%M`)
# make logging directories
mkdir -p "logs/$DATE"
# return the date and time to the caller
echo $DATE $TIME
