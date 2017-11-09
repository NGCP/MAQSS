#!/bin/bash
DATE=$(echo `date +%y-%m-%d`)
TIME=$(echo `date +%H%M`)
let length=${#DATE}
DATE=${DATE:0:length}
# make logging directories
mkdir -p "logs/$DATE"
mkdir "logs/$DATE/$TIME"
# return the date and time to the caller
echo $DATE $TIME