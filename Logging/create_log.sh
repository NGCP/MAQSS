#!/bin/bash
DATE=$(date)
# remove special characters from $DATE
DATE=$(echo logfolder)
let length=${#DATE}
DATE=${DATE:0:length}
mkdir "$DATE"
echo $DATE