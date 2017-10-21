#!/usr/bin/env bash
#rm /home/pi/multitest/MAQSS/build/*log
#log="/home/pi/multitest/MAQSS/build/log_$(date +%Y-%m-%d_%H:%M:%S)_log"
#sleep 1
#timestamp=$(/bin/date '+%d-%m-%Y_%H:%M:%S')
#log="/home/pi/multitest/MAQSS/build/log_$(/bin/date '+%d-%m-%Y_%H:%M:%S')_log"
FILE=count.log
cd /home/pi/multitest/MAQSS/build
if [ ! -e $FILE ]; then
    echo "0" > $FILE
else
    awk -F, '{$1=$1+1}1' OFS=, $FILE > tmp & mv tmp $FILE
fi
count=$( cat $FILE )
mkdir $count
cd $count
#mkdir "$timestamp"
#cd "$timestamp"
/home/pi/multitest/MAQSS/build/offboard -f /home/pi/multitest/MAQSS/InputFile &> Log &
ps
