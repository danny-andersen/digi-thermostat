#!/bin/bash
function upload_temp {
    time=$(date +%H%M)
    echo "Temperature:" $time: $temp >> $filename
    ./dropbox_uploader.sh upload $filename $filename
}

function upload_image {
  sudo fswebcam -r 1280x720 home.jpg
  ./dropbox_uploader.sh upload home.jpg home.jpg
}

#Start
cd "$(dirname "$0")"

#Check wifi up
ping -c2 192.168.1.254 > /dev/null
if [ $? == 0 ]
then
  touch wifi-up.txt
fi

#Check last time could contact wifi AP 
#If greater than 15 mins ago, restart wlan0
cnt=$(find ./ -name wifi-up.txt -mmin +15 | wc -l)
if [ ${cnt} != 0 ]
then
      sudo /sbin/ifdown --force wlan0
      sleep 10
      sudo /sbin/ifup --force wlan0
      sleep 10
      #Check if interface now back up
      ifquery --state wlan0
      if [ $? == 1 ]
      then
	  #Failed to restart interface so reboot
          sudo /sbin/shutdown -r now
      fi
fi
#Catch all - If greater than 60 mins ago, reboot
#Note that this will cause a reboot every hour if AP is down
cnt=$(find ./ -name wifi-up.txt -mmin +60 | wc -l)
if [ ${cnt} != 0 ]
then
      sudo /sbin/shutdown -r now
fi

rm host_list.html
wget -O host_list.html 192.168.1.254 

cp lan_devices.txt lan_devices.old
./parse_html.sh host_list.html | grep -v "^Active" | grep -v "^No" | grep -v "^Disabled" > lan_devices.txt

filename=$(date +%Y%m%d)"_device_change.txt"
sensor_file=/sys/bus/w1/devices/28-051673fdeeff/w1_slave
masterstation=../sketchbook/digi-thermostat/masterstation
safeDevice="DansG3|SansMobile"
sudo chmod 666 /dev/video0

diff -q lan_devices.txt lan_devices.old
if [ $? -eq 1 ]
then
	date +%H%M > tmpdiff
	diff lan_devices.txt lan_devices.old | sed -f seddiff >> tmpdiff
	cat tmpdiff >> $filename
	./dropbox_uploader.sh upload $filename $filename
	./dropbox_uploader.sh upload lan_devices.txt lan_devices.txt
fi

diff -q ${masterstation}/status.txt thermostat_status.txt
if [ $? -eq 1 ]
then
	echo "Uploading changed thermostat status"
	cp ${masterstation}/status.txt thermostat_status.txt
	./dropbox_uploader.sh upload thermostat_status.txt thermostat_status.txt
fi

./dropbox_uploader.sh download command.txt command.txt
if [ -f command.txt ]
then
    contents=$(cat command.txt)
    echo "Running command:" $contents
    if [ $contents = "temp" ];
    then
	if [ -f $sensor_file ]
	then
    	    temp=$(grep "t=" $sensor_file | awk '{print $10}' | awk -F= '{print $2}')
	    temp=$(echo "scale=2; $temp / 1000" | bc -l)
	    upload_temp
	fi
    fi
    if [ $contents = "photo" ];
    then
	upload_image
    fi
    ./dropbox_uploader.sh delete command.txt
    rm command.txt
fi

./dropbox_uploader.sh download setTemp.txt setTemp.txt
if [ -f setTemp.txt ]
then
    mv setTemp.txt $masterstation
    ./dropbox_uploader.sh delete setTemp.txt
fi

./dropbox_uploader.sh download setSchedule.txt setSchedule.txt
if [ -f setSchedule.txt ]
then
    mv setSchedule.txt $masterstation/schedule.txt
    ./dropbox_uploader.sh delete setSchedule.txt
fi

./dropbox_uploader.sh download holiday.txt holiday.txt
if [ -f holiday.txt ]
then
    mv holiday.txt $masterstation
    ./dropbox_uploader.sh delete holiday.txt
fi
	 
#Check if motion is running
sudo service motion status
if [ $? -eq 0 ]
then
    #Motion is running
    grep -qE $safeDevice lan_devices.txt  
    if [ $? -eq 0 ]
    then
	echo "Dans home - turn motion detection off"
	sudo service motion stop
    fi
else
    #Motion isnt running
    grep -qE $safeDevice lan_devices.txt  
    if [ $? -ne 0 ]
    then
	echo "Dans not home - turn motion detection on"
	sudo service motion start
    fi
fi

#Upload any motion video not uploaded and delete file
files=$(find motion_images/ -name "*.avi" -mmin +0 -size +200k)
num=$(echo $files | wc -w)
if [ $num -gt 0 ]
then
	for file in $files
	do
	    ./dropbox_uploader.sh upload $file $file
	    rm $file
	done
fi
	    
#Delete all old jpeg files
find motion_images/ -name "*.jpg" -mmin +5 -exec rm '{}' ';'
find motion_images/ -name "*.avi" -mmin +5 -size -200k -exec rm '{}' ';'

#Refresh picture every hour
mins=$(date +%M)
if [ $mins -eq 0 ]
then
   upload_image
fi

#Update weather every 30mins
istime=$((mins % 30))
if [ $istime -eq 0 ]
then
   cd ${masterstation}
   ./getBBCWeather.sh
   cd -
fi

#Check up on temperature
if [ -f $sensor_file ]
then
    temp=$(grep "t=" $sensor_file | awk '{print $10}' | awk -F= '{print $2}')
    temp=$((temp + 500))
    temp=$((temp / 1000))
    #temp=$(echo $temp '/ 1000' | bc -l)
    oldtemp=$(cat temperature.txt)
    if [ $oldtemp -ne $temp ]
    then
        echo $temp > temperature.txt
	upload_temp
    fi
fi

