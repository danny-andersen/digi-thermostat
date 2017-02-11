#!/bin/bash
function upload_temp {
    time=$(date +%H%M)
    echo "Temperature:" $time: $temp >> $filename
    ./dropbox_uploader.sh upload $filename $filename
}

function upload_image {
  sudo fswebcam -r 1280x720 home.jpg
  ./dropbox_uploader.sh delete home.jpg
  ./dropbox_uploader.sh upload home.jpg home.jpg
}

cd "$(dirname "$0")"
rm host_list.html
wget -O host_list.html 192.168.1.254 

cp lan_devices.txt lan_devices.old
./parse_html.sh host_list.html | grep -v "^Active" | grep -v "^No" | grep -v "^Disabled" > lan_devices.txt

filename=$(date +%Y%m%d)"_device_change.txt"
sensor_file=/sys/bus/w1/devices/28-051673fdeeff/w1_slave
safeDevice="DansG3|SansMobile"
sudo chmod 666 /dev/video0

diff -q lan_devices.txt lan_devices.old
if [ $? -eq 1 ]
then
	date +%H%M > tmpdiff
	diff lan_devices.txt lan_devices.old | sed -f seddiff >> tmpdiff
	cat tmpdiff >> $filename
	./dropbox_uploader.sh upload $filename $filename
	./dropbox_uploader.sh delete lan_devices.txt
	./dropbox_uploader.sh upload lan_devices.txt lan_devices.txt
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

#Upload any motion video detected and delete file
files=$(find motion_images/ -name "*.avi" -mmin 1)
num=$(echo $files | wc -w)
if [ $num -gt 0 ]
then
	for file in $files
	do
	    ./dropbox_uploader.sh upload $file $file
	    rm $file
	done
fi
	    

#Refresh picture every 10mins
mins=$(date +%M)
istime=$((mins % 60))
if [ $istime -eq 0 ]
then
   upload_image
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

