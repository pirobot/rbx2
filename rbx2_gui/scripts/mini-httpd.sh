#!/bin/sh 

# Run the mini-httpd webserver on the port defined in the
# mini-httpd.conf file and set the document directory to the current
# directory

PID=`pidof mini-httpd`

if [ ! -z $PID ]; then
    kill $PID
fi

sleep 1

echo "Launching mini-httpd..."

/usr/sbin/mini-httpd -C `rospack find rbx2_gui`/scripts/mini-httpd.conf -d `rospack find rbx2_gui`
