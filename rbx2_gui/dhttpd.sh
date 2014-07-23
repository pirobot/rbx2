#!/bin/sh 

# Run the dhttpd webserver on the defined port and set the root directory to
# the rbx2_gui directory

PORT=8181

echo 'Launching dhttpd on port' $PORT

if [ ! `pidof dhttpd` ]; then
	dhttpd -p $PORT -r `rospack find rbx2_gui`
fi
