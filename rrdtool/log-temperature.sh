#!/bin/bash
home="/home/server"
USBTEMP_CMD="usbtemp"
TEMP_DATABASE="$home/temperatures.rrd"

# No config below.
rrdtool_bin=`which rrdtool`
if [ ! -x "$rrdtool_bin" ]; then
  echo "Cannot find rrdtool executable. Exiting."
  exit 1
fi

# Global variables
temperature="UNDEF"
humidity="UNDEF"

# Query the available sensors.
query_cmd="$USBTEMP_CMD all raw "
all="UNDEF"

while [ 1 ]; do
	all=`$query_cmd 2> /dev/null`
	if [ ! -z $all ]; then
		break;
	fi
	sleep 1
done

temperature=`echo "$all" | cut -d ',' -f 1`
humidity=`echo "$all" | cut -d ',' -f 2`

query_cmd="$USBTEMP_CMD temp 2421C7030000 raw "
while [ 1 ]; do
	temperature=`$query_cmd 2> /dev/null`
	if [ ! -z $all ]; then
		break;
	fi
	sleep 1
done


#query_cmd="$USBTEMP_CMD temperature raw"
#temperature=`$query_cmd 2> /dev/null`
#query_cmd="$USBTEMP_CMD humidity raw"
#humidity=`$query_cmd 2> /dev/null`

echo "Found readings: temperature=$temperature humidity=$humidity"
# Now, log the readings to the RRDTool database.
$rrdtool_bin update $TEMP_DATABASE "N:$temperature:$humidity"

