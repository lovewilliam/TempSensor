#!/bin/sh

home="/home/server"

TEMP_DATABASE="$home/temperatures.rrd"

rrdtool_bin=`which rrdtool`
if [ ! -x "$rrdtool_bin" ]; then
  echo "Cannot find rrdtool executable. Exiting."
  exit 1
fi

# Calculate parameters for the database.
interval=300 # 300 secs = 5 minutes
timeout=`echo "scale=0; (2*$interval + 0.2*$interval)/1" | bc -l`
mintemp=-25.0
maxtemp=50.0
minhu=20
maxhu=95
avgnumsample=12
xfactor=0.5
avgnumdays=1825 # 5 years
realnumdays=365 # 1 year
secondsperday=`echo "scale=0; 60*60*24" | bc -l`
avgnumsamples=`echo "scale=0; ($avgnumdays * ($secondsperday / ($interval * $avgnumsample)))/1" | bc -l`
realnumsamples=`echo "scale=0; ($realnumdays * ($secondsperday / ($interval * 1)))/1" | bc -l`

echo "Creating database $TEMP_DATABASE with parameters"
echo "- measurement interval: $interval s"
echo "- timeout: $timeout s"
echo "- mintemp: $mintemp, maxtemp: $maxtemp"
echo "- min/max average sample: $avgnumsample samples per data entry."
echo "- will store $avgnumdays days of average samples ($avgnumsamples samples)"
echo "- will store $realnumdays days of real samples ($realnumsamples samples)"


# see also: http://www.cuddletech.com/articles/rrd/ar01s02.html
rrdtool create $TEMP_DATABASE \
  --start N --step $interval \
  DS:temperature:GAUGE:$timeout:$mintemp:$maxtemp \
  DS:humidity:GAUGE:$timeout:$minhu:$maxhu\
  RRA:MIN:$xfactor:$avgnumsample:$avgnumsamples \
  RRA:MAX:$xfactor:$avgnumsample:$avgnumsamples \
  RRA:AVERAGE:$xfactor:1:$realnumsamples

