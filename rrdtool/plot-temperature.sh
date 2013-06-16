#!/bin/bash
# Check for installation directory, load configuration.

home="/home/server"

TEMP_DATABASE="$home/temperatures.rrd"
GRAPH_DIRECTORY="/var/www/localhost/htdocs/sensors/"

rrdtool_bin=`which rrdtool`
if [ ! -x "$rrdtool_bin" ]; then
  echo "Cannot find rrdtool executable. Exiting."
  exit 1
fi

draw_graph_complete_last() {
  filename=$1
  title=$2
  starttime=$3
  timestamp=`date "+%d.%m.%y %k\:%M"`
  rrdtool graph "$GRAPH_DIRECTORY/$filename" \
    -a PNG --title="$title" \
    --start "$starttime" --vertical-label "Deg Celsius" \
    "DEF:probe1=$TEMP_DATABASE:temperature:AVERAGE" \
    'LINE1:probe1#ff0000:Emb. Lab. ' \
    'GPRINT:probe1:LAST:last\: %2.1lf C' \
    'GPRINT:probe1:MIN:min\: %2.1lf C' \
    'GPRINT:probe1:MAX:max\: %2.1lf C' \
    'GPRINT:probe1:AVERAGE:avg\: %2.1lf C\j' \
    "COMMENT:generated by Phoenix-NAS on $timestamp\c"
#'AREA:probe3#cccccc:HVAC' \
}


draw_graph_complete () {
  filename=$1
  title=$2
  starttime=$3
  timestamp=`date "+%d.%m.%y %k\:%M"`
  rrdtool graph "$GRAPH_DIRECTORY/$filename" \
    -a PNG --title="$title" \
    --start "$starttime" --vertical-label "Deg Celsius" \
    "DEF:probe1=$TEMP_DATABASE:temperature:AVERAGE" \
    'LINE1:probe1#ff0000:Emb. Lab.  ' \
    'GPRINT:probe1:MIN:min\: %2.1lf C' \
    'GPRINT:probe1:MAX:max\: %2.1lf C' \
    'GPRINT:probe1:AVERAGE:avg\: %2.1lf C\j' \
    "COMMENT:generated by Phoenix-NAS on $timestamp\c"
#'AREA:probe3#cccccc:HVAC' \
}

draw_humidity_complete_last()
{
  filename=$1
  title=$2
  starttime=$3
  timestamp=`date "+%d.%m.%y %k\:%M"`
  rrdtool graph "$GRAPH_DIRECTORY/$filename" \
    -a PNG --title="$title" \
    --start "$starttime" --vertical-label "Humidity" \
    "DEF:probe1=$TEMP_DATABASE:humidity:AVERAGE" \
    'LINE1:probe1#00ff00:Emb. Lab. ' \
    'GPRINT:probe1:LAST:last\: %2.1lf' \
    'GPRINT:probe1:MIN:min\: %2.1lf' \
    'GPRINT:probe1:MAX:max\: %2.1lf' \
    'GPRINT:probe1:AVERAGE:avg\: %2.1lf\j' \
    "COMMENT:generated by Phoenix-NAS on $timestamp\c"
#'AREA:probe3#cccccc:HVAC' \
}

draw_humidity_complete()
{
  filename=$1
  title=$2
  starttime=$3
  timestamp=`date "+%d.%m.%y %k\:%M"`
  rrdtool graph "$GRAPH_DIRECTORY/$filename" \
    -a PNG --title="$title" \
    --start "$starttime" --vertical-label "Humidity" \
    "DEF:probe1=$TEMP_DATABASE:humidity:AVERAGE" \
    'LINE1:probe1#00ff00:Emb. Lab.  ' \
    'GPRINT:probe1:MIN:min\: %2.1lf' \
    'GPRINT:probe1:MAX:max\: %2.1lf' \
    'GPRINT:probe1:AVERAGE:avg\: %2.1lf\j' \
    "COMMENT:generated by Phoenix-NAS on $timestamp\c"
#'AREA:probe3#cccccc:HVAC' \

}

# commands to generate graphs follow. You can add or modify the 
# examples below. Syntax is:
# draw_graph_? <filename> <title> <time modifier> <features>
# where features is 
draw_graph_complete_last "last-24-hours.png" "Temperature - last 24 hours" "-1day"
draw_graph_complete "last-week.png" "Temperature - last week" "-1week"
draw_graph_complete "last-month.png" "Temperature - last month" "-1month"
draw_graph_complete "last-year.png" "Temperature - last year" "-1year"
draw_humidity_complete_last "last-24-hours_hu.png" "Humidity - last 24 hours" "-1day"
draw_humidity_complete "last-week_hu.png" "Humidity - last week" "-1week"
draw_humidity_complete "last-month_hu.png" "Humidity - last month" "-1month"
draw_humidity_complete "last-year_hu.png" "Humidity - last year" "-1year"
