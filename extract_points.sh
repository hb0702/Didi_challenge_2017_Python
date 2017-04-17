#! /bin/bash
# full file path
filepath=$1

scriptpath=$(readlink -f "$0")
scriptdir=$(dirname "$SCRIPT")

sudo chmod 755 $scriptdir/kor_didi_pkg/src/point_extractor.py $scriptdir/kor_didi_pkg/src/point_extractor.py

/bin/echo -e "\e[92mRunning kor_didi_pkg with $filepath\e[0m"
#run extract_points.launch
roslaunch kor_didi2.launch bag_file_path:=$filepath
