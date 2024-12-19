#!/bin/bash

weather_type="snow"

root_dir=$(dirname $(cd $(dirname $0);pwd))
exec_dir="$root_dir/bin"
data_dir="$root_dir/data/$weather_type"
og_dir="$data_dir/og"
sn_dir="$data_dir/sn"
sp_dir="$data_dir/sp"
rt_dir="$data_dir/rt"
tor_ep_dir="$rt_dir/tor/ep"
tor_en_dir="$rt_dir/tor/en"
ewm_ep_dir="$rt_dir/ewm/ep"
ewm_en_dir="$rt_dir/ewm/en"

check_dir(){
  if [ ! -d $1 ]
  then
    mkdir -p $1
  fi
}

check_dir $og_dir

# TOR
check_dir $tor_ep_dir
check_dir $tor_en_dir
# EWM
check_dir $ewm_ep_dir
check_dir $ewm_en_dir

