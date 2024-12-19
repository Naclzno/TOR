#!/bin/bash

source dirs.sh
if [ $weather_type == "snow" ]; then
  # Snow
  i=1 
  for og_file in $og_dir/*.txt 
  do 
    filename=$(basename $og_file .txt)  
    ep_filename="${filename}_ep.txt"    
    en_filename="${filename}_en.txt"    
    
    echo "Processing [FILE $i: $filename] ..." 
    # echo -n "ROR used " #打印出 "ROR used "，-n 参数使得打印不会换行
    # #运行 ror 程序，该程序位于 exec_dir 目录下，其参数由当前正在处理的文件和其他几个固定的值组成。
    # $exec_dir/ror -f $og_file -ep $ror_ep_dir/$ep_filename -en $ror_en_dir/$en_filename -n 3 -r 0.6 -dt 70 -it 25
    # echo -n "SOR used "
    # $exec_dir/sor -f $og_file -ep $sor_ep_dir/$ep_filename -en $sor_en_dir/$en_filename -k 15 -m 0.9 -dt 70 -it 25
    # echo -n "DROR used "
    # # $exec_dir/dror -f $og_file -ep $dror_ep_dir/$ep_filename -en $dror_en_dir/$en_filename -n 3 -m 1.5 -a 0.2 -dt 100 -it 50 
    # $exec_dir/dror -f $og_file -ep $dror_ep_dir/$ep_filename -en $dror_en_dir/$en_filename -n 3 -m 1.75 -a 0.2 -r 0.04 -dt 100 -it 50 
    # echo -n "DSOR used "
    # $exec_dir/dsor -f $og_file -ep $dsor_ep_dir/$ep_filename -en $dsor_en_dir/$en_filename -k 12 -m 0.15 -r 0.16 -dt 100 -it 50 
    # echo -n "DDIOR used "
    # $exec_dir/ddior -f $og_file -ep $ddior_ep_dir/$ep_filename -en $ddior_en_dir/$en_filename -k 15 -a 0.2 -dt 100 -it 50 
    # echo -n "LIDSOR used "
    # $exec_dir/lidsor -f $og_file -ep $lidsor_ep_dir/$ep_filename -en $lidsor_en_dir/$en_filename -k 12 -m 0.12 -r 0.12 -d 100 -i 50 
    # echo -n "TOR used " 
    # $exec_dir/tor -f $og_file -ep $tor_ep_dir/$ep_filename -en $tor_en_dir/$en_filename -n 1 -r 0.5 -dt 60 -it 25
    echo -n "EWM used "
    $exec_dir/ewm -f $og_file -ep $ewm_ep_dir/$ep_filename -en $ewm_en_dir/$en_filename -m 2.5 -a 0.2 -r 0.12 -dt 50 -it 25 -r2 0.1 -st 0.27

    i=$(expr $i + 1)
  done 
fi

i=1
for og_file in $og_dir/*.txt
do
  filename=$(basename $og_file .txt)  
  ep_filename="${filename}_ep.txt"    
  en_filename="${filename}_en.txt"    
  echo "Processing [FILE $i: $filename] ..."
  # echo -n "[ROR] "
  # $exec_dir/eva -f $og_file -ep $ror_ep_dir/$ep_filename -en $ror_en_dir/$en_filename 
  # echo -n "[SOR] "
  # $exec_dir/eva -f $og_file -ep $sor_ep_dir/$ep_filename -en $sor_en_dir/$en_filename 
  # echo -n "[DROR] "
  # $exec_dir/eva -f $og_file -ep $dror_ep_dir/$ep_filename -en $dror_en_dir/$en_filename 
  # echo -n "[DSOR] "
  # $exec_dir/eva -f $og_file -ep $dsor_ep_dir/$ep_filename -en $dsor_en_dir/$en_filename 
  # echo -n "[DDIOR] "
  # $exec_dir/eva -f $og_file -ep $ddior_ep_dir/$ep_filename -en $ddior_en_dir/$en_filename 
  # echo -n "[LIDSOR] "
  # $exec_dir/eva -f $og_file -ep $lidsor_ep_dir/$ep_filename -en $lidsor_en_dir/$en_filename 
  # echo -n "[TOR] "
  # $exec_dir/eva -f $og_file -ep $tor_ep_dir/$ep_filename -en $tor_en_dir/$en_filename
  echo -n "[EWM] "
  $exec_dir/eva -f $og_file -ep $ewm_ep_dir/$ep_filename -en $ewm_en_dir/$en_filename   
  i=$(expr $i + 1)
done
