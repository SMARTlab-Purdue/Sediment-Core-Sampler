#!/bin/bash

out_file=$1

if [ "${out_file}x" == "x" ]; then
  echo "out_file is not given "
  rosbag record -a -o ${BAGFILES}
else
  echo "output to $out_file"
  rosbag record -a -O ${BAGFILES}${out_file}
fi


