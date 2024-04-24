#!/bin/bash

input_dir="./code/img"
output_dir="./code/vid"

mkdir -p "$output_dir"

ffmpeg -f image2 -framerate 25 -i "$input_dir/rgbimage_%d.png" -vcodec libx264 -crf 22 "$output_dir/video.mp4"
