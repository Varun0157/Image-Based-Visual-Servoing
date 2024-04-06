SRC_DIR=__tmp
DEST_DIR=vid

mkdir $SRC_DIR
ffmpeg -f image2 -i img/rgbimage_%d.png ./$SRC_DIR/tmp.mpg
mkdir -p vid
ffmpeg -i ./$SRC_DIR/tmp.mpg -acodec copy -vcodec copy -f mp4 ./vid/vid.mp4
rm -rf $SRC_DIR

