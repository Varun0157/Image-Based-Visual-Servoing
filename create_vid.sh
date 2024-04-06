SRC_DIR=code/__tmp
DEST_DIR=code/vid

mkdir $SRC_DIR
ffmpeg -f image2 -i code/img/rgbimage_%d.png ./$SRC_DIR/tmp.mpg
mkdir -p $DEST_DIR
ffmpeg -i ./$SRC_DIR/tmp.mpg -acodec copy -vcodec copy -f mp4 ./$DEST_DIR/vid.mp4
rm -rf $SRC_DIR

