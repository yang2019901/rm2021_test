CC = g++

cflags = -std=c++11 \
-I /usr/local/include \
-I /usr/local/include/opencv \
-I /usr/local/include/opencv2 \
-L /usr/local/lib \
-l opencv_core \
-l opencv_imgproc \
-l opencv_imgcodecs \
-l opencv_video \
-l opencv_ml \
-l opencv_highgui \
-l opencv_objdetect \
-l opencv_flann \
-l opencv_imgcodecs \
-l opencv_photo \
-l opencv_videoio \
-l opencv_features2d \
-l opencv_calib3d \
-l opencv_dnn \
-l pthread
#  ^~~~~~~ that command is to link pthread library 
# linking "pthread" manually is needed to solve error about "undefined reference to symbol 'pthread_create@@GLIBC_2.2.5'"
# that error occurs because pthread is not a default library in Linux

obj = rune/runehiter/kalman.o \
rune/runehiter/RuneHiter.o \
rune/runehiter/SinePredictor.o \
rune/runehiter/sys_time.o \
rune/runehiter/tool.o 

opt = -O3

rm2021_adv: main.o $(obj)
	$(CC) main.o $(obj) -o rm2021_adv $(cflags)
	rm *.o
	rm rune/runehiter/*.o

main.o: main.cpp
	$(CC) -c main.cpp -o main.o $(cflags) $(opt)

$(obj):%.o:%.cpp
	$(CC) -c $< -o $@ $(cflags) $(opt)

# rm2021_adv: main.cpp
# 	gcc main.cpp -o rm2021_adv $(cflags)

.PHONY: clean
clean:
	rm rm2021_adv