CC = g++

obj = gimbal_test.o gimbal_control.o

opt = -O1

cflags = -I /usr/local/include \
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
-l opencv_videoio 

gimbal_test: $(obj)
	$(CC) $(obj) -o gimbal_test $(cflags)
	rm *.o

$(obj):%.o:%.cpp
	$(CC) -c $< -o $@ $(cflags) $(opt)

.PHONY:clean
clean: 
	rm gimbal_test