gimbal_test: gimbal_test.cpp gimbal_control.cpp 
	g++ gimbal_test.cpp gimbal_control.cpp -o gimbal_test -I /usr/local/include -I /usr/local/include/opencv \
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

clean:
	rm gimbal_test