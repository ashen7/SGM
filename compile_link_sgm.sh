g++ main.cpp semi_global_matching.cpp sgm_util.cpp -std=gnu++11 -o sgm_stereo \
    -I /home/yipeng/thirdlib/glog/include -I /home/yipeng/thirdlib/gflags/include -I /home/yipeng/thirdlib/opencv/include/ \
    -L /home/yipeng/thirdlib/glog/lib -L /home/yipeng/thirdlib/gflags/lib -L /home/yipeng/thirdlib/opencv/lib/ \
    -lglog -lgflags -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_imgcodecs
