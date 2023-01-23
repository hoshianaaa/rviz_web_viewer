sudo -A modprobe v4l2loopback video_nr=10,11,12,13
cd ~/momo-2022.4.1_ubuntu-20.04_x86_64
./momo --video-device /dev/video10 test
