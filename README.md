This program  is used for sending live camera video to RTMP-Server,then the rtmp-stream can be decoded and played in any player which supports RTMP protocol.

It works well on camera device such as **ASUS Xtion pro** and **Kinect**. 
Please make sure the RTMP-Server have started,and the camera device is connected correctly before executing this program.

I am using nginx-rtmp server ,please make your stream server supports RTMP protocal and make your **nginx.conf** like this:

```
# ******* custom config******
rtmp {
    server {
        listen 1935;
		application rgb{
			live on;
			allow all;
		}
		application depth{
			live on;
			allow all;
		}
    }
}
# ******* custom config******
```

There are two kinds of programs :

## 1.C++ Version

First one is C++ version. The code is in /camera_pusher_program .
Compile :
```
g++ camera_pusher.cpp  -o pusher -lOpenNI2 -lavformat -lavdevice -lavcodec -lavutil -lpthread -lswscale -lavfilter -lswresample  -lz -llzma -lbz2 -lrt -ltheoraenc -ltheoradec -lx264 `pkg-config opencv --cflags --libs`
```

## 2.Ros Version 

Second one is [ROS](http://wiki.ros.org/) (Robot Operating System)  version.The code is in **/xbot2_tools**  ,you can build it by `catkin_make --pkg xbot2_tools ` . 

run:

```
rosrun xbot2_tools camera_pusher
```

You can subscribe  "/camera/streamer_info" topic to receive notifications of its working state from another ROS node.

rtmp address of RGB video --  rtmp://localhost/rgb

rtmp address of Depth video -- rtmp://localhost/depth

(You can change it freely as long as your server has been configured correctly)

