#include <stdlib.h>  
#include <iostream>
#include <stdio.h>  
#include <string>  
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2/OpenNI.h> 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>

extern "C"{
#include <libavutil/mathematics.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
#include <libavutil/time.h>   
#include <libavcodec/avcodec.h>
#include <libavutil/pixfmt.h>
}


/**
    This program  is used for sending live camera video to RTMP-Server,
    then the rtmp-stream can be decoded and played in any player which supports RTMP protocol.
    It works well on camera device such as ASUS Xtion pro and Kinect. 
    Please make sure the RTMP-Server hava started,
    and the camera device is connected correctly before executing this program.

    run:
            rosrun xbot2_tools camera_pusher

    You can subscribe  "/camera/streamer_info" topic to receive notifications of its working 
    state from another ROS node.
    rtmp address of RGB video --  rtmp://localhost/rgb
    rtmp address of Depth video -- rtmp://localhost/depth
    @author  lisongting(https://github.com/lisongting)

*/

using namespace std;  
using namespace cv;
using namespace openni;

void startOpenni();
void initializeFFmpeg();
void send_mat_to_server(cv::Mat * mat,bool is_rgb);
void checkOpenNIError( Status result, char* status ) ;
void checkFFmpegError(char* info ,int ret);
void signal_handler(int s);

//global member variables
int64_t frame_count_rgb = 0;
int64_t frame_count_depth = 0;
int64_t start_time;
int t,t1,t2;
int hz = 25;
AVCodec* encoder = NULL;
AVCodecContext* codec_ctx_rgb = NULL;
AVCodecContext* codec_ctx_depth = NULL;
AVStream* out_stream_rgb = NULL;
AVStream* out_stream_depth = NULL; 
AVOutputFormat* outputfmt_rgb = NULL;
AVOutputFormat* outputfmt_depth = NULL;
AVFormatContext* format_ctx_rgb =NULL;
AVFormatContext* format_ctx_depth = NULL;
const char * rtmp_addr_rgb = "rtmp://localhost/rgb";
const char * rtmp_addr_depth="rtmp://localhost/depth";
//publish stream information to let other nodes get information about this program
ros::Publisher info_publisher;
openni::Device any_device;
openni::VideoStream video_stream_depth;
openni::VideoStream video_stream_color;

int main( int argc, char** argv ){
    ros::init(argc,argv,"camera_pusher");
    ros::NodeHandle nh("~");
    info_publisher = nh.advertise<std_msgs::String>("/camera/streamer_info",100);

    initializeFFmpeg();
    if(ros::ok()){
        startOpenni();
    }else{
        cout<<"Ros Error "<<endl;
    }
    
    return 0;
}

//initilaze FFmpeg to handle video stream and encoding 
void initializeFFmpeg(){
    av_register_all();
    t = avformat_network_init();
    checkFFmpegError("network init",t);

    encoder = avcodec_find_encoder(AV_CODEC_ID_H264);

    if(!encoder){
        checkFFmpegError("find avcodec", -1);
    }else{
        checkFFmpegError("find avcodec", 1);
    }
    codec_ctx_rgb = avcodec_alloc_context3(encoder);
    codec_ctx_rgb->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx_rgb->codec_id = AV_CODEC_ID_H264;
    codec_ctx_rgb->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx_rgb->framerate = (AVRational){hz,1};
    codec_ctx_rgb->bit_rate = 400000;//bit rate 400kbps
    codec_ctx_rgb->width = 640;
    codec_ctx_rgb->height = 480;
    codec_ctx_rgb->gop_size = 20;
    codec_ctx_rgb->qmin = 2;
    codec_ctx_rgb->qmax = 30;
    codec_ctx_rgb->max_b_frames =0;
    codec_ctx_rgb->codec = encoder;

    codec_ctx_depth = avcodec_alloc_context3(encoder);
    codec_ctx_depth->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx_depth->codec_id = AV_CODEC_ID_H264;
    codec_ctx_depth->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx_depth->framerate = (AVRational){hz,1};
    codec_ctx_depth->bit_rate = 200000;//bit rate 
    codec_ctx_depth->width = 640;
    codec_ctx_depth->height = 480;
    codec_ctx_depth->gop_size = 20;
    codec_ctx_depth->qmin = 2;
    codec_ctx_depth->qmax = 30;
    codec_ctx_depth->max_b_frames =0;
    codec_ctx_depth->codec = encoder;

    //set x264 options
    t = av_opt_set(codec_ctx_rgb->priv_data,"preset","ultrafast",0);
    checkFFmpegError("set preset policy to ultrafast ",t);
    t = av_opt_set(codec_ctx_rgb->priv_data,"tune","zerolatency",0);
    checkFFmpegError("set tune policy to zerolatency ",t);
    t = av_opt_set(codec_ctx_rgb->priv_data,"profile","baseline",0);
    checkFFmpegError("set profile policy to baseline",t);

    av_opt_set(codec_ctx_depth->priv_data,"preset","ultrafast",0);
    av_opt_set(codec_ctx_depth->priv_data,"tune","zerolatency",0);
    av_opt_set(codec_ctx_depth->priv_data,"profile","baseline",0);


    avcodec_open2(codec_ctx_rgb,encoder,NULL);
    t = avcodec_open2(codec_ctx_depth,encoder,NULL);
    checkFFmpegError("open avcodec",t);

    avformat_alloc_output_context2(&format_ctx_rgb,NULL,"flv",rtmp_addr_rgb);
    avformat_alloc_output_context2(&format_ctx_depth,NULL,"flv",rtmp_addr_depth);
    
    if(!format_ctx_rgb){
        std_msgs::String msg;
        string str = "Could not create output format context";
        cout<<str<<endl;
        msg.data = str;
        info_publisher.publish(msg);
    }
    format_ctx_rgb->oformat->flags = AVFMT_NOTIMESTAMPS  ;
    format_ctx_depth->oformat->flags = AVFMT_NOTIMESTAMPS  ;
    outputfmt_rgb = format_ctx_rgb->oformat;
    outputfmt_depth = format_ctx_depth->oformat;

    if(!outputfmt_rgb){
         std_msgs::String msg;
        string str ="Could not create OutputFormat";
        cout<<str<<endl;
        msg.data =str;
        info_publisher.publish(msg);
    }

    out_stream_rgb = avformat_new_stream(format_ctx_rgb, encoder);
    out_stream_depth = avformat_new_stream(format_ctx_depth,encoder);
    if(!out_stream_rgb){
        std_msgs::String msg;
        string str ="Could not create output stream ";
        cout<<str<<endl;
        msg.data = str;
        info_publisher.publish(msg);
        return ;
    }

    out_stream_rgb->time_base = (AVRational){1,10000};
    out_stream_rgb->index = 0;
    out_stream_rgb->codec = codec_ctx_rgb;
    out_stream_depth->time_base =  (AVRational){1,10000};
    out_stream_depth->index = 0;
    out_stream_depth->codec = codec_ctx_depth;

    if (outputfmt_rgb->flags & AVFMT_GLOBALHEADER) {
        out_stream_rgb->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;  
    }
    if(format_ctx_depth->flags&AVFMT_GLOBALHEADER){
        out_stream_depth->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;  
    }

    format_ctx_rgb->nb_streams = 1;
    format_ctx_rgb->streams[0] = out_stream_rgb;
    format_ctx_depth->nb_streams = 1;
    format_ctx_depth->streams[0] = out_stream_depth;
    memcpy(format_ctx_rgb->filename,rtmp_addr_rgb,sizeof(rtmp_addr_rgb));
    memcpy(format_ctx_depth->filename,rtmp_addr_depth,sizeof(rtmp_addr_depth));

    cout<<"---------------Stream Info----------------------"<<endl;
    av_dump_format(format_ctx_rgb,0,rtmp_addr_rgb,1);
    av_dump_format(format_ctx_depth,0,rtmp_addr_depth,1);
    cout<<"------------------------------------------------"<<endl;

    //Open output URL  
    if (!(outputfmt_rgb->flags & AVFMT_NOFILE)) {  
        t1 = avio_open(&format_ctx_rgb->pb, rtmp_addr_rgb, AVIO_FLAG_WRITE);  
        checkFFmpegError("open RGB output  URL ", t1);
    }  
    if(!(outputfmt_depth->flags & AVFMT_NOFILE)){
         t2 = avio_open(&format_ctx_depth->pb, rtmp_addr_depth, AVIO_FLAG_WRITE);  
        checkFFmpegError("open Depth output  URL ", t2);
    }

    //Write file header 
    t1 = avformat_write_header(format_ctx_rgb, NULL); 
    t2 = avformat_write_header(format_ctx_depth,NULL);
    checkFFmpegError("write RGB output header ",t1); 
    checkFFmpegError("write Depth output header ",t2); 
}


//start Openni to read image data from camera device
void startOpenni(){
    // initialize OpenNI environment
    Status result;
    result = openni::OpenNI::initialize();
    checkOpenNIError(result ,"initialize openni  ");

    // open device,such as Kinect and ASUS xtion pro
    result =  any_device.open(openni::ANY_DEVICE );
    checkOpenNIError(result ,"open camera device ");

    // create depth camera stream 
    video_stream_depth.create( any_device, openni::SENSOR_DEPTH );
    //create RGB camera stream 
    video_stream_color.create( any_device, openni::SENSOR_COLOR );

    //set Depth-Image  mode
    openni::VideoMode depth_mode;
    //image  width :640 height:480
    depth_mode.setResolution( 640, 480 );
    depth_mode.setFps( 30 );
    // pixel format
    depth_mode.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );
    video_stream_depth.setVideoMode( depth_mode);

    // set RGB-Image mode
    openni::VideoMode color_mode;
    color_mode.setResolution( 640, 480 );
    color_mode.setFps( 30 );
    color_mode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );
    video_stream_color.setVideoMode( color_mode);
    //set depth and color image registration mode
    if( any_device.isImageRegistrationModeSupported(
        openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) ){
        any_device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }
        
    // open depth stream and color stream 
    Status depthStatus = video_stream_depth.start();
    checkOpenNIError(depthStatus,"start depth stream ");
    Status colorStatus = video_stream_color.start();
    checkOpenNIError(colorStatus,"start color stream ");
    // create  OpenCV image window 
    // namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
    // namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );
    
    openni::VideoFrameRef  frame_depth;
    openni::VideoFrameRef  frame_color;
     
    frame_count_rgb =0;
    frame_count_depth = 0;
    start_time = av_gettime();

    signal(SIGINT,signal_handler);
    //read data stream periodically and save frame data info VideoFrameRef 
    while( 1){
        if(video_stream_color.isValid()){
            if(video_stream_color.readFrame( &frame_color )==STATUS_OK){    
                //transform VideoFrameRef to cv::Mat
                const cv::Mat mImageRGB(frame_color.getHeight(), frame_color.getWidth(), CV_8UC3, (void*)frame_color.getData());
                //make RGB pixel format to BGR pixel format
                cv::Mat imageBGRSrc , imageBGRDest;
                cv::cvtColor( mImageRGB, imageBGRSrc, CV_RGB2BGR );
                imageBGRDest .create(imageBGRSrc.size(),imageBGRSrc.type());

                //get the mirror image of source image 
                Mat map_x;
                Mat map_y;
                map_x.create(imageBGRSrc.size(),CV_32FC1);
                map_y.create(imageBGRSrc.size(),CV_32FC1);
                for(int i=0;i<imageBGRSrc.rows;i++){
                    for(int j=0;j<imageBGRSrc.cols;j++){
                        map_x.at<float>(i,j) =(float) (imageBGRSrc.cols-j);
                        map_y.at<float>(i,j) = (float) i;
                    }
                }
                remap(imageBGRSrc,imageBGRDest,map_x,map_y,CV_INTER_LINEAR);

                //show Image 
                //cv::imshow( "Color Image", imageBGRDest );
                //waitKey(1);
                //send image to rtmp server 
                send_mat_to_server(&imageBGRDest,true);
            }else{
                cout<<"read color stream error"<<endl;
            }
        }
        
        if(video_stream_depth.isValid()){
            Status readFrameStatus ;
            readFrameStatus = video_stream_depth.readFrame( &frame_depth );
            checkOpenNIError(readFrameStatus,"read depth frame");
            if(readFrameStatus ==STATUS_OK){
                const cv::Mat mImageDepth( frame_depth.getHeight(), frame_depth.getWidth(), CV_16UC1, (void*)frame_depth.getData());
                // change pixel format to CV_8U 
                cv::Mat mScaledDepth;
                // get max depth value 
                int iMaxDepth = video_stream_depth.getMaxPixelValue();
                mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );
                flip(mScaledDepth,mScaledDepth,1);
                  
                cv::Mat imageDepthSrc ;
                cv::cvtColor( mScaledDepth, imageDepthSrc, CV_GRAY2BGR );
                //cv::imshow( "Depth Image", imageDepthSrc );
                //cv::waitKey(1);
                send_mat_to_server(&imageDepthSrc,false);
            }else{
                cout<<"read depth stream error"<<endl;
            }
        }else{
            cout<<"depth stream is not valid"<<endl;
        }
    }


}

/**
    send  image frame to rtmp server 
    @param mat : the image data of opencv
    @param is_rgb : if this value is true,send rgb image data ,otherwise 
                               send  depth image data
*/
void send_mat_to_server(cv::Mat * mat,bool is_rgb){
    int nbytes;
    int got_pkt =0 ;
    const int strides[] = {mat->step[0]};
    AVPacket* pkt =NULL;

    pkt = av_packet_alloc();
    pkt->data = NULL;
    pkt->size = 0;
    av_init_packet(pkt);

    // Creating two frames for conversion
    AVFrame *pFrameYUV =av_frame_alloc();
    AVFrame *pFrameBGR =av_frame_alloc();
    int width = mat->cols;
    int height = mat->rows;
    pFrameBGR->width = width;
    pFrameBGR->height = height;
    
   // Assign image buffers
    avpicture_fill((AVPicture *)pFrameBGR, mat->data, AV_PIX_FMT_BGR24,width, height);

    // Determine required buffer size and allocate buffer for YUV frame 
    int numBytesYUV=av_image_get_buffer_size(AV_PIX_FMT_YUV420P, width,height,1); 
    uint8_t* bufferYUV=(uint8_t *)av_malloc(numBytesYUV*sizeof(uint8_t));
    avpicture_fill((AVPicture *)pFrameYUV, bufferYUV, AV_PIX_FMT_YUV420P,width, height);

    //Initialise Software scaling context
    SwsContext *sws_ctx = sws_getContext(width,height,
                                                        AV_PIX_FMT_BGR24,
                                                        width,height,
                                                        AV_PIX_FMT_YUV420P,
                                                        SWS_BICUBIC,
                                                        NULL,NULL,NULL);  
    if(is_rgb){
        pFrameYUV->width = width;
        pFrameYUV->height = height;
        pFrameYUV->format = AV_PIX_FMT_YUV420P;
        pFrameYUV->pts = frame_count_rgb;
    }else{
        pFrameYUV->width = width;
        pFrameYUV->height = height;
        pFrameYUV->format = AV_PIX_FMT_YUV420P;
        pFrameYUV->pts = frame_count_depth;
    }
   
    // Convert the image from  BGR to YUV
    sws_scale(sws_ctx, (uint8_t const * const *)pFrameBGR->data,
                        pFrameBGR->linesize, 0, height,
                        pFrameYUV->data, pFrameYUV->linesize);
    
    if(is_rgb){
        t1 = avcodec_encode_video2(format_ctx_rgb->streams[0]->codec,pkt,pFrameYUV,&got_pkt);
        char info[100];
        sprintf(info,"%s %d","encode RGB video frame number: ",frame_count_rgb);
        checkFFmpegError(info,t1);
     }else{
        t2 = avcodec_encode_video2(format_ctx_depth->streams[0]->codec,pkt,pFrameYUV,&got_pkt);
        char info[100];
        sprintf(info,"%s %d","encode Depth video frame number: ",frame_count_depth);
        checkFFmpegError(info,t2);
     }
   
   
    if(got_pkt==1){
        pkt->stream_index = 0;
        if(is_rgb){
            AVRational time_base = out_stream_rgb->time_base;  //
            AVRational framerate = codec_ctx_rgb->framerate; //FPS   
            AVRational time_base_q = {1,AV_TIME_BASE};  //AV_TIME_BASE   1000000

            //duration between 2 frame
            int64_t interval_duration = (double)(AV_TIME_BASE)*(1 / av_q2d(framerate));;//40000

            //change ffmpeg  internal  time to  the outputstream time  
            pkt->pts = av_rescale_q(frame_count_rgb *interval_duration, time_base_q, time_base);  
            pkt->dts = pkt->pts;
            pkt->duration = av_rescale_q(interval_duration, time_base_q, time_base); 
            pkt->pos = -1;

            //Delay
            int64_t pts_time = av_rescale_q(pkt->pts, time_base, time_base_q);
            int64_t now_time = av_gettime() - start_time;
            if (pts_time > now_time){
                av_usleep(pts_time - now_time);
            }
            t1  = av_write_frame(format_ctx_rgb,pkt);
            checkFFmpegError("write RGB frame to output URL ",t1);
            frame_count_rgb++;
        }else{
            AVRational time_base = out_stream_depth->time_base;  //
            AVRational framerate = codec_ctx_depth->framerate; //FPS   
            AVRational time_base_q = {1,AV_TIME_BASE};  //AV_TIME_BASE   1000000

            //duration between 2 frame
            int64_t interval_duration = (double)(AV_TIME_BASE)*(1 / av_q2d(framerate));;//40000

            //change ffmpeg  internal  time to  the outputstream time  
            pkt->pts = av_rescale_q(frame_count_depth *interval_duration, time_base_q, time_base);  
            pkt->dts = pkt->pts;
            pkt->duration = av_rescale_q(interval_duration, time_base_q, time_base); 
            pkt->pos = -1;

            //Delay
            int64_t pts_time = av_rescale_q(pkt->pts, time_base, time_base_q);
            int64_t now_time = av_gettime() - start_time;
            if (pts_time > now_time){
                av_usleep(pts_time - now_time);
            }
            t2  = av_write_frame(format_ctx_depth,pkt);
            checkFFmpegError("write Depth frame to output URL ",t2);
            frame_count_depth++;
        }

    }

    av_free(bufferYUV);
    av_frame_free(&pFrameBGR);
    av_frame_free(&pFrameYUV);
    av_free_packet(pkt);
    sws_freeContext(sws_ctx);
}

void signal_handler(int s){
    std_msgs::String msg ;
    string str = "caught interrupt signal , program exited";
    msg.data = str;
    info_publisher.publish(msg);

     //close tream and close device 
    video_stream_depth.destroy();
    video_stream_color.destroy();
    any_device.close();
    openni::OpenNI::shutdown();

    av_write_trailer(format_ctx_rgb);  
    avformat_close_input(&format_ctx_rgb);
    avformat_free_context(format_ctx_rgb);
    exit(1);
}

void checkOpenNIError( Status result, char* status )  {   
    std_msgs::String msg;
    string str ;
    if( result != STATUS_OK ) {
        cerr << "[Error]   --  "<<  status <<endl;
        cout<<OpenNI::getExtendedError() << endl;  
        str.append("[Error]   --  ");
        exit(0);
    }else{
         cout<<"[OK]      --  "<<status<<endl;
         str.append("[OK]      --  ");
    }
    str.append(status);
    msg.data  = str;
    info_publisher.publish(msg);
}  

void checkFFmpegError(char* info ,int ret){
    std_msgs::String msg;
    string str ;
    if(ret <0){
        cerr<< "[Error]   --  "<<info<<endl;
        str = "[Error]   --  ";
        exit(0);
    }else{
        cout<<"[OK]      --  "<<info<<endl;
        str = "[OK]      --  ";
    }
    str.append(info);
    msg.data  = str;
    info_publisher.publish(msg);
}