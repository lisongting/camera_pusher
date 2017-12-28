#include <stdlib.h>  
#include <iostream>
#include <stdio.h>  
#include <string>  
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2/OpenNI.h> 
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

using namespace std;  
using namespace cv;
using namespace openni;

void startOpenni();
void initializeFFmpeg();
void send_mat_to_server(cv::Mat * mat);
void checkOpenNIError( Status result, string status ) ;
void checkFFmpegCall(char* info ,int ret);
void signal_handler(int s);

//global member variables
int64_t frameCount = 0;
int64_t start_time;
int t;
int hz = 25;
AVCodec *encoder = NULL;
AVCodecContext *codecCtx = NULL;
AVStream *outStream = NULL;
AVOutputFormat* outputfmt = NULL;
AVFormatContext* format_ctx =NULL;
AVIOContext* ioContext = NULL;
// declare and open device,such as Kinect and ASUS xtion pro
openni::Device devAnyDevice;
openni::VideoStream streamColor;
const char * rtmp_addr_rgb = "rtmp://localhost/rgb";

int main( int argc, char** argv ){
    initializeFFmpeg();
    startOpenni();
    av_write_trailer(format_ctx);  
    avformat_close_input(&format_ctx);
    avformat_free_context(format_ctx);
    return 0;
}

//initilaze FFmpeg to handle video stream and encoding 
void initializeFFmpeg(){
    av_register_all();
    t = avformat_network_init();
    checkFFmpegCall("netwotk init",t);

    encoder = avcodec_find_encoder(AV_CODEC_ID_H264);

    if(!encoder){
        checkFFmpegCall("find avcodec", -1);
    }else{
        checkFFmpegCall("find avcodec", 1);
    }
    codecCtx = avcodec_alloc_context3(encoder);

    codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    codecCtx->codec_id = AV_CODEC_ID_H264;
    codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    codecCtx->framerate = (AVRational){hz,1};
    codecCtx->bit_rate = 400000;//bit rate 400kbps
    codecCtx->width = 640;
    codecCtx->height = 480;
    codecCtx->gop_size = 20;
    codecCtx->qmin = 2;
    codecCtx->qmax = 30;
    codecCtx->max_b_frames =0;
    codecCtx->codec = encoder;

    //set h264 options
    t = av_opt_set(codecCtx->priv_data,"preset","ultrafast",0);
    checkFFmpegCall("set preset policy to ultrafast ",t);
    t = av_opt_set(codecCtx->priv_data,"tune","zerolatency",0);
    checkFFmpegCall("set tune policy to zerolatency ",t);
    t = av_opt_set(codecCtx->priv_data,"profile","baseline",0);
    checkFFmpegCall("set profile policy to baseline",t);

    t = avcodec_open2(codecCtx,encoder,NULL);
    checkFFmpegCall("open avcodec",t);

    avformat_alloc_output_context2(&format_ctx,NULL,"flv",rtmp_addr_rgb);
    if(!format_ctx){
        cout<<"Could not create output format context"<<endl;
    }
    format_ctx->oformat->flags = AVFMT_NOTIMESTAMPS  ;
    outputfmt = format_ctx->oformat;
    if(!outputfmt){
        cout<<"Could not create OutputFormat"<<endl;
    }

    outStream = avformat_new_stream(format_ctx, encoder);
    if(!outStream){
        cout<<"Could not create output stream "<<endl;
        return ;
    }

    outStream->time_base = (AVRational){1,10000};
    outStream->index = 0;
    outStream->codec = codecCtx;

    if (outputfmt->flags & AVFMT_GLOBALHEADER) {
        outStream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;  
    }
    format_ctx->nb_streams = 1;
    format_ctx->streams[0] = outStream;
    memcpy(format_ctx->filename,rtmp_addr_rgb,sizeof(rtmp_addr_rgb));

    cout<<"---------------Stream Info----------------------"<<endl;
    av_dump_format(format_ctx,0,rtmp_addr_rgb,1);
    cout<<"------------------------------------------------"<<endl;

    //Open output URL  
    if (!(outputfmt->flags & AVFMT_NOFILE)) {  
        t = avio_open(&format_ctx->pb, rtmp_addr_rgb, AVIO_FLAG_WRITE);  
        checkFFmpegCall("open output  URL ", t);
    }  

    //Write file header 
    t = avformat_write_header(format_ctx, NULL); 
    
    checkFFmpegCall("write output header ",t); 
}


//start Openni to read image data from camera device
void startOpenni(){
    // initialize OpenNI environment
    Status result;
    result = openni::OpenNI::initialize();
    checkOpenNIError(result ,"initialize openni  ");

    
    result =  devAnyDevice.open(openni::ANY_DEVICE );
    checkOpenNIError(result ,"open camera device ");

    //create RGB camera stream 
    streamColor.create( devAnyDevice, openni::SENSOR_COLOR );
    
    // set RGB-Image mode
    openni::VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 30 );
    mModeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );

    streamColor.setVideoMode( mModeColor);
        
    // open depth stream and color stream 
    Status colorStatus = streamColor.start();
    checkOpenNIError(colorStatus,"start color stream ");
    // create  OpenCV image window 
    
    openni::VideoFrameRef  frameColor;
     
    int frameCount =0;
    start_time = av_gettime();
    signal(SIGINT,signal_handler);
    //read data stream periodically and save frame data info VideoFrameRef 
    while( 1){
        frameCount++;
        if(streamColor.isValid()){
            if(streamColor.readFrame( &frameColor )==STATUS_OK){
                    cout<<"reading color frame : "<<frameCount <<endl;
    
                    //transform VideoFrameRef to cv::Mat
                    const cv::Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
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

                    //send image to rtmp server 
                    send_mat_to_server(&imageBGRDest);
            }else{
                cout<<"read color stream error"<<endl;
            }
        }
        if( cv::waitKey(1) == 'q')
            break;
    }

}

void send_mat_to_server(cv::Mat * mat){
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
    // cout<<"numBytesYUV:"<<numBytesYUV<<endl;
    uint8_t* bufferYUV=(uint8_t *)av_malloc(numBytesYUV*sizeof(uint8_t));
    avpicture_fill((AVPicture *)pFrameYUV, bufferYUV, AV_PIX_FMT_YUV420P,width, height);

    // Initialise Software scaling context
    SwsContext *sws_ctx = sws_getContext(width,height,
                                                        AV_PIX_FMT_BGR24,
                                                        width,height,
                                                        AV_PIX_FMT_YUV420P,
                                                        SWS_BICUBIC,
                                                        NULL,NULL,NULL);
    
    pFrameYUV->width = width;
    pFrameYUV->height = height;
    pFrameYUV->format = AV_PIX_FMT_YUV420P;
    pFrameYUV->pts = frameCount;
    // Convert the image from  BGR to YUV
    sws_scale(sws_ctx, (uint8_t const * const *)pFrameBGR->data,
                        pFrameBGR->linesize, 0, height,
                        pFrameYUV->data, pFrameYUV->linesize);

    //YUV420      a UV is shared by four  Y 
    // Trying to see the different planes of YUV
    Mat MY = Mat(height, width, CV_8UC1);
    memcpy(MY.data,pFrameYUV->data[0], height*width);

    t = avcodec_encode_video2(format_ctx->streams[0]->codec,pkt,pFrameYUV,&got_pkt);
    checkFFmpegCall("encode to video frame ",t);
    if(got_pkt==1){
        cout<<"Successfully encoded frame number :"<<frameCount<<endl;
        
        pkt->stream_index = 0;
        AVRational time_base = outStream->time_base;  //
        AVRational framerate = codecCtx->framerate; //FPS   
        AVRational time_base_q = {1,AV_TIME_BASE};  //AV_TIME_BASE   1000000

        //duration between 2 frame
        int64_t interval_duration = (double)(AV_TIME_BASE)*(1 / av_q2d(framerate));;//40000

        //cout<<"interval_duration  :"<<interval_duration<<endl;
        //change ffmpeg  internal  time to  the outputstream time  
        pkt->pts = av_rescale_q(frameCount *interval_duration, time_base_q, time_base);  
        pkt->dts = pkt->pts;
        pkt->duration = av_rescale_q(interval_duration, time_base_q, time_base); 
        pkt->pos = -1;

        //Delay
        int64_t pts_time = av_rescale_q(pkt->pts, time_base, time_base_q);
        int64_t now_time = av_gettime() - start_time;
        //cout<<"pts_time :"<<pts_time<<"  now_time: "<<now_time<<"   start_time: "<<start_time<<endl;
        if (pts_time > now_time){
            //cout<<"delay :"<<pts_time-now_time<<endl;
            av_usleep(pts_time - now_time);
        }
        //cout<<"pkt   pts:"<<pkt->pts<<"  dts:"<<pkt->dts<<"  duration:"<<pkt->duration<<endl;
        // t = av_interleaved_write_frame(format_ctx, pkt);
        t  = av_write_frame(format_ctx,pkt);
        checkFFmpegCall("write frame to output URL ",t);

        frameCount++;
    }

    av_free(bufferYUV);
    av_frame_free(&pFrameBGR);
    av_frame_free(&pFrameYUV);
    av_free_packet(pkt);
    sws_freeContext(sws_ctx);
}

void signal_handler(int s){
    string str = "\ncaught interrupt signal , program exited";
    cout<<str<<endl;
     //close tream and close device 
    streamColor.destroy();
    devAnyDevice.close();
    openni::OpenNI::shutdown();

    av_write_trailer(format_ctx);  
    avformat_free_context(format_ctx);
    exit(1);
}


void checkOpenNIError( Status result, string status )  {   
    if( result != STATUS_OK ) {
        cerr << "[Error]   --  "<<  status <<endl;
        cout<<OpenNI::getExtendedError() << endl;  
        exit(0);
    }
    cout<<"[OK]      --  "<<status<<endl;
}  

void checkFFmpegCall(char* info ,int ret){
    if(ret <0){
        cerr<< "[Error]   --  "<<info<<endl;
        exit(0);
    }else{
        cout<<"[OK]      --  "<<info<<endl;
    }
}