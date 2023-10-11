#include <sstream>

#include <stdio.h>
#include <stdlib.h>

#include <time.h>
//#include <sys/time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/fb.h>
#include <syslog.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_port.h"
#include "interface/mmal/mmal_pool.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include <cairo/cairo.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/core/core_c.h"
#include "opencv2/videoio/legacy/constants_c.h"
#include "opencv2/highgui/highgui_c.h"

extern "C" {

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/avutil.h"
#include "libavutil/mathematics.h"
#include "libavformat/avio.h"
#include "libavutil/common.h"
#include "libavutil/log.h"
#include "libavformat/version.h"

}

using namespace std;
using namespace cv;

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

#define VIDEO_FPS 15
#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 960

typedef struct {
    int width;
    int height;
    MMAL_COMPONENT_T *camera;
    MMAL_COMPONENT_T *encoder;
    MMAL_COMPONENT_T *preview;
    MMAL_PORT_T *camera_preview_port;
    MMAL_PORT_T *camera_video_port;
    MMAL_PORT_T *camera_still_port;
    MMAL_POOL_T *camera_video_port_pool;
    MMAL_PORT_T *encoder_input_port;
    MMAL_POOL_T *encoder_input_pool;
    MMAL_PORT_T *encoder_output_port;
    MMAL_POOL_T *encoder_output_pool;
    VCOS_SEMAPHORE_T semaphore;
    uint8_t *overlay_buffer;
    uint8_t *overlay_buffer2;
    int overlay;
    float fps;
    int detect = 0;

} PORT_USERDATA;

typedef struct
{
   signed char x_vector;
   signed char y_vector;
   short sad;
} INLINE_MOTION_VECTOR;

int there_is_motion = 150;
int exceeding = 250;
int count_exceeding=0;
int number_of_sequence = 0;
int framecount = -1;


AVCodec *outCodec;
AVStream *outputStream;
AVFormatContext *outputFormatCtx;
AVOutputFormat *outputFormat;
AVRational omxtimebase = { 1, VIDEO_FPS};

uint8_t* config_frame;
int config_frame_size=0;

bool AlertSendVideo = false;

const char DirSave[] = "/srv/http/pics/";
#define MOTION_PATH_ALARM "/Motion/send.sh"
#define MOTION_EXEC_ALARM "send.sh"

#define MOTION_PATH_VIDEO "/Motion/sendvideo.sh"
#define MOTION_EXEC_VIDEO "sendvideo.sh"


char timestrtemp[80];

void conv_yuv420_to_mat(Mat &dst, unsigned char* pYUV420, int width, int height)
{
    if (!pYUV420) {
	return;
    }
 
    IplImage *yuvimage, *rgbimg, *yimg, *uimg, *vimg, *uuimg, *vvimg;
 
    int nWidth = width;
    int nHeight = height;
    rgbimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 3);
    yuvimage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 3);
 
    yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
    uimg = cvCreateImageHeader(cvSize(nWidth / 2, nHeight / 2), IPL_DEPTH_8U, 1);
    vimg = cvCreateImageHeader(cvSize(nWidth / 2, nHeight / 2), IPL_DEPTH_8U, 1);
 
    uuimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
    vvimg = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
 
    cvSetData(yimg, pYUV420, nWidth);
    cvSetData(uimg, pYUV420 + nWidth * nHeight, nWidth / 2);
    cvSetData(vimg, pYUV420 + long(nWidth*nHeight*1.25), nWidth / 2);
    cvResize(uimg, uuimg, CV_INTER_LINEAR);
    cvResize(vimg, vvimg, CV_INTER_LINEAR);
 
    cvMerge(yimg, uuimg, vvimg, NULL, yuvimage);
    cvCvtColor(yuvimage, rgbimg, CV_YCrCb2RGB);
 
    cvReleaseImage(&uuimg);
    cvReleaseImage(&vvimg);
    cvReleaseImageHeader(&yimg);
    cvReleaseImageHeader(&uimg);
    cvReleaseImageHeader(&vimg);
 
    cvReleaseImage(&yuvimage);
 
    //dst = Mat(*rgbimg,int(1));
    dst = cvarrToMat(rgbimg,true);
    //rgbimg->
    cvReleaseImage(&rgbimg);
}



void saveImg(unsigned char* image, const char* index, bool AlarmSend ) {
    char filesave[250], filesave_small[250];
    Mat motion;

   snprintf(filesave,250,"%s%s%s.jpg",DirSave,timestrtemp,index);  
   conv_yuv420_to_mat(motion, image, VIDEO_WIDTH, VIDEO_HEIGHT);
   imwrite(filesave, motion);
   resize(motion,motion,Size(),0.3,0.3);
   snprintf(filesave_small,250,"%s%s%s-small.jpg",DirSave,timestrtemp,index);
   imwrite(filesave_small, motion);

    if ( AlarmSend ) {
      signal(SIGCHLD, SIG_IGN);
      int pid = fork();
      if(!pid){ execlp(MOTION_PATH_ALARM,MOTION_EXEC_ALARM,filesave,NULL );  }
    }

}

static void Init_MP4(){

    char filesave[250];
    snprintf(filesave,250,"%s%s.mp4",DirSave,timestrtemp);  

    outputFormat = av_guess_format("mp4",filesave,NULL);
        if(!outputFormat)
          fprintf(stderr,"ERROR av guess format\n");

        if(avformat_alloc_output_context2(&outputFormatCtx,outputFormat,NULL,filesave) < 0 ){
          fprintf(stderr,"Error avformat_alloc_output_context2 \n");
        }

        if(!outputFormatCtx){
          fprintf(stderr,"Error alloc output 2 \n");
        }
        outCodec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if( !outCodec ){
          fprintf(stderr,"Error avcodec_find_encoder \n");
        }

        outputStream = avformat_new_stream(outputFormatCtx,outCodec);
          if(!outputStream){
          fprintf(stderr,"Error outputStream\n");
        }
        
        outputStream->codecpar->codec_id = AV_CODEC_ID_H264;
        outputStream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        outputStream->codecpar->width = VIDEO_WIDTH;
        outputStream->codecpar->height = VIDEO_HEIGHT;
        outputStream->codecpar->format = AV_PIX_FMT_YUV420P; //AV_PIX_FMT_BGR24; //AV_PIX_FMT_YUV420P;
        outputStream->time_base = omxtimebase;
        
        
        if ( !(outputFormatCtx->flags & AVFMT_NOFILE) )
          if( avio_open2(&outputFormatCtx->pb , filesave , AVIO_FLAG_WRITE ,NULL, NULL) < 0 ){
            fprintf(stderr,"Error avio_open2");
          }

        if(avformat_write_header(outputFormatCtx , NULL) < 0){
            fprintf(stderr,"Error avformat_write_header");
        }
    
        av_dump_format(outputFormatCtx , 0 ,filesave ,1);


	AVPacket opkt;
        av_init_packet(&opkt);

        opkt.data=(uint8_t* )av_mallocz(config_frame_size);
        memcpy(opkt.data,config_frame,config_frame_size);
        opkt.size=config_frame_size;

        AVRational time_base = outputStream->time_base;
        opkt.pts = opkt.dts = 0;
        opkt.pts = av_rescale_q(opkt.pts, omxtimebase, time_base);
        opkt.dts = av_rescale_q(opkt.dts, omxtimebase, time_base);
        opkt.duration = av_rescale_q(1, omxtimebase, time_base);
        int ret = av_write_frame(outputFormatCtx, &opkt);
        if(ret < 0)  fprintf(stderr, "Error write frame");

}	

static void Save_MP4(MMAL_BUFFER_HEADER_T  *buff, int timeStampValue){
    AVPacket opkt;
        av_init_packet(&opkt);

        opkt.data=(uint8_t* )av_mallocz(buff->length);
        memcpy(opkt.data,buff->data,buff->length);
        opkt.size=buff->length;
        
        AVRational time_base = outputStream->time_base;
        opkt.pts = opkt.dts = timeStampValue;
        opkt.pts = av_rescale_q(opkt.pts, omxtimebase, time_base);
        opkt.dts = av_rescale_q(opkt.dts, omxtimebase, time_base);
        opkt.duration = av_rescale_q(1, omxtimebase, time_base);
        int ret = av_write_frame(outputFormatCtx, &opkt);
        if(ret < 0)  fprintf(stderr, "Error write frame");
}

static void Close_MP4(){

    char filesave[250];
    snprintf(filesave,250,"%s%s.mp4",DirSave,timestrtemp);  


    int ret = av_write_trailer(outputFormatCtx);
    if(ret < 0 )
	fprintf(stderr,"Error write trailer");

    if ( AlertSendVideo ) {
      signal(SIGCHLD, SIG_IGN);
      int pid = fork();
      if(!pid){ execlp(MOTION_PATH_VIDEO,MOTION_EXEC_VIDEO,filesave,NULL );  }
      AlertSendVideo = false;
    }

}

void camera_video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    static int frame_count = 0;
    static struct timespec t1;
    struct timespec t2;
    uint8_t *local_overlay_buffer;

    time_t seconds;
    struct tm * timeinfo;

    if (frame_count == 0) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    }
    clock_gettime(CLOCK_MONOTONIC, &t2);

    int d = t2.tv_sec - t1.tv_sec;

    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;
    MMAL_BUFFER_HEADER_T *output_buffer = 0;
    MMAL_POOL_T *pool = userdata->camera_video_port_pool;

    frame_count++;

    if (userdata->overlay == 0) {
        local_overlay_buffer = userdata->overlay_buffer;
    }
    else {
        local_overlay_buffer = userdata->overlay_buffer2;
    }

    int chrominance_offset = userdata->width * userdata->height;
    int v_offset = chrominance_offset / 4;
    int chroma = 0;

    output_buffer = mmal_queue_get(userdata->encoder_input_pool->queue);

    if (output_buffer) {
        mmal_buffer_header_mem_lock(buffer);
        memcpy(output_buffer->data, buffer->data, buffer->length);

        for (int x = 0; x < 600; x++) {
            for (int y = 0; y < 100; y++) {
                if (local_overlay_buffer[(y * 600 + x) * 4] > 0) {
                    output_buffer->data[y * userdata->width + x ] = 0xdf;
                    chroma= y / 2 * userdata->width / 2 + x / 2 + chrominance_offset;
                    output_buffer->data[chroma] = 0x38 ;
                    output_buffer->data[chroma+v_offset] = 0xb8 ;
                }
            }
        }

        output_buffer->length = buffer->length;
        mmal_buffer_header_mem_unlock(buffer);

        vcos_semaphore_wait(&userdata->semaphore);
        if (mmal_port_send_buffer(userdata->encoder_input_port, output_buffer) != MMAL_SUCCESS) {
            fprintf(stderr, "ERROR: Unable to send buffer \n");
        }


        unsigned char* pointer = (unsigned char *)(output_buffer->data);

        if (userdata->detect>=there_is_motion) {

	    number_of_sequence++;
    	    count_exceeding=0;

            if (number_of_sequence==1) { 

		time (&seconds); timeinfo = localtime (&seconds); 
		strftime (timestrtemp,80,"%Y%m%d_%H%M%S",timeinfo);

        	Init_MP4(); framecount=2;
        	saveImg(pointer,"-f",false); 

    	    }
	} else {
    	    if ( count_exceeding>exceeding) {
        	number_of_sequence=0; count_exceeding=0;
        	framecount = -1;
                Close_MP4();
    	    } else {
        	if ( number_of_sequence > 0 )
        	    count_exceeding++;
    	    }  
	}

	if (number_of_sequence==45 && count_exceeding==0 ) {
    	    AlertSendVideo = true;
    	    saveImg(pointer,"-s",true); 
	}

    } else {
        fprintf(stderr, "ERROR: mmal_queue_get (%d)\n", output_buffer);
    }


    if (frame_count % 10 == 0) {
        clock_gettime(CLOCK_MONOTONIC, &t2);
        float d = (t2.tv_sec + t2.tv_nsec / 1000000000.0) - (t1.tv_sec + t1.tv_nsec / 1000000000.0);
        float fps = 0.0;

        if (d > 0) {
            fps = frame_count / d;
        } else {
            fps = frame_count;
        }
        userdata->fps = fps;
    }

    mmal_buffer_header_release(buffer);

    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            fprintf(stderr, "Error: Unable to return a buffer to the video port\n");
        }
    }
}

static void encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;

    mmal_buffer_header_release(buffer);
    vcos_semaphore_post(&userdata->semaphore);
}

static void encoder_output_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;
    MMAL_POOL_T *pool = userdata->encoder_output_pool;

    mmal_buffer_header_mem_lock(buffer);

    if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {

	int num_mv=buffer->length/4;
	int vcount=0;
        INLINE_MOTION_VECTOR *imv = (INLINE_MOTION_VECTOR *)(buffer->data);

	for (int i=0; i < num_mv; i++) {
    	    double magnitude = sqrt(imv[i].x_vector*imv[i].x_vector + imv[i].y_vector*imv[i].y_vector);
    	    if (magnitude > 0) vcount++;
	}
        //if ( vcount > 50 ) vcount = 50;
        userdata->detect = vcount; //(int)((userdata->detect+vcount)/2);    

    }  else {

        if ( buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
            config_frame=(uint8_t* )av_mallocz(buffer->length);
	    memcpy(config_frame,buffer->data,buffer->length);
	    config_frame_size=buffer->length;
        }

        if ( framecount >= 0 )  { 
            Save_MP4(buffer,framecount++); 
        }   

    }

    mmal_buffer_header_mem_unlock(buffer);

    mmal_buffer_header_release(buffer);

    vcos_semaphore_post(&userdata->semaphore);

    if (port->is_enabled) {
        MMAL_STATUS_T status;
        MMAL_BUFFER_HEADER_T *new_buffer;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            fprintf(stderr, "Unable to return a buffer to the video port\n");
        }
    }

}

void fill_port_buffer(MMAL_PORT_T *port, MMAL_POOL_T *pool) {
    int num = mmal_queue_length(pool->queue);

    for (int q = 0; q < num; q++) {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
        if (!buffer) {
            fprintf(stderr, "Unable to get a required buffer %d from pool queue\n", q);
        }

        if (mmal_port_send_buffer(port, buffer) != MMAL_SUCCESS) {
            fprintf(stderr, "Unable to send a buffer to port (%d)\n", q);
        }

    }
}

int setup_camera(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_ES_FORMAT_T *format;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &userdata->camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: create camera %x\n", status);
        return -1;
    }
    userdata->camera_preview_port = userdata->camera->output[MMAL_CAMERA_PREVIEW_PORT];
    userdata->camera_video_port = userdata->camera->output[MMAL_CAMERA_VIDEO_PORT];
    userdata->camera_still_port = userdata->camera->output[MMAL_CAMERA_CAPTURE_PORT];

    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
            .max_stills_w = VIDEO_WIDTH,
            .max_stills_h = VIDEO_HEIGHT,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = VIDEO_WIDTH,
            .max_preview_video_h = VIDEO_HEIGHT,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(userdata->camera->control, &cam_config.hdr);
    }

    format = userdata->camera_preview_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = VIDEO_WIDTH;
    format->es->video.height = VIDEO_HEIGHT;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = VIDEO_WIDTH;
    format->es->video.crop.height = VIDEO_HEIGHT;

    status = mmal_port_format_commit(userdata->camera_preview_port);

    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: camera viewfinder format couldn't be set\n");
        return -1;
    }

    mmal_format_copy(userdata->camera_video_port->format,userdata->camera_preview_port->format);

    format = userdata->camera_video_port->format;
    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = VIDEO_WIDTH;
    format->es->video.height = VIDEO_HEIGHT;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = VIDEO_WIDTH;
    format->es->video.crop.height = VIDEO_HEIGHT;
    format->es->video.frame_rate.num = VIDEO_FPS;
    format->es->video.frame_rate.den = 1;

    userdata->camera_video_port->buffer_size = format->es->video.width * format->es->video.height * 12 / 8;
    userdata->camera_video_port->buffer_num = 2;

    fprintf(stderr, "INFO:camera video buffer_size = %d\n", userdata->camera_video_port->buffer_size);
    fprintf(stderr, "INFO:camera video buffer_num = %d\n", userdata->camera_video_port->buffer_num);

    status = mmal_port_format_commit(userdata->camera_video_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit camera video port format (%u)\n", status);
        return -1;
    }

    userdata->camera_video_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(userdata->camera_video_port, userdata->camera_video_port->buffer_num, userdata->camera_video_port->buffer_size);
    userdata->camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;


    status = mmal_port_enable(userdata->camera_video_port, camera_video_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera video port (%u)\n", status);
        return -1;
    }

    status = mmal_component_enable(userdata->camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera (%u)\n", status);
        return -1;
    }


    fill_port_buffer(userdata->camera_video_port, userdata->camera_video_port_pool);

    if (mmal_port_parameter_set_boolean(userdata->camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("%s: Failed to start capture\n", __func__);
    }

    fprintf(stderr, "INFO: camera created\n");
    return 0;
}

int setup_encoder(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_PORT_T *preview_input_port = NULL;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &userdata->encoder);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create preview (%u)\n", status);
        return -1;
    }

    userdata->encoder_input_port = userdata->encoder->input[0];
    userdata->encoder_output_port = userdata->encoder->output[0];
    userdata->encoder_input_port = userdata->encoder_input_port;
    userdata->encoder_output_port = userdata->encoder_output_port;

    mmal_format_copy(userdata->encoder_input_port->format, userdata->camera_video_port->format);
    userdata->encoder_input_port->buffer_size = userdata->encoder_input_port->buffer_size_recommended;
    userdata->encoder_input_port->buffer_num = 1;

    userdata->encoder_output_port->buffer_size = userdata->encoder_output_port->buffer_size_recommended;
    userdata->encoder_output_port->buffer_num = 1;
    status = mmal_port_format_commit(userdata->encoder_input_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder input port format (%u)\n", status);
        return -1;
    }

    userdata->encoder_output_port->format->encoding = MMAL_ENCODING_H264;
    userdata->encoder_output_port->format->bitrate = 2000000;

    userdata->encoder_output_port->buffer_size = userdata->encoder_output_port->buffer_size_recommended;

    if (userdata->encoder_output_port->buffer_size < userdata->encoder_output_port->buffer_size_min) {
        userdata->encoder_output_port->buffer_size = userdata->encoder_output_port->buffer_size_min;
    }

    userdata->encoder_output_port->buffer_num = userdata->encoder_output_port->buffer_num_recommended;

    if (userdata->encoder_output_port->buffer_num < userdata->encoder_output_port->buffer_num_min) {
        userdata->encoder_output_port->buffer_num = userdata->encoder_output_port->buffer_num_min;
    }

    if ( mmal_port_parameter_set_boolean(userdata->encoder_output_port, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 1) != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder vector motion \n");
        return -1;
    }

    MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, 10};
    if ( mmal_port_parameter_set(userdata->encoder_output_port, &param.hdr) != MMAL_SUCCESS) {
        fprintf(stderr, "Unable to set intraperiod");
        return -1;
    }

    status = mmal_port_format_commit(userdata->encoder_output_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder output port format (%u)\n", status);
        return -1;
    }

    fprintf(stderr, " encoder input buffer_size = %d\n", userdata->encoder_input_port->buffer_size);
    fprintf(stderr, " encoder input buffer_num = %d\n", userdata->encoder_input_port->buffer_num);

    fprintf(stderr, " encoder output buffer_size = %d\n", userdata->encoder_output_port->buffer_size);
    fprintf(stderr, " encoder output buffer_num = %d\n", userdata->encoder_output_port->buffer_num);

    userdata->encoder_input_pool = (MMAL_POOL_T *) mmal_port_pool_create(userdata->encoder_input_port, userdata->encoder_input_port->buffer_num, userdata->encoder_input_port->buffer_size);
    userdata->encoder_input_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    status = mmal_port_enable(userdata->encoder_input_port, encoder_input_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder input port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO:Encoder input pool has been created\n");

    userdata->encoder_output_pool = (MMAL_POOL_T *) mmal_port_pool_create(userdata->encoder_output_port, userdata->encoder_output_port->buffer_num, userdata->encoder_output_port->buffer_size);
    userdata->encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    status = mmal_port_enable(userdata->encoder_output_port, encoder_output_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder output port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO:Encoder output pool has been created\n");    

    fill_port_buffer(userdata->encoder_output_port, userdata->encoder_output_pool);

    fprintf(stderr, "INFO:Encoder has been created\n");
    return 0;
}


int setup_preview(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_CONNECTION_T *camera_preview_connection = 0;
    MMAL_PORT_T *preview_input_port;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &userdata->preview);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create preview (%u)\n", status);
        return -1;
    }
    preview_input_port = userdata->preview->input[0];

    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;
        param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
        param.fullscreen = 1;
        status = mmal_port_parameter_set(preview_input_port, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            fprintf(stderr, "Error: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }


    status = mmal_connection_create(&camera_preview_connection, userdata->camera_preview_port, preview_input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create connection (%u)\n", status);
        return -1;
    }

    status = mmal_connection_enable(camera_preview_connection);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable connection (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO: preview created\n");
    return 0;
}

int main(int argc, char** argv) {

    PORT_USERDATA userdata;
    MMAL_STATUS_T status;


    cairo_surface_t *surface,*surface2;
    cairo_t *context,*context2;

    memset(&userdata, 0, sizeof (PORT_USERDATA));

    userdata.width = VIDEO_WIDTH;
    userdata.height = VIDEO_HEIGHT;
    userdata.fps = 0.0;
    userdata.detect = 0;

    fprintf(stderr, "VIDEO_WIDTH : %i\n", userdata.width );
    fprintf(stderr, "VIDEO_HEIGHT: %i\n", userdata.height );
    fprintf(stderr, "VIDEO_FPS   : %i\n",  VIDEO_FPS);
    fprintf(stderr, "Running...\n");

    bcm_host_init();

    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 600, 100);
    context = cairo_create(surface);
    cairo_rectangle(context, 0.0, 0.0, 600, 100);
    cairo_set_source_rgba(context, 0.0, 0.0, 0.0, 1.0);
    cairo_fill(context);

    userdata.overlay_buffer = cairo_image_surface_get_data(surface);
    userdata.overlay = 1;

    surface2 = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 600, 100);
    context2 = cairo_create(surface2);
    cairo_rectangle(context2, 0.0, 0.0, 600, 100);
    cairo_set_source_rgba(context2, 0.0, 0.0, 0.0, 1.0);
    cairo_fill(context2);

    userdata.overlay_buffer2 = cairo_image_surface_get_data(surface2);

    vcos_semaphore_create(&userdata.semaphore, "videoEncoder", 1);

    if (1 && setup_camera(&userdata) != 0) {
        fprintf(stderr, "Error: setup camera %x\n", status);
        return -1;
    }

    if (1 && setup_encoder(&userdata) != 0) {
        fprintf(stderr, "Error: setup encoder %x\n", status);
        return -1;
    }

    if (1 && setup_preview(&userdata) != 0) {
        fprintf(stderr, "Error: setup preview %x\n", status);
        return -1;
    }


    time_t seconds;
    struct tm * timeinfo;
    char time_stamp[80];
    char text[256];

    while (1) {
        time (&seconds); timeinfo = localtime (&seconds); 
        strftime (time_stamp,80,"%d/%m/%Y %H:%M:%S ",timeinfo);

        if (userdata.overlay == 1) { 
            cairo_rectangle(context, 0.0, 0.0, 600, 100);
            cairo_set_source_rgba(context, 0.0, 0.0, 0.0, 1.0);
            cairo_fill(context);
            cairo_move_to(context, 0.0, 0.0);
            cairo_set_source_rgba(context, 1.0, 1.0, 1.0, 1.0);        
            cairo_move_to(context, 0.0, 30.0);
            cairo_set_font_size(context, 20.0);
            sprintf(text, "Golden Eye - %s  %02.1f FPS %04d Vector",time_stamp, userdata.fps, userdata.detect);
            cairo_show_text(context, text);
            userdata.overlay = 0;
        }
        else {
            cairo_rectangle(context2, 0.0, 0.0, 600, 100);
            cairo_set_source_rgba(context2, 0.0, 0.0, 0.0, 1.0);
            cairo_fill(context2);
            cairo_move_to(context2, 0.0, 0.0);
            cairo_set_source_rgba(context2, 1.0, 1.0, 1.0, 1.0);        
            cairo_move_to(context2, 0.0, 30.0);
            cairo_set_font_size(context2, 20.0);
            sprintf(text, "Golden Eye - %s  %02.1f FPS %04d Vector",time_stamp, userdata.fps, userdata.detect);
            cairo_show_text(context2, text);
            userdata.overlay = 1;
        }

        usleep(30000);
    }

    vcos_semaphore_delete(&userdata.semaphore);

    return 0;
}

