#ifndef V4L2CAM_H
#define V4L2CAM_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
class TScamera
{
    public:
    TScamera(double fx, double fy, double cx, double cy, double xi, double lambda, double alpha, double b, double c)
        :fx_(fx),fy_(fy),cx_(cx),cy_(cy),xi_(xi),lamda_(lambda),alpha_(alpha),b_(b),c_(c){w2_ = 0.42399;}
    cv::Point3d get_unit_sphere_coordinate(cv::Point2d pixel)
    {
        double x = pixel.x - cx_;
        double y = pixel.y - cy_;
        double mx = (fy_*x - b_*y)/(fx_*fy_-b_*c_);
        double my = (-c_*x + fx_*y)/(fx_*fy_-b_*c_);
        double ksai = alpha_ / (1 - alpha_);
        double r_square = mx*mx + my*my;
        double gamma = (ksai+std::sqrt(1+(1-ksai*ksai)*r_square))/(r_square+1);
        double yita = lamda_*(gamma-ksai)+std::sqrt(((gamma-ksai)*(gamma-ksai)-1)*lamda_*lamda_+1);
        double mz = yita*(gamma-ksai);
        double mu = xi_*(mz-lamda_)+std::sqrt(xi_*xi_*((mz-lamda_)*(mz-lamda_)-1)+1);
        return cv::Point3d(mu*yita*gamma*mx, mu*yita*gamma*my, mu*(mz-lamda_) - xi_);
    }
    cv::Point2d project(cv::Point3d p)
    {
        double X = p.x;
        double Y = p.y;
        double Z = p.z;
        double d1 = std::sqrt(X*X+Y*Y+Z*Z);
      //  if(Z <= -w2_*d1) return cv::Point2d(-1, -1);
        double d2 = std::sqrt(X*X+Y*Y+std::pow(Z+xi_*d1,2));
        double d3 = std::sqrt(X*X+Y*Y+std::pow(Z+xi_*d1+lamda_*d2,2));
        double ksai = Z+xi_*d1+lamda_*d2+alpha_/(1-alpha_)*d3;
        double pixel_x = fx_ * X/ksai + b_ * Y/ksai + cx_;
        double pixel_y = c_ * X/ksai + fy_ * Y/ksai + cy_;
        return cv::Point2d(pixel_x, pixel_y);
    }
    private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double xi_;
    double lamda_;
    double alpha_;
    double b_;
    double c_;
    double w2_;
};
class V4L2Cam
{
public:
    static inline int camera_ioctl(int fd, int request, void *arg)
    {
        int r = -1;

        do
        {
            r = ioctl(fd, request, arg);
        } while (r < 0 && EINTR == errno);

        return r;
    }
    int Init(std::string device, int width, int height)
    {
        fd = open(device.data(), O_RDWR | O_NONBLOCK);
        if(fd == -1)
            return -1;
        
        //检查摄像头设备，获取信息。
        v4l2_capability cap;
        if (-1 == camera_ioctl(fd, VIDIOC_QUERYCAP, &cap))
        {
            perror("ictol cap!");
            return -1;
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            fprintf(stderr, "is no video capture device\n");
            return -1;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            fprintf(stderr, "does not support streaming i/o\n");
            return -1;
        }
        //打印摄像头相关信息
        // printf("\nVIDOOC_QUERYCAP\n");
        // printf("the camera driver is %s\n", cap.driver);
        // printf("the camera card is %s\n", cap.card);
        // printf("the camera bus info is %s\n", cap.bus_info);
        // printf("the version is %d\n", cap.version);

        //摄像头所支持的像素格式
        v4l2_fmtdesc fmtdesc;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmtdesc.index = 0;
        printf("Support format:\n");
        while (camera_ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)!= -1)
        {
            printf("\t%d.%s\n", fmtdesc.index + 1, fmtdesc.description);
            fmtdesc.index++;
        }

        //设置像素格式
        v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;

        //应用设置格式 
        if (camera_ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            printf("VIDIOC_S_FMT\n");
            return -1;
        }

        //读出设置查看是否设置格式成功
        if (camera_ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
            printf("VIDIOC_S_FMT\n");
            return -1;
        }

        printf("fmt.type:\t\t%d\n",fmt.type);
        printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
        printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
        printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
        char str[5];
        sprintf(str, "%c%c%c%c", fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        pixelformat = str;
        
        // v4l2_streamparm streamparm;
        // streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // if (camera_ioctl(fd, VIDIOC_G_PARM, &streamparm) < 0) {
        //     printf("VIDIOC_G_PARM\n");
        //     return -1;
        // }

        // streamparm.parm.capture.timeperframe.numerator = 1;
        // streamparm.parm.capture.timeperframe.denominator = 30;
        // if (camera_ioctl(fd, VIDIOC_S_PARM, &streamparm) < 0) {
        //     printf("VIDIOC_S_PARM\n");
        //     return -1;
        // }

        v4l2_control ctrl;
        ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        ctrl.value = 2;
        if(camera_ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
            printf("VIDIOC_S_CTRL");
            return -1;
        }

        WIDTH = fmt.fmt.pix.width;
        HEIGHT = fmt.fmt.pix.height;

        // printf("init camera success!\n");
        return 0;
    }

    int StartRun()
    {
        //申请内核空间
        struct v4l2_requestbuffers reqbuffer;
        reqbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        reqbuffer.count = 4; //向内核申请4个缓冲区
        reqbuffer.memory = V4L2_MEMORY_MMAP ;//映射方式
        int ret  = camera_ioctl(fd, VIDIOC_REQBUFS, &reqbuffer);
        if(ret < 0)
        {
            perror("申请队列缓冲区失败");
        }
        //把内核缓冲区队列映射到用户地址空间
        struct v4l2_buffer mapbuffer;
        //初始化type, index
        mapbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for(int i=0; i<4; i++)
        {
            mapbuffer.index = i;
            ret = camera_ioctl(fd, VIDIOC_QUERYBUF, &mapbuffer);//从内核空间中查询一个空间做映射
            if(ret < 0)
            {
                perror("查询内核空间队列失败");
            }
            mptr[i] = (unsigned char *)mmap(NULL, mapbuffer.length, PROT_READ|PROT_WRITE, 
                                                MAP_SHARED, fd, mapbuffer.m.offset);
            size[i]=mapbuffer.length;

            //入队
            ret  = camera_ioctl(fd, VIDIOC_QBUF, &mapbuffer);
            if(ret < 0)
            {
                perror("入队失败");
            }
        }
        //开始采集
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ret = camera_ioctl(fd, VIDIOC_STREAMON, &type);
        if(ret < 0)
        {
            perror("采集失败");
        }

        return 0;
    }
    int GetFrame(cv::Mat& img)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int ret = select(fd + 1, &fds, NULL, NULL, &tv);
        if(ret == -1 && (errno = EINTR)) return -1;
        
        //7.读取帧数据
        readbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ret = ioctl(fd, VIDIOC_DQBUF, &readbuffer);
        if(ret < 0)
        {
            perror("读取帧数据失败");
            return -1;
        }
        std::vector<unsigned char> data;
        unsigned char* buff = mptr[readbuffer.index];
        for(unsigned int i = 0; i < readbuffer.length; i++)
        {
            data.push_back(buff[i]);
        }
        img = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
        //再次入队
        ret = camera_ioctl(fd, VIDIOC_QBUF, &readbuffer);
        if(ret < 0)
        {
            perror("放回队列失败");
            return -1;
        }

        return 0;
    }
    int StopRun()
    {
        //8.停止采集
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        camera_ioctl(fd, VIDIOC_STREAMOFF, &type);
        //9.释放映射
        for(int i=0; i<4; i++){
            munmap(mptr[i], size[i]);
        }
        //10.关闭设备
        close(fd);

        return 0;
    }

private:
    int fd = -1;
    int WIDTH, HEIGHT;

    //V4l2相关结构体
    struct v4l2_buffer  readbuffer;
    unsigned char *mptr[4];//保存映射后用户空间的首地址
    unsigned int  size[4];
    std::string pixelformat;
};

#endif