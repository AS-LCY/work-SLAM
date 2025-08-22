#ifndef VIEWER_H
#define VIEWER_H
// pangolin库
// #include <pangolin/pangolin.h>

// Eigen库
#include <Eigen/Core>
#include <Eigen/Dense>

//  std
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>
// lidar_slam
#include "lidar_slam/backend.hpp"
#include "lidar_slam/common_lib.h"
namespace lidar_slam {
#if 0
/*std::string matrixToString(const pangolin::OpenGlMatrix& matrix) {
    std::stringstream ss;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ss << matrix.m[j * 4 + i] << " ";
        }
        ss << std::endl;
    }
    return ss.str();
}*/
// 演示了如何画出一个预先存储的轨迹
struct Control_status{
    bool localizationMode = false;
    bool showMap = true;
    bool showLidar = true;
    bool showObstacle = false;
    bool saveMap = false;
    bool reset = false;
    float mapMin = 0;
    float mapMax = 1;
};
class Viewer{
    public:
    Viewer(bool mode){
        pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768); // 创建窗口
        
        glEnable(GL_DEPTH_TEST); // 开启深度测试
        glEnable(GL_BLEND); // 开启混合渲染
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // 设置混合函数
        
  /*      pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
        pangolin::Var<bool> menuTopView("menu.Top View",false,false);
        // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
        pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);
        pangolin::Var<bool> menuStop("menu.Stop",false,false);
        pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
        pangolin::Var<bool> menuStep("menu.Step",false,false); */   
        s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000), //投影矩阵
            // 屏幕的宽度、高度、相机的水平视角、垂直视角、相机在z轴上的位置、相机到屏幕的距离的最小值和最大值。
            pangolin::ModelViewLookAt(0, 0, 30, 0, 0, 0,pangolin::AxisY) // 视图矩阵
            // 相机的位置、相机观察的目标点、相机的朝向向量
        );
    
        d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            // 表示窗口在x轴和y轴上的起点和终点位置，以及窗口的宽高比，宽高比为负数，则实际上是768：1024
            .SetHandler(new pangolin::Handler3D(s_cam));  

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        localizationMode = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Localization Mode", false, true));
        *localizationMode = mode;
        showMap = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.show map", true, true));
        showObstacle = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.show obstacle", false, true));
        showLidar = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.show lidar", true, true));
        saveMap = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.save map", false, false));
        reset = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.reset", false, false));
        MapMin = std::unique_ptr<pangolin::Var<float>>(new pangolin::Var<float>("menu.min map ", 0, -1, 1));
        MapMax = std::unique_ptr<pangolin::Var<float>>(new pangolin::Var<float>("menu.max map ", 0.5, 0.5, 2));
        odom_cloud.reset(new PointCloudType());    
        std::cout << "start pangolin viewer"<<std::endl; 
    }
    ~Viewer(){}
    void Start(){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清空颜色缓冲区和深度缓冲区        
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 设置清屏颜色   
        d_cam.Activate(s_cam); // 激活显示窗口和渲染状态对象  
    }
    Control_status getControl(){
      //   pangolin::OpenGlMatrix p = s_cam.GetModelViewMatrix();
       //  std::cout << "Projection Matrix:" << std::endl << matrixToString(p) << std::endl;
         status.localizationMode = (*localizationMode);
         status.showMap = (*showMap);
         status.showObstacle = (*showObstacle);
         status.showLidar = (*showLidar);

         status.saveMap = (*saveMap);
         status.reset = (*reset);
         status.mapMin = (*MapMin);
         status.mapMax = (*MapMax);
         *saveMap = 0;
         *reset = 0;
         return status;
    }

    void DrawTrajectory(std::vector<KeyPose> poses,Eigen::Vector3f color){
         if (poses.size() == 0)
             return;

        glLineWidth(2); // 设置线宽
    /*    for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].pose.translation(); // 获取相机位姿矩阵中的平移部分，即相机的位置。
            Eigen::Vector3d Xw = poses[i].pose * (0.1 * Eigen::Vector3d(1, 0, 0)); // 获取x轴方向的单位向量,乘以0.1是为了调整坐标轴线段的长度
            Eigen::Vector3d Yw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 1, 0)); // 获取y轴方向的单位向量
            Eigen::Vector3d Zw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 0, 1)); // 获取z轴方向的单位向量
            
            glBegin(GL_LINES); // 开始绘制线段
            glColor3f(1.0, 0.0, 0.0); // 设置线段颜色 rgb
            // 绘制线段的两个端点
            glVertex3d(Ow[0], Ow[1], Ow[2]); // 原点的坐标
            glVertex3d(Xw[0], Xw[1], Xw[2]); // x轴方向的坐标    ----> 绘制x轴线段 为红色
 
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Yw[0], Yw[1], Yw[2]);// y轴方向的坐标    ----> 绘制y轴线段 为绿色
 
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Zw[0], Zw[1], Zw[2]);// z轴方向的坐标    ----> 绘制z轴线段 为蓝色
            glEnd(); // 结束绘制
        }*/
        
        // 画出连线
        for (size_t i = 0; i < poses.size()-1; i++) {
            glColor3f(color[0],color[1],color[2]); // 黑色
            glBegin(GL_LINES); // 开始绘制线段
            auto p1 = poses[i].pose, p2 = poses[i + 1].pose; // 获取相邻相机位姿
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]); // 绘制线段的两个端点(相邻相机位姿的位置)
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
    }
    void DrawTrajectory(std::vector<Eigen::Isometry3d> poses,Eigen::Vector3f color){
         if (poses.size() == 0)
             return;

        glLineWidth(2); // 设置线宽
    /*    for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].pose.translation(); // 获取相机位姿矩阵中的平移部分，即相机的位置。
            Eigen::Vector3d Xw = poses[i].pose * (0.1 * Eigen::Vector3d(1, 0, 0)); // 获取x轴方向的单位向量,乘以0.1是为了调整坐标轴线段的长度
            Eigen::Vector3d Yw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 1, 0)); // 获取y轴方向的单位向量
            Eigen::Vector3d Zw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 0, 1)); // 获取z轴方向的单位向量
            
            glBegin(GL_LINES); // 开始绘制线段
            glColor3f(1.0, 0.0, 0.0); // 设置线段颜色 rgb
            // 绘制线段的两个端点
            glVertex3d(Ow[0], Ow[1], Ow[2]); // 原点的坐标
            glVertex3d(Xw[0], Xw[1], Xw[2]); // x轴方向的坐标    ----> 绘制x轴线段 为红色
 
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Yw[0], Yw[1], Yw[2]);// y轴方向的坐标    ----> 绘制y轴线段 为绿色
 
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Zw[0], Zw[1], Zw[2]);// z轴方向的坐标    ----> 绘制z轴线段 为蓝色
            glEnd(); // 结束绘制
        }*/
        
        // 画出连线
        for (size_t i = 0; i < poses.size()-1; i++) {
            glColor3f(color[0],color[1],color[2]); // 黑色
            glBegin(GL_LINES); // 开始绘制线段
            auto p1 = poses[i], p2 = poses[i + 1]; // 获取相邻相机位姿
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]); // 绘制线段的两个端点(相邻相机位姿的位置)
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
    }
    void DrawTrajectory(std::deque<Eigen::Isometry3d> poses,Eigen::Vector3f color){
         if (poses.size() == 0)
             return;

        glLineWidth(2); // 设置线宽
    /*    for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].pose.translation(); // 获取相机位姿矩阵中的平移部分，即相机的位置。
            Eigen::Vector3d Xw = poses[i].pose * (0.1 * Eigen::Vector3d(1, 0, 0)); // 获取x轴方向的单位向量,乘以0.1是为了调整坐标轴线段的长度
            Eigen::Vector3d Yw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 1, 0)); // 获取y轴方向的单位向量
            Eigen::Vector3d Zw = poses[i].pose * (0.1 * Eigen::Vector3d(0, 0, 1)); // 获取z轴方向的单位向量
            
            glBegin(GL_LINES); // 开始绘制线段
            glColor3f(1.0, 0.0, 0.0); // 设置线段颜色 rgb
            // 绘制线段的两个端点
            glVertex3d(Ow[0], Ow[1], Ow[2]); // 原点的坐标
            glVertex3d(Xw[0], Xw[1], Xw[2]); // x轴方向的坐标    ----> 绘制x轴线段 为红色
 
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Yw[0], Yw[1], Yw[2]);// y轴方向的坐标    ----> 绘制y轴线段 为绿色
 
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);// 原点的坐标
            glVertex3d(Zw[0], Zw[1], Zw[2]);// z轴方向的坐标    ----> 绘制z轴线段 为蓝色
            glEnd(); // 结束绘制
        }*/
        
        // 画出连线
        for (size_t i = 0; i < poses.size()-1; i++) {
            glColor3f(color[0],color[1],color[2]); // 黑色
            glBegin(GL_LINES); // 开始绘制线段
            auto p1 = poses[i], p2 = poses[i + 1]; // 获取相邻相机位姿
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]); // 绘制线段的两个端点(相邻相机位姿的位置)
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
    }
    void DrawLine(Eigen::Isometry3d pose1 ,Eigen::Isometry3d pose2, Eigen::Vector3f color){
        glLineWidth(2); // 设置线宽
        glColor3f(color[0],color[1],color[2]); // 黑色
        glBegin(GL_LINES); // 开始绘制线段
        glVertex3d(pose1.translation()[0], pose1.translation()[1], pose1.translation()[2]); // 绘制线段的两个端点(相邻相机位姿的位置)
        glVertex3d(pose2.translation()[0], pose2.translation()[1], pose2.translation()[2]);
        glEnd();
    }

    void DrawPose(Eigen::Isometry3d pose){

        glPushMatrix();    
        glMultMatrixd(pose.matrix().data());//pangolin后续绘制中的所有坐标均需要乘以这个矩阵
        //3绘制相机轮廓线
        glLineWidth(4); //4-1
        glBegin(GL_LINES);//4-2
        glColor3f(1.0, 0.0, 0.0); // 设置线段颜色 rgb
        // 绘制线段的两个端点
        glVertex3d(0.0, 0.0, 0.0); // 原点的坐标
        glVertex3d(1.0, 0.0, 0.0); // x轴方向的坐标    ----> 绘制x轴线段 为红色

        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);// 原点的坐标
        glVertex3d(0.0, 1.0, 0.0);// y轴方向的坐标    ----> 绘制y轴线段 为绿色

        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);// 原点的坐标
        glVertex3d(0.0, 0.0, 1.0);// z轴方向的坐标    ----> 绘制z轴线段 为蓝色
        //5
        glEnd();
            //3更新矩阵
        glPopMatrix();
    }
    void DrawCloud(PointCloudType::Ptr cloud,Eigen::Isometry3d pose,Eigen::Vector3f color,float size){
        if (cloud->points.size() == 0)
           return;
        glPushMatrix();  
        glMultMatrixd(pose.matrix().data());
        pcl::copyPointCloud(*cloud, *odom_cloud);
        glPointSize(size);
        glBegin(GL_POINTS);
        glColor3f(color[0],color[1],color[2]);
        for(int i=0; i < odom_cloud->points.size();i++)
        {
            glVertex3f(odom_cloud->points[i].x,odom_cloud->points[i].y,odom_cloud->points[i].z);
        }
        glEnd();
        glPopMatrix();
    }
    void DrawCloud(PointCloudType::Ptr cloud,Eigen::Vector3f color,float size){
        if (cloud->points.size() == 0)
           return;
        pcl::copyPointCloud(*cloud, *odom_cloud);
        glPointSize(size);
        glBegin(GL_POINTS);
        glColor3f(color[0],color[1],color[2]);
        for(int i=0; i < odom_cloud->points.size();i++)
        {
            if (odom_cloud->points[i].z > status.mapMin && odom_cloud->points[i].z < status.mapMax)
                glVertex3f(odom_cloud->points[i].x,odom_cloud->points[i].y,odom_cloud->points[i].z);
        }
        glEnd();
    }
    
    void DrawCloud(std::vector<Eigen::Vector3f>& points,Eigen::Vector3f color,float size){
        if (points.size() == 0)
            return;
        std::vector<Eigen::Vector3f> colors(points.size(), color); 
        glPointSize(size);
        pangolin::glDrawColoredVertices(points.size(), points.data(), colors.data(), GL_POINTS);
    }
    void Finish(){
        pangolin::FinishFrame(); // 结束当前帧的绘制
        
    }
    

    private:
        pangolin::OpenGlRenderState s_cam;
        pangolin::View d_cam;
        PointCloudType::Ptr odom_cloud;
        std::unique_ptr<pangolin::Var<bool>> localizationMode;
        std::unique_ptr<pangolin::Var<bool>> saveMap;
        std::unique_ptr<pangolin::Var<bool>> reset;
        std::unique_ptr<pangolin::Var<bool>> showMap;
        std::unique_ptr<pangolin::Var<bool>> showObstacle;
        std::unique_ptr<pangolin::Var<bool>> showLidar;
        std::unique_ptr<pangolin::Var<float>> MapMin;
        std::unique_ptr<pangolin::Var<float>> MapMax;
        Control_status status;

};
#endif
} // namespace lidar_slam
#endif
