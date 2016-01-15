/*
* Software License Agreement (BSD License)
*Copyright (c) 2015, micROS Team
 http://micros.nudt.edu.cn/
*All rights reserved.
* Copyright (c) 2009, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>  
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;

class DetectionCV
{
  ros::Publisher image_object_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_sub_;

  ros::Subscriber object_pridict_sub_;
  
  CvPoint object;
  
public:
  DetectionCV(ros::NodeHandle& nh_);
  ~DetectionCV(void);
  
protected:
  int p_initial_x;
  int p_initial_y;
  int p_initial_r;  
private:
  void object_detection(const sensor_msgs::ImageConstPtr& msg);
  double Sum(IplImage *srcImg,int nrow,int ncol);
  void ShowIndex(int row,int col,IplImage *srcImg);
  void ImageConvert(IplImage *srcImg,int nrow,int ncol);
  void sdf2circle(IplImage* Phi_0,int nrow,int ncol,int Next_X,int Next_Y,int r,CvSize ImgSize);
  void Heaviside(IplImage* phi,double epsilon,IplImage* H);
  void Delta(IplImage* phi,double epsilon,IplImage* Delta_h);
  void NeumannBoundCond(IplImage* phi);
  double binaryfit_C1(IplImage* Img,IplImage* H_phi,int nrow,int ncol);
  double binaryfit_C2(IplImage* Img,IplImage* H_phi,int nrow,int ncol);
  void EVOL_CV(IplImage* I,IplImage* phi,double nu,double lambda_1,double lambda_2,double timestep,double epsilon,int numIter);
  void ImageProcess_1(IplImage* phi,int nrow,int ncol);
  void PlanePoint(IplImage* ProcessImg_1,IplImage* SImg,int nrow,int ncol,CvPoint &PlanePoint);
  void CV_Detect(IplImage* UNImg,CvPoint &Plane);
 
};
