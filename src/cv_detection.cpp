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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include "cv_detection.h"


using namespace std;
using namespace cv;


DetectionCV::DetectionCV(ros::NodeHandle& nh_)
:it_(nh_)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("initial_x", p_initial_x, 350);
    private_nh_.param("initial_y", p_initial_y, 200);
    private_nh_.param("initial_r", p_initial_r, 20);

    object.x=p_initial_x;
    object.y=p_initial_y;

    camera_sub_=it_.subscribe("/camera/image_raw", 1, &DetectionCV::object_detection, this);

    image_object_pub_=nh_.advertise<geometry_msgs::Pose2D>("/camera/object_position", 1, true);

}

DetectionCV::~DetectionCV(void)
{

}
/*Object detection*/
void DetectionCV::object_detection(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat prev_image;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    IplImage UNImg=cv_ptr->image.operator _IplImage();

    cvShowImage("original",&UNImg);
    cv::waitKey(3);

    CV_Detect(&UNImg, object);

    geometry_msgs::Pose2D object_pose2D;
    object_pose2D.x=object.x;
    object_pose2D.y=object.y;

    image_object_pub_.publish(object_pose2D);
}

double DetectionCV::Sum(IplImage *srcImg,int nrow,int ncol)
{
    double ImgSum=0;
    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
             ImgSum+=CV_IMAGE_ELEM(srcImg,float,i,j);
        }
    }

    return ImgSum;
}

void DetectionCV::ShowIndex(int row,int col,IplImage *srcImg)
{
    int nrow,ncol;
    CvSize ImageSize=cvGetSize(srcImg);
    while(1)
    {
        scanf("%d",&nrow);
        scanf("%d",&ncol);
        if(nrow==-1||ncol==-1)
            break;
    }
}

void DetectionCV::ImageConvert(IplImage *srcImg,int nrow,int ncol)
{
    float ImageMax=0,Gain;
    double Gain1=1.0;
    IplImage* TempImage=cvCloneImage(srcImg);

    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
            if(CV_IMAGE_ELEM(srcImg,float,i,j)>ImageMax)
            {
                ImageMax=CV_IMAGE_ELEM(srcImg,float,i,j);
            }
        }
    }

    //cvMaxS(srcImg,Gain1,TempImage);

    Gain=255/ImageMax;

    cvSet(TempImage,cvScalar(Gain));
    cvMul(srcImg,TempImage,srcImg,1.0);

    cvReleaseImage(&TempImage);
}

void DetectionCV::sdf2circle(IplImage* Phi_0,int nrow,int ncol,int Next_X,int Next_Y,int r,CvSize ImgSize)
{
    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
             CV_IMAGE_ELEM(Phi_0,float,i,j)=sqrt((pow((double)(j-Next_X),2.0)+pow((double)(i-Next_Y),2.0)))-(double)r;
        }
    }
}

void DetectionCV::Heaviside(IplImage* phi,double epsilon,IplImage* H)
{
    CvSize ImageSize=cvGetSize(phi);

    for(int i=0;i<ImageSize.height;i++)
    {
        for(int j=0;j<ImageSize.width;j++)
        {
            CV_IMAGE_ELEM(H,float,i,j)=0.5*(1+(2/3.1416)*atan(CV_IMAGE_ELEM(phi,float,i,j)/epsilon));
        }
    }

}

void DetectionCV::Delta(IplImage* phi,double epsilon,IplImage* Delta_h)
{
    IplImage* TempImage=cvCloneImage(phi);

    cvMul(phi,phi,Delta_h,1.0);
    cvAddS(Delta_h,cvScalar(epsilon*epsilon),Delta_h);
    cvSet(TempImage,cvScalar(epsilon/3.1416));
    cvDiv(TempImage,Delta_h,Delta_h,1.0);

    cvReleaseImage(&TempImage);
}

void DetectionCV::NeumannBoundCond(IplImage* phi)//Make a function satisfy Neumann boundary condition
{
    CvSize ImgSize=cvGetSize(phi);
    int i;
    int nrow=ImgSize.height,ncol=ImgSize.width;
    CV_IMAGE_ELEM(phi,float,0,0)       = CV_IMAGE_ELEM(phi,float,2,2);
    CV_IMAGE_ELEM(phi,float,0,ncol-1)  = CV_IMAGE_ELEM(phi,float,2,ncol-3);
    CV_IMAGE_ELEM(phi,float,nrow-1,0)  = CV_IMAGE_ELEM(phi,float,nrow-3,2);
    CV_IMAGE_ELEM(phi,float,nrow-1,ncol-1)  = CV_IMAGE_ELEM(phi,float,nrow-3,ncol-3);
    for(i=1;i<ncol-1;i++)
    {
         CV_IMAGE_ELEM(phi,float,4330,i)=CV_IMAGE_ELEM(phi,float,2,i);
    }
    for(i=1;i<ncol-1;i++)
    {
         CV_IMAGE_ELEM(phi,float,nrow-1,i)=CV_IMAGE_ELEM(phi,float,nrow-3,i);
    }
    for(i=1;i<nrow-1;i++)
    {
        CV_IMAGE_ELEM(phi,float,i,0)=CV_IMAGE_ELEM(phi,float,i,2);
    }
    for(i=1;i<nrow-1;i++)
    {
         CV_IMAGE_ELEM(phi,float,i,ncol-1)=CV_IMAGE_ELEM(phi,float,i,ncol-3);
    }

}

double DetectionCV::binaryfit_C1(IplImage* Img,IplImage* H_phi,int nrow,int ncol)
{
    IplImage* a=cvCloneImage(Img);
    double numer_1,denom_1,C1;

    cvMul(H_phi,Img,a,1.0);

    numer_1=Sum(a,nrow,ncol);
    denom_1=Sum(H_phi,nrow,ncol);
    C1=numer_1/denom_1;

    cvReleaseImage(&a);
    return C1;
}

double DetectionCV::binaryfit_C2(IplImage* Img,IplImage* H_phi,int nrow,int ncol)
{
    IplImage* a=cvCloneImage(Img);
    double numer_1,denom_1,C2;
    cvSubRS(H_phi,cvScalar(1.0),H_phi);
    cvMul(H_phi,Img,a);
    numer_1=Sum(a,nrow,ncol);
    denom_1=Sum(H_phi,nrow,ncol);
    C2=numer_1/denom_1;

    cvReleaseImage(&a);
    return C2;
}
/*Chan-Vese model-based image segmentation*/
void DetectionCV::EVOL_CV(IplImage* I,IplImage* phi,double nu,double lambda_1,double lambda_2,double timestep,double epsilon,int numIter)
{
    IplImage *Hphi=cvCloneImage(phi);
    IplImage *kappa=cvCloneImage(phi);
    IplImage *diracPhi=NULL,*I_C1=NULL,*I_C2=NULL,*lambda1=NULL,*lambda2=NULL,*Times=NULL;
    double C1,C2;
    CvSize ImgSize=cvGetSize(I);
    int nrow=ImgSize.height,ncol=ImgSize.width;

    I_C1=cvCloneImage(I);
    I_C2=cvCloneImage(I);
    diracPhi=cvCloneImage(I);
    lambda1=cvCloneImage(I);
    lambda2=cvCloneImage(I);
    Times=cvCloneImage(I);

    for(int i=0;i<numIter;i++)
    {
        NeumannBoundCond(phi);

        Delta(phi,epsilon,diracPhi);

        Heaviside(phi, epsilon,Hphi);

        C1=binaryfit_C1(I,Hphi,nrow,ncol);

        C2=binaryfit_C2(I,Hphi,nrow,ncol);

        cvSubS(I,cvScalar(C1),I_C1);
        cvPow(I_C1,I_C1,2.0);
        cvSet(lambda1,cvScalar(lambda_1));
        cvMul(lambda1,I_C1,I_C1);
        cvSubRS(I_C1,cvScalar(nu),I_C1);

        cvSubS(I,cvScalar(C2),I_C2);
        cvPow(I_C2,I_C2,2.0);
        cvSet(lambda2,cvScalar(lambda_2));
        cvMul(lambda2,I_C2,I_C2);
        cvAdd(I_C1,I_C2,I_C1);
        cvMul(diracPhi,I_C1,diracPhi);
        cvSet(Times,cvScalar(timestep));
        cvMul(Times,diracPhi,Times);
        cvAdd(phi,Times,phi);
    }

    cvReleaseImage(&diracPhi);
    cvReleaseImage(&Hphi);
    cvReleaseImage(&kappa);
    cvReleaseImage(&I_C1);
    cvReleaseImage(&I_C2);
    cvReleaseImage(&lambda1);
    cvReleaseImage(&lambda2);
    cvReleaseImage(&Times);

}

void DetectionCV::ImageProcess_1(IplImage* phi,int nrow,int ncol)
{
    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
            if(CV_IMAGE_ELEM(phi,float,i,j)<0)
            {
                CV_IMAGE_ELEM(phi,float,i,j)=-CV_IMAGE_ELEM(phi,float,i,j);
                if(CV_IMAGE_ELEM(phi,float,i,j)>255)
                    CV_IMAGE_ELEM(phi,float,i,j)=255;
            }
            else
                CV_IMAGE_ELEM(phi,float,i,j)=0;
        }
    }

}
/*Circling the UAV*/
void DetectionCV::PlanePoint(IplImage* ProcessImg_1,IplImage* SImg,int nrow,int ncol,CvPoint &PlanePoint)
{
    int num=0,SumRow=0,SumCol=0,AveRow=0,AveCol=0;
    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
            if(CV_IMAGE_ELEM(ProcessImg_1,float,i,j)!=0&&CV_IMAGE_ELEM(SImg,float,i,j)>200)
            {
                num++;
                SumRow+=i;
                SumCol+=j;
            }
        }
    }
    if(num!=0)
    {
        AveRow=SumRow/num;
        AveCol=SumCol/num;
        PlanePoint.x=AveCol;
        PlanePoint.y=AveRow;
        //cout<<PlanePoint.x<<","<<PlanePoint.y<<endl;
    }
    else
    {
        PlanePoint.x=0;
        PlanePoint.y=0;
        ROS_WARN("No Such Point");
    }
}

void DetectionCV::CV_Detect(IplImage* UNImg,CvPoint &Plane)
{
    IplImage *phi_0=NULL,*phi=NULL;
    IplImage *floatUNImg=NULL,*HSVImg=NULL,*SImg=NULL,*HImg=NULL,*VImg=NULL;
    CvSize ImgSize;
    int nrow,ncol;
    int Next_X=Plane.x;
    int Next_Y=Plane.y;
    int r;
    int i,m,n;
    int numIter;
    int HeadNum,LastHeadNum;
    double nu,lambda_1,lambda_2,timestep,epsilon;
    float index1,index2;

    LastHeadNum=0;
    numIter=10;
    nu = 0.001*255*255;
    lambda_1=1;
    lambda_2=1;
    epsilon=1;
    timestep=0.8;

    ImgSize=cvGetSize(UNImg);
    nrow=ImgSize.height;
    ncol=ImgSize.width;

    floatUNImg=cvCreateImage(ImgSize,IPL_DEPTH_32F,3);
    HSVImg=cvCreateImage(ImgSize,IPL_DEPTH_32F,3);
    SImg=cvCreateImage(ImgSize,IPL_DEPTH_32F,1);
    phi_0 = cvCloneImage(SImg);
    phi = cvCloneImage(SImg);

    cvConvertScale(UNImg,floatUNImg,1.0/255.0,0);

    cvCvtColor(floatUNImg,HSVImg,CV_BGR2HSV);

    cvSplit(HSVImg,HImg,SImg,VImg,NULL);

    ImageConvert(SImg,nrow,ncol);

    sdf2circle(phi,nrow,ncol,Next_X,Next_Y,p_initial_r,ImgSize);


    for(i=0;i<numIter;i++)
    {
        EVOL_CV(SImg,phi,nu,lambda_1, lambda_2, timestep, epsilon, 1);

        HeadNum=0;

        for(m=0;m<nrow;m++)
        {
            for(n=0;n<ncol;n++)
            {
                index1=CV_IMAGE_ELEM(phi,float,m,n);
                index2=CV_IMAGE_ELEM(SImg,float,m,n);

                if(CV_IMAGE_ELEM(phi,float,m,n)<=0)
                    if(CV_IMAGE_ELEM(SImg,float,m,n)>=200)
                         HeadNum++;
            }
        }
        if(abs(HeadNum-LastHeadNum)<=0&&HeadNum>10)
        {
            r=HeadNum/4;
            if(r>20)
                r=20;
            if(r<5)
                r=5;
            break;
        }
        LastHeadNum=HeadNum;

    }

    cvCircle(UNImg,Plane,20,CV_RGB(255,0,0),2,8,0);
    ImageProcess_1(phi,nrow,ncol);

    PlanePoint(phi,SImg,nrow,ncol,Plane);

    cvCircle(UNImg,Plane,2,CV_RGB(0,255,0),2,8,0);

    cvShowImage("Result",UNImg);
    waitKey();
    cvReleaseImage(&phi_0);
    cvReleaseImage(&phi);
    cvReleaseImage(&floatUNImg);
    cvReleaseImage(&HSVImg);
    cvReleaseImage(&HImg);
    cvReleaseImage(&SImg);
    cvReleaseImage(&VImg);

}


