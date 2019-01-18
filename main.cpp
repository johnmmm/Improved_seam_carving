#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "Improved_seam.hpp"
#include "Video_seam.hpp"

using namespace std;
using namespace cv;

bool is_min[1000] = {false};

//计算能量曲线
void calculateColEnergy(Mat& srcMat, Mat& dstMat, Mat& traceMat)  //Mat 矩阵
{
    srcMat.copyTo(dstMat);   //不用“=”，防止两个矩阵指向的都是同一个矩阵，现在只需要传里面的数值
    
    for (int i = 1;i < srcMat.rows;i++)  //从第2行开始计算
    {
        //第一列
        if (dstMat.at<float>(i-1,0) <= dstMat.at<float>(i-1,1))  //.at 获取像素
        {
            dstMat.at<float>(i,0) = srcMat.at<float>(i,0) + dstMat.at<float>(i-1,0);
            traceMat.at<float>(i,0) = 1; //traceMat记录当前位置的上一行应取那个位置，上左为0，上中1，上右为2
        }
        else
        {
            dstMat.at<float>(i,0) = srcMat.at<float>(i,0) + dstMat.at<float>(i-1,1);
            traceMat.at<float>(i,0) = 2;
        }
        
        //中间列
        for (int j = 1;j < srcMat.cols-1;j++)  //获取每一个点的能量路径
        {
            float k[3];  //获取上一行对应三个位置的像素作比较
            k[0] = dstMat.at<float>(i-1,j-1);
            k[1] = dstMat.at<float>(i-1,j);
            k[2] = dstMat.at<float>(i-1,j+1);
            
            int index = 0;
            if (k[1] < k[0])
                index = 1;
            if (k[2] < k[index])
                index = 2;
            dstMat.at<float>(i,j) = srcMat.at<float>(i,j) + dstMat.at<float>(i-1,j-1+index);
            traceMat.at<float>(i,j) = index;
            
        }
        
        //最后一列
        if (dstMat.at<float>(i-1,srcMat.cols-1) <= dstMat.at<float>(i-1,srcMat.cols-2))
        {
            dstMat.at<float>(i,srcMat.cols-1) = srcMat.at<float>(i,srcMat.cols-1) + dstMat.at<float>(i-1,srcMat.cols-1);
            traceMat.at<float>(i,srcMat.cols-1) = 1;
        }
        else
        {
            dstMat.at<float>(i,srcMat.cols-1) = srcMat.at<float>(i,srcMat.cols-1) + dstMat.at<float>(i-1,srcMat.cols-2);
            traceMat.at<float>(i,srcMat.cols-1) = 0;
        }
        
    }
}

// 找出最小能量线
void getMinColEnergyTrace(const Mat& energyMat, const Mat& traceMat, Mat& minTrace)
{
    int row = energyMat.rows - 1;// 取的是energyMat最后一行的数据，所以行标是rows-1
    int index = 0;	// 保存的是最小那条轨迹的最下面点在图像中的列标
    
    // 获得index，即最后那行最小值的位置
    for (int i = 1; i < energyMat.cols; i++)
    {
        if (energyMat.at<float>(row,i) < energyMat.at<float>(row,index))
        {
            index = i;
        } 
    }
    
    // 以下根据traceMat，得到minTrace，minTrace是多行一列矩阵
    minTrace.at<float>(row,0) = index;
    int tmpIndex = index;
    
    for (int i = row; i > 0; i--)
    {
        int temp = traceMat.at<float>(i,tmpIndex);// 当前位置traceMat所存的值
        
        if (temp == 0) // 往左走
            tmpIndex = tmpIndex - 1;
        else if (temp == 2) // 往右走
            tmpIndex = tmpIndex + 1;
        // 如果temp = 1，则往正上走，tmpIndex不需要做修改
        
        minTrace.at<float>(i-1,0) = tmpIndex;
    }
}

void delOneCol(Mat& srcMat, Mat& dstMat, Mat& minTrace, Mat& beDeletedLine)
{
    for (int i = 0; i < dstMat.rows; i++)  //对目标图像进行操作
    {
        int k = minTrace.at<float>(i,0);
        
        for (int j = 0; j < k; j++)  //删除列前的元素复制
        {
            dstMat.at<Vec3b>(i,j)[0] = srcMat.at<Vec3b>(i,j)[0];  // 8U 类型的 RGB 彩色图像 [i] 存放bgr的值
            dstMat.at<Vec3b>(i,j)[1] = srcMat.at<Vec3b>(i,j)[1];
            dstMat.at<Vec3b>(i,j)[2] = srcMat.at<Vec3b>(i,j)[2];
        }
        for (int j = k; j < dstMat.cols-1; j++)  //复制删除列后的元素 最后一列默认不处理
        {
            if (j == dstMat.cols - 1)
                continue;
            dstMat.at<Vec3b>(i,j)[0] = srcMat.at<Vec3b>(i,j+1)[0];
            dstMat.at<Vec3b>(i,j)[1] = srcMat.at<Vec3b>(i,j+1)[1];
            dstMat.at<Vec3b>(i,j)[2] = srcMat.at<Vec3b>(i,j+1)[2];
        }
        beDeletedLine.at<float>(i,0) = k;
    }
}

void old_seam_carving (Mat& inputImage, Mat& outputImage)
{
    int rows = inputImage.rows;
    int cols = inputImage.cols;
    
    Mat image_gray(rows, cols, CV_8U, Scalar(0));  
    cvtColor(inputImage, image_gray, CV_BGR2GRAY); //彩色图像转换为灰度图像
    //cout << image_gray.at<uchar>(0, 0) << endl;
    
    Mat gradiant_H(rows, cols, CV_32F, Scalar(0));//水平梯度矩阵  32bit浮点数
    Mat gradiant_V(rows, cols, CV_32F, Scalar(0));//垂直梯度矩阵
    int scale = 1, delta = 0, kernal_size = 3;
    Laplacian(image_gray, gradiant_H, gradiant_H.depth(), kernal_size, scale, delta);
    Laplacian(image_gray, gradiant_V, gradiant_V.depth(), kernal_size, scale, delta);
    Mat gradMag_mat(rows, cols, CV_32F, Scalar(0));
    add(abs(gradiant_H), abs(gradiant_V), gradMag_mat);

    //计算能量线
    Mat energyMat(rows, cols, CV_32F, Scalar(0));//累计能量矩阵
    Mat traceMat(rows, cols, CV_32F, Scalar(0));//能量最小轨迹矩阵
    calculateColEnergy(gradMag_mat, energyMat, traceMat);
    
    //找出最小能量线
    Mat minTrace(rows, 1, CV_32F, Scalar(0));//能量最小轨迹矩阵中的最小的一条的轨迹
    getMinColEnergyTrace(energyMat, traceMat, minTrace);

    Mat tmpImage(rows, cols, inputImage.type());
    inputImage.copyTo(tmpImage);  //复制
    for (int i = 0; i < rows; i++)
    {
        int k = minTrace.at<float>(i,0);
        tmpImage.at<Vec3b>(i,k)[0] = 0;
        tmpImage.at<Vec3b>(i,k)[1] = 0;
        tmpImage.at<Vec3b>(i,k)[2] = 255;
    }
    //tmpImage.copyTo(outputImage);
    // imshow("Carving Window", tmpImage);
    // waitKey();

    Mat image2(rows, cols-1, inputImage.type());
    Mat beDeletedLine(rows, 1, CV_8UC3);
    delOneCol(inputImage, image2, minTrace, beDeletedLine);
    image2.copyTo(outputImage);
}

float calEnergy (float i_1j, float ij_1, float ij1, float i1j)
{
    float sum = 0;
    float LR = abs(ij1 - ij_1);
    float LU1 = abs(i_1j - ij_1);
    float LU2 = abs(i1j - ij_1);
    sum = LR + LU1 - LU2;
    return sum;
}

void makeEnergy(Mat& inputImage, Mat& outMat)
{
    int rows = inputImage.rows;
    int cols = inputImage.cols;

    Mat tmpImage (rows+2, cols+2, CV_32F, Scalar(0));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            tmpImage.at<float>(i+1, j+1) = float(inputImage.at<uchar>(i, j));
    for (int i = 1; i <= rows; i++)
    {
        tmpImage.at<float>(i, 0) = tmpImage.at<float>(i, 0);
        tmpImage.at<float>(i, cols+1) = tmpImage.at<float>(i, cols);
    }
    for (int j = 1; j <= cols; j++)
    {
        tmpImage.at<float>(0, j) = tmpImage.at<float>(1, j);
        tmpImage.at<float>(rows+1, j) = tmpImage.at<float>(rows, j);
    }
        
    Mat energyMat(rows, cols, CV_32F, Scalar(0));
    for (int i = 1; i < rows+1; i++)
    {
        for (int j = 1; j < cols+1; j++)
        {
            float i_1j, ij_1, ij1, i1j;
            i_1j = tmpImage.at<float>(i-1, j);
            ij_1 = tmpImage.at<float>(i, j-1);
            ij1 = tmpImage.at<float>(i, j+1);
            i1j = tmpImage.at<float>(i+1, j);
            energyMat.at<float>(i-1, j-1) = calEnergy(i_1j, ij_1, ij1, i1j);
        }
    }
    energyMat.copyTo(outMat);
}

void new_seam_carving(Mat& inputImage, Mat& outputImage, Mat& deletedLine)
{
    int rows = inputImage.rows;
    int cols = inputImage.cols;
    
    Mat image_gray(rows, cols, CV_8U, Scalar(0));  
    cvtColor(inputImage, image_gray, CV_BGR2GRAY); //彩色图像转换为灰度图像
    // cout << image_gray.at<uchar>(0, 0) << endl;
    
    // Mat gradiant_H(rows, cols, CV_32F, Scalar(0));//水平梯度矩阵  32bit浮点数
    // Mat gradiant_V(rows, cols, CV_32F, Scalar(0));//垂直梯度矩阵
    // int scale = 1, delta = 0, kernal_size = 3;
    // Laplacian(image_gray, gradiant_H, gradiant_H.depth(), kernal_size, scale, delta);
    // Laplacian(image_gray, gradiant_V, gradiant_V.depth(), kernal_size, scale, delta);
    Mat gradMag_mat(rows, cols, CV_32F, Scalar(0));
    // add(abs(gradiant_H), abs(gradiant_V), gradMag_mat);
    makeEnergy(image_gray, gradMag_mat);

    //计算能量线
    Mat energyMat(rows, cols, CV_32F, Scalar(0));//累计能量矩阵
    Mat traceMat(rows, cols, CV_32F, Scalar(0));//能量最小轨迹矩阵
    calculateColEnergy(gradMag_mat, energyMat, traceMat);
    
    //找出最小能量线
    Mat minTrace(rows, 1, CV_32F, Scalar(0));//能量最小轨迹矩阵中的最小的一条的轨迹
    getMinColEnergyTrace(energyMat, traceMat, minTrace);

    Mat tmpImage(rows, cols, inputImage.type());
    inputImage.copyTo(tmpImage);  //复制
    for (int i = 0; i < rows; i++)
    {
        int k = minTrace.at<float>(i,0);
        tmpImage.at<Vec3b>(i,k)[0] = 0;
        tmpImage.at<Vec3b>(i,k)[1] = 0;
        tmpImage.at<Vec3b>(i,k)[2] = 255;
    }
    //tmpImage.copyTo(outputImage);
    imshow("Carving Window", tmpImage);
    waitKey();

    Mat image2(rows, cols-1, inputImage.type());
    Mat beDeletedLine(rows, 1, CV_32F);
    delOneCol(inputImage, image2, minTrace, beDeletedLine);
    image2.copyTo(outputImage);
    beDeletedLine.copyTo(deletedLine);
}

int main (int argc, char** argv)
{
    Mat image1 = imread("./test1.png");
    // namedWindow("Original Window");
    // imshow("Original Window", image1);
    // waitKey();

    // namedWindow("Carving Window");
    //Mat image2(image1.rows, image1.cols, image1.type());
    Mat tmpMat;
    Mat image2;
    image1.copyTo(tmpMat);
    Mat deletedLines(image1.rows, 300, CV_32F);
    Mat deletedLine1(image1.rows, 1, CV_32F);

    clock_t time1, time2;
    time1 = clock();
    video_seam_carving();
    time2 = clock();
    printf("总共时间： %f s\n", (double)(time2 - time1) / CLOCKS_PER_SEC);



    // time1 = clock();
    // for (int i = 0; i < 200; i++)
    // {
    //     improved_seam_carving(tmpMat, image2);
    //     tmpMat = image2;
    // }
    // time2 = clock();
    // imshow("Carving Window", image2);
    // waitKey();
    // imwrite("./test1_1.png", image2);
    // printf("总共时间： %f s\n", (double)(time2 - time1) / CLOCKS_PER_SEC);

    // for (int i = 0; i < 200; i++)
    // {
    //     old_seam_carving(tmpMat, image2);
    //     tmpMat = image2;
    //     // for (int j = 0; j < image1.rows; j++)
    //     // {
    //     //     deletedLines.at<float>(i, j) = deletedLine1.at<float>(j, 1) + i;
    //     // }
    //     //image2.copyTo(image1);
    //     // imshow("Carving Window", image2);
    //     // waitKey();
    //     cout << i << endl;
    // }
    // imshow("Carving Window", image2);
    // waitKey();
    // imwrite("./test1_3.png", image2);

    //试图显示切掉的线
    // for (int i = 0; i < 50; i++)
    // {
    //     for (int j = 0; j < image1.rows; j++)
    //     {
    //         int k = deletedLines.at<float>(i, j);
    //         image1.at<Vec3b>(j,k)[0] = 0;
    //         image1.at<Vec3b>(j,k)[1] = 0;
    //         image1.at<Vec3b>(j,k)[2] = 255;
    //     }
    // }
    // imshow("Origin Window", image1); 
    // waitKey();
    
    return 0;
}
