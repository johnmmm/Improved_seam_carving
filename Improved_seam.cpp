#include "Improved_seam.hpp"

using namespace std;

map<int, map<int, int, greater<int>>> topo_max;//起始点，结束点和对应上线
map<int, map<int, int>> topo_tmp;
bool is_instack[1000000];//判断是否读到了
int search_place[1000000];//知道搜到第几个了
vector<int> search_stack;//记录一路上的顺序

int get_id(int i, int j, int cols)
{
    return (i - 1) * cols + j - 1;
}

void improved_seam_carving (Mat& inputImage, Mat& outputImage)
{
    int rows = inputImage.rows;
    int cols = inputImage.cols;
    int s = rows * cols;
    int t = rows * cols + 1;

    //--------->>>>>输入图图片
    Mat image_gray(rows, cols, CV_8U, Scalar(0));  
    cvtColor(inputImage, image_gray, CV_BGR2GRAY);
    //构造一个变大的图
    Mat tmpImage (rows+2, cols+2, CV_32F, Scalar(0));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            tmpImage.at<float>(i+1, j+1) = float(image_gray.at<uchar>(i, j));
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
    
    //--------->>>>>建立图的集合
    //构造边们
    for (int i = 1; i < rows; i++)
    {
        for (int j = 1; j < cols; j++)
        {
            int a1 = get_id(i, j, cols);
            int a2 = get_id(i, j+1, cols);
            int a3 = get_id(i+1, j, cols);
            int a4 = get_id(i+1, j+1, cols);
            float i_1j, ij_1, ij1, i1j;
            i_1j = tmpImage.at<float>(i-1, j);
            ij_1 = tmpImage.at<float>(i, j-1);
            ij1 = tmpImage.at<float>(i, j+1);
            i1j = tmpImage.at<float>(i+1, j);
            topo_max[a2][a1] = INFMAX;
            topo_max[a2][a3] = INFMAX;
            topo_max[a4][a1] = INFMAX;
            topo_max[a1][a2] = abs(ij1 - ij_1);
            topo_max[a1][a3] = abs(i_1j - ij_1);
            topo_max[a3][a1] = abs(i1j - ij_1);
        }
    }
    for (int j = 1; j < cols; j++)
    {
        int i = rows;
        int a1 = get_id(i, j, cols);
        int a2 = get_id(i, j+1, cols);
        int ij_1 = tmpImage.at<float>(i, j-1);
        int ij1 = tmpImage.at<float>(i, j+1);
        topo_max[a2][a1] = INFMAX;
        topo_max[a1][a2] = abs(ij1 - ij_1);
    }
    for (int i = 1; i < rows; i++)
    {
        int j = cols;
        int a1 = get_id(i, j, cols);
        int a3 = get_id(i+1, j, cols);
        float i_1j, ij_1, i1j;
        i_1j = tmpImage.at<float>(i-1, j);
        ij_1 = tmpImage.at<float>(i, j-1);
        i1j = tmpImage.at<float>(i+1, j);
        topo_max[a1][a3] = abs(i_1j - ij_1);
        topo_max[a3][a1] = abs(i1j - ij_1);
    }
    for (int i = 1; i <= rows; i++)
    {
        int a1 = get_id(i, 1, cols);
        int a2 = get_id(i, cols, cols);
        topo_max[s][a1] = INFMAX;
        topo_max[a2][t] = INFMAX;
    }
    int amount = 2 * (2 * cols * rows - cols - rows) + 2 * (rows - 1) * (cols - 1) + rows * 2;
    int total = 0;
    for (auto it = topo_max.begin(); it != topo_max.end(); it++)
    {
        map<int, int, greater<int>> tmp_map = topo_max[it->first];
        for (auto it1 = tmp_map.begin(); it1 != tmp_map.end(); it1++)
        {
            topo_tmp[it->first][it1->first] = 0;
            total++;
        }
    }
    printf("rows: %d, cols: %d\n", rows, cols);
    printf("amount: %d\n", amount);
    printf("total: %d\n", total);
    printf("1: %d, 2: %d, 3: %d\n", topo_max[150000][150001], topo_max[0][1], topo_max[22][21]);
    
    
    //--------->>>>>手写神搜
    int max_flow = 0;
    int tmp_place = s;//当前节点
    int min_flow = INFMAX;//记录路径中最小的
    bool flag_no_place = false;//是否没路可走了，回退
    search_stack.push_back(s);
    while (1)
    {
        //搜索下一个
        // if (max_flow == 9)
        // {
        //     printf("now in: %d\n", tmp_place);
        //     namedWindow("Original Window");
        //     imshow("Original Window", image_gray);
        //     waitKey();
        // }
        
        map<int, int, greater<int>> tmp_map = topo_max[tmp_place];
        int cur_place = 0;
        flag_no_place = true;
        if (topo_max[tmp_place].count(t) > 0)//直接进终点，这样简便
        {
            tmp_place = t;
            search_stack.push_back(tmp_place);
            flag_no_place = false;
        }
        else
        {
            for (auto it = tmp_map.begin(); it != tmp_map.end(); it++)
            {
                if (search_place[it->first] > 0 || topo_tmp[tmp_place][it->first] == it->second)//搜过或者不够
                {
                    cur_place++;
                    continue;
                }
                else if (cur_place >= search_place[tmp_place])//满足要求
                {
                    search_place[tmp_place] = cur_place+1;
                    tmp_place = it->first;
                    search_stack.push_back(tmp_place);
                    flag_no_place = false;
                    break;
                }
                cur_place++;
            }
        }
        
        if (flag_no_place)
        {
            //回退
            if (tmp_place == s)//结束了
            {
                break;
            }
            else
            {
                //search_place[search_stack.at(search_stack.size()-1)] = 0;
                search_stack.pop_back();
                tmp_place = search_stack.at(search_stack.size()-1);
            }
        }

        if (tmp_place == t)
        {
            //结束了，做收尾工作
            for (int i = 0; i < search_stack.size() - 1; i++)
            {
                int s_place = search_stack.at(i);
                int t_place = search_stack.at(i+1);
                if (min_flow > topo_max[s_place][t_place] - topo_tmp[s_place][t_place])
                    min_flow = topo_max[s_place][t_place] - topo_tmp[s_place][t_place];
            }
            for (int i = 0; i < search_stack.size() - 1; i++)
            {
                int s_place = search_stack.at(i);
                int t_place = search_stack.at(i+1);
                topo_tmp[s_place][t_place] += min_flow;
                //search_place[s_place] = 0;
            }
            //恢复原状
            //max_flow += min_flow;
            //printf("now max_flow: %d\n", max_flow);

            //search_place[search_stack.size()-1] = 0;
            memset(search_place, 0, sizeof(int)*1000000);
            search_stack.clear();
            search_stack.push_back(s);
            tmp_place = s;
            min_flow = INFMAX;

            
            continue;
        }
            
    }
    

    //--------->>>>>找到最小割
    total = 0;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols-1; j++)
        {
            int a1 = get_id(i+1, j+1, cols);
            if (topo_max[a1][a1+1] == topo_tmp[a1][a1+1])
                total++;
            if (i < rows - 1)
            {
                if (topo_max[a1][a1+cols] == topo_max[a1][a1+cols] || topo_max[a1+cols][a1] == topo_max[a1+cols][a1])
                    total++;
            }
        }
    printf("place to cut: %d\n", total);

    memset(search_place, 0, sizeof(int)*1000000);
    tmp_place = s;//当前节点
    flag_no_place = false;//是否没路可走了，回退
    search_stack.clear();
    search_stack.push_back(s);
    int seam_place[1000] = {0};
    while(1)
    {
        map<int, int, greater<int>> tmp_map = topo_max[tmp_place];
        int cur_place = 0;
        flag_no_place = true;
        for (auto it = tmp_map.begin(); it != tmp_map.end(); it++)
        {
            if (search_place[it->first] > 0 || topo_tmp[tmp_place][it->first] == it->second)//搜过或者不够
            {
                cur_place++;
                continue;
            }
            else if (cur_place >= search_place[tmp_place])//满足要求
            {
                search_place[tmp_place] = cur_place+1;
                tmp_place = it->first;
                search_stack.push_back(tmp_place);
                flag_no_place = false;
                int i_row = tmp_place / cols;
                int j_col = tmp_place % cols;
                if (seam_place[i_row] < j_col)
                    seam_place[i_row] = j_col;
                break;
            }
            cur_place++;
        }

        if (flag_no_place)
        {
            //回退
            if (tmp_place == s)//结束了
                break;
            else
            {
                search_stack.pop_back();
                tmp_place = search_stack.at(search_stack.size()-1);
            }
        }
    }
    for (int i = 0; i < rows; i++)
        printf("%d ", seam_place[i]);
    printf("\n");
    Mat showImage1(rows, cols, inputImage.type());
    inputImage.copyTo(showImage1);  //复制
    for (int i = 0; i < rows; i++)
    {
        showImage1.at<Vec3b>(i,seam_place[i])[0] = 0;
        showImage1.at<Vec3b>(i,seam_place[i])[1] = 0;
        showImage1.at<Vec3b>(i,seam_place[i])[2] = 255;
    }
    //tmpImage.copyTo(outputImage);
    imshow("Carving Window", showImage1);
    waitKey();

    //--------->>>>>删除对应边
} 