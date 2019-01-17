#include "Improved_seam.hpp"

using namespace std;

//300x200x100
unordered_map<int, int> topo_max[MAX_SIZE];//起始点，结束点和对应上线
// map<int, int> topo_tmp[MAX_SIZE];
short search_place[MAX_SIZE+1];//知道搜到第几个了
int topo_record[MAX_SIZE][10] = {0};
short topo_top[MAX_SIZE] = {0};
vector<int> search_stack;//记录一路上的顺序
int search_point[MAX_SIZE] = {0};
int zero_fuck = 0;

int get_id(int i, int j, int cols)
{
    return (i - 1) * cols + j - 1;
} 

void print_stack()
{
    for (int i = 0; i < search_stack.size(); i++)
        printf("%d, ", search_stack.at(i));
    printf("\n");
}

void sap_dinic_flow (int s, int t, int rows, int cols)
{
    //--------->>>>>手写神搜
    int max_flow = 0;
    int tmp_place = s;//当前节点
    int min_flow = INFMAX;//记录路径中最小的
    bool flag_no_place = false;//是否没路可走了，回退
    memset(search_place, 0, sizeof(short)*MAX_SIZE);
    memset(search_point, 0, sizeof(int)*MAX_SIZE);
    search_stack.push_back(s);

    //int d[rows][cols] = {0};
    int d[rows*cols+2] = {0};
    memset(d, 0, sizeof(int)*(rows*cols+2));
    //初始化最短增广路径
    queue<int> bfs_points;
    for (int i = 0; i < rows; i++)
    {
        int tmp_id = get_id(i+1, cols, cols);
        bfs_points.push(tmp_id);
        d[tmp_id] = 1;
        search_place[d[tmp_id]]++;
    }
    while (bfs_points.size() > 0)
    {
        tmp_place = bfs_points.front();
        int ii = tmp_place / cols;
        int jj = tmp_place % cols;
        vector<int> to_search_list;
        to_search_list.push_back(get_id(ii, jj, cols));
        to_search_list.push_back(get_id(ii, jj+1, cols));
        to_search_list.push_back(get_id(ii, jj+2, cols));
        to_search_list.push_back(get_id(ii+1, jj, cols));
        to_search_list.push_back(get_id(ii+1, jj+2, cols));
        to_search_list.push_back(get_id(ii+2, jj, cols));
        to_search_list.push_back(get_id(ii+2, jj+1, cols));
        to_search_list.push_back(get_id(ii+2, jj+2, cols));
        for (int i = 0; i < to_search_list.size(); i++)
        {
            int tmp = to_search_list.at(i);
            if (tmp < 0 || tmp > rows * cols - 1)
                continue;
            else if (topo_max[tmp].count(tmp_place) > 0)//没必要的判定
            {
                if (d[tmp] == 0 && topo_max[tmp][tmp_place] > 0)//新的点
                {
                    d[tmp] = d[tmp_place] + 1;
                    search_place[d[tmp]]++;
                    bfs_points.push(tmp);
                }
            }
        }
        bfs_points.pop();
    }

    //开始搞
    tmp_place = s;
    search_point[s] = s;
    int next_place = 0, min_d = INFMAX;
    for (auto it = topo_max[tmp_place].begin(); it != topo_max[tmp_place].end(); it++)
    {
        if (it->second < min_d)
        {
            min_d = it->second;
            next_place = it->first;
        }
    }
    d[tmp_place] = min_d;
    while (1)
    {
        //printf("tmp_place: %d\n", tmp_place);
        flag_no_place = true;
        min_d = INFMAX;
        for (auto it = topo_max[tmp_place].begin(); it != topo_max[tmp_place].end(); it++)
        {
            int target_place = it->first;
            if (topo_max[tmp_place][target_place] > 0)
            {
                min_d = min(min_d, d[target_place]);
                if (d[tmp_place] == d[target_place] + 1)
                {
                    flag_no_place = false;
                    search_point[target_place] = tmp_place;
                    tmp_place = target_place;
                    break;
                }
            } 
        }
        if (flag_no_place)
        {
            search_place[d[tmp_place]]--;
            if (search_place[d[tmp_place]] == 0)
                break;
            d[tmp_place] = min_d + 1;
            search_place[d[tmp_place]]++;
            tmp_place = search_point[tmp_place];
            continue;
        }
        if (tmp_place % cols == cols - 1)//到终点了
        {
            int flow_place = tmp_place;
            while (search_point[flow_place] != s)
            {
                min_flow = min(min_flow, topo_max[search_point[flow_place]][flow_place]);
                flow_place = search_point[flow_place];
            }
            flow_place = tmp_place;
            while (search_point[flow_place] != s)
            {
                topo_max[search_point[flow_place]][flow_place] -= min_flow;
                topo_max[flow_place][search_point[flow_place]] += min_flow;
                flow_place = search_point[flow_place];
            }

            //同时重启s
            min_d = INFMAX;
            next_place = 0;
            for (auto it = topo_max[s].begin(); it != topo_max[s].end(); it++)
            {
                int target_place = it->first;
                if (min_d > d[target_place])
                {
                    min_d = d[target_place];
                    next_place = it->first;
                }
            }
            tmp_place = s;
            search_point[next_place] = tmp_place;
            tmp_place = next_place;
            
            //恢复原状
            max_flow += min_flow;
            //printf("now max_flow: %d\n", max_flow);
            // namedWindow("???");
            // waitKey();
            min_flow = INFMAX;
            continue;
        }
    }
    printf("max flow is: %d\n", max_flow);
}

void edmonds_karp_flow (int s, int t)
{
    //--------->>>>>手写神搜
    int max_flow = 0;
    int tmp_place = s;//当前节点
    int min_flow = INFMAX;//记录路径中最小的
    bool flag_no_place = false;//是否没路可走了，回退
    memset(search_place, 0, sizeof(short)*MAX_SIZE);
    search_stack.push_back(s);
    while (1)
    {     
        unordered_map<int, int> tmp_map = topo_max[tmp_place];
        int cur_place = search_place[tmp_place];
        flag_no_place = true;
        if (topo_max[tmp_place].count(t) > 0)//直接进终点，这样简便
        {
            tmp_place = t;
            search_stack.push_back(tmp_place);
            flag_no_place = false;
        }
        else
        {
            while (cur_place < topo_top[tmp_place])
            {
                int target_place = topo_record[tmp_place][cur_place];
                if (search_place[target_place] > 0)//搜过
                {
                    cur_place++;
                    continue;
                }
                //满了
                else if (topo_max[tmp_place][target_place] == 0)
                {
                    cur_place++;
                    continue;
                }
                else if (cur_place >= search_place[tmp_place])//满足要求
                {
                    search_place[tmp_place] = cur_place+1;
                    tmp_place = target_place;
                    search_stack.push_back(tmp_place);
                    flag_no_place = false;
                    break;
                }
                else
                    cur_place++;
            }
        }
        if (flag_no_place)//回退
        {
            if (tmp_place == s)//结束了
                break;
            else
            {
                search_stack.pop_back();
                tmp_place = search_stack.at(search_stack.size()-1);
            }
        }

        if (tmp_place == t)//结束了，做收尾工作
        {
            for (int i = 0; i < search_stack.size() - 1; i++)
            {
                int s_place = search_stack.at(i);
                int t_place = search_stack.at(i+1);
                if (min_flow > topo_max[s_place][t_place])
                    min_flow = topo_max[s_place][t_place];
            }
            for (int i = 0; i < search_stack.size() - 1; i++)
            {
                int s_place = search_stack.at(i);
                int t_place = search_stack.at(i+1);
                topo_max[s_place][t_place] -= min_flow;
                topo_max[t_place][s_place] += min_flow;
            }
            //恢复原状
            max_flow += min_flow;
            //printf("now max_flow: %d\n", max_flow);
            memset(search_place, 0, sizeof(short)*MAX_SIZE);
            search_stack.clear();
            search_stack.push_back(s);
            tmp_place = s;
            min_flow = INFMAX;
            continue;
        }     
    }
    printf("max flow is: %d\n", max_flow);
}

void improved_seam_carving (Mat& inputImage, Mat& outputImage)
{
    int rows = inputImage.rows;
    int cols = inputImage.cols;
    int s = rows * cols;
    int t = rows * cols + 1;
    int total_points = rows * cols + 2;
    clock_t start_time, time1, time2, time3, time4;
    start_time = clock();

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
    time1 = clock();
    
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
    for (int i = 0; i < total_points; i++)
    {
        unordered_map<int, int> tmp_map = topo_max[i];
        int topo_place = 0;
        for (auto it1 = tmp_map.begin(); it1 != tmp_map.end(); it1++)
        {
            topo_record[i][topo_place++] = it1->first;
            total++;
        }
        topo_top[i] = topo_place;
    }
    printf("rows: %d, cols: %d\n", rows, cols);
    printf("amount: %d\n", amount);
    printf("total: %d\n", total);
    printf("1: %d, 2: %d, 3: %d\n", topo_max[150000][150001], topo_max[0][1], topo_max[22][21]);
    time2 = clock();
    
    //--------->>>>>手写神搜
    sap_dinic_flow(s, t, rows, cols);
    //edmonds_karp_flow(s, t);
    time3 = clock();

    //--------->>>>>找到最小割
    total = 0;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols-1; j++)
        {
            int a1 = get_id(i+1, j+1, cols);
            if (topo_max[a1][a1+1] == 0)
                total++;
            if (i < rows - 1)
            {
                if (topo_max[a1][a1+cols] == topo_max[a1][a1+cols] || topo_max[a1+cols][a1] == topo_max[a1+cols][a1])
                    total++;
            }
        }
    printf("place to cut: %d\n", total);

    queue<int> bfs_points;
    while(!bfs_points.empty())
        bfs_points.pop();
    int d[rows*cols] = {0};
    int tmp_place;
    int seam_place[1000] = {0};
    memset(d, 0, sizeof(int)*rows*cols);
    memset(seam_place, 0, sizeof(int)*1000);
    for (int i = 0; i < rows; i++)
    {
        bfs_points.push(get_id(i+1, 1, cols));
        d[get_id(i+1, 1, cols)] = 1;
    }
    while (bfs_points.size() > 0)
    {
        tmp_place = bfs_points.front();
        int ii = tmp_place / cols;
        int jj = tmp_place % cols;
        seam_place[ii] = max(seam_place[ii], jj);
        for (auto it = topo_max[tmp_place].begin(); it != topo_max[tmp_place].end(); it++)
        {
            int tmp = it->first;
            if (d[tmp] == 0 && topo_max[tmp_place][tmp] > 0)
            {
                d[tmp] = d[tmp_place] + 1;
                bfs_points.push(tmp);
            }
        }
        bfs_points.pop();
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
    time4 = clock();
    imshow("Carving Window", showImage1);
    waitKey();

    //--------->>>>>删除对应边

    printf("载入图片： %f s\n", (double)(time1 - start_time) / CLOCKS_PER_SEC);
    printf("构造图边： %f s\n", (double)(time2 - time1) / CLOCKS_PER_SEC);
    printf("进行神搜： %f s\n", (double)(time3 - time2) / CLOCKS_PER_SEC);
    printf("寻找割集： %f s\n", (double)(time4 - time3) / CLOCKS_PER_SEC);
    printf("总共时间： %f s\n", (double)(time4 - start_time) / CLOCKS_PER_SEC);
} 