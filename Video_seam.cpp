#include "Video_seam.hpp"

using namespace std;

Mat inputImage[FRAME_NUM];
Mat tmpImage[FRAME_NUM];
Mat outputImage[FRAME_NUM];
//tmp到output

int mode_j[MAXI_SIZE];
int del_i[MAXI_SIZE];
int del_k[MAXI_SIZE];
int topo_tmp[MAXI_SIZE+1][27];
vector<int> topo_re[MAXI_SIZE];
int seam_place[350][100];

int search_place[MAXI_SIZE+22];
int search_point[MAXI_SIZE] = {0};

int s_order[24000];
vector<int> random_num;
bool is_random[25022] = {false};
int d[MAXI_SIZE] = {0};

int globals, globalt, globalrow, globalcol, globallen;

int get_id(int i, int j, int k)//这回不变了
{
    if (i < 0 || i >= globalrow || j < 0 || j >= globalcol || k < 0 || k >= globallen)
        return INFMAXI;
    return k * globalrow * globalcol + i * globalcol + j;
}

int get_place(int tmp, int target)
{
    if (target == globalt)
        return 13;
    int i1 = del_i[tmp];
    int j1 = mode_j[tmp];
    int k1 = del_k[tmp];
    int i2 = del_i[target];
    int j2 = mode_j[target];
    int k2 = del_k[target];
    return 9 * (k1 + 1 - k2) + 3 * (i1 + 1 - i2) + j1 + 1 - j2;
}

void init_result()
{
    for (int i = 0; i < MAXI_SIZE; i++)
    {
        del_i[i] = (i / globalcol) % globalrow;
        mode_j[i] = i % globalcol;
        del_k[i] = i / (globalrow * globalcol);
        topo_re[i].clear();
    }

    memset(topo_tmp, 0, sizeof(int)*MAXI_SIZE*27);
    memset(seam_place, 0, sizeof(int)*350*100);

    memset(s_order, 0, sizeof(int)*24000);
    memset(is_random, 0, sizeof(bool)*25000);
    srand((int)time(0));
    random_num.clear();
    while(random_num.size() < globalrow*globallen)
    {
        int newnum = rand() % (globalrow*globallen);
        if (is_random[newnum])
            continue;
        else
        {
            random_num.push_back(newnum);
            is_random[newnum] = true;
        }
    }
}

void sap_dinic_flow ()
{
    //--------->>>>>手写神搜
    printf("begin:\n");
    int rows = globalrow, cols = globalcol, len = globallen;
    int s = globals, t = globalt;
    long long max_flow = 0;
    int tmp_place = s;//当前节点
    int min_flow = INFMAXI;//记录路径中最小的
    bool flag_no_place = false;//是否没路可走了，回退
    memset(search_place, 0, sizeof(int)*MAXI_SIZE);
    memset(search_point, 0, sizeof(int)*MAXI_SIZE);
    memset(d, 0, sizeof(int)*MAXI_SIZE);
    //初始化最短增广路径
    queue<int> bfs_points;
    for (int i = 0; i < rows; i++)
        for(int k = 0; k < len; k++)
    {
        int tmp_id = get_id(i, cols-1, k);
        bfs_points.push(tmp_id);
        d[tmp_id] = 1;
        search_place[d[tmp_id]]++;
    }

    while (bfs_points.size() > 0)
    {
        tmp_place = bfs_points.front();
        int ii = del_i[tmp_place];
        int jj = mode_j[tmp_place];
        int kk = del_k[tmp_place];
        vector<int> to_search_list;
        to_search_list.push_back(get_id(ii-1, jj-1, kk-1));
        to_search_list.push_back(get_id(ii-1, jj, kk-1));
        to_search_list.push_back(get_id(ii-1, jj+1, kk-1));
        to_search_list.push_back(get_id(ii, jj-1, kk-1));
        to_search_list.push_back(get_id(ii, jj+1, kk-1));
        to_search_list.push_back(get_id(ii+1, jj-1, kk-1));
        to_search_list.push_back(get_id(ii+1, jj, kk-1));
        to_search_list.push_back(get_id(ii+1, jj+1, kk-1));
        to_search_list.push_back(get_id(ii-1, jj-1, kk));
        to_search_list.push_back(get_id(ii-1, jj, kk));
        to_search_list.push_back(get_id(ii-1, jj+1, kk));
        to_search_list.push_back(get_id(ii, jj-1, kk));
        to_search_list.push_back(get_id(ii, jj+1, kk));
        to_search_list.push_back(get_id(ii+1, jj-1, kk));
        to_search_list.push_back(get_id(ii+1, jj, kk));
        to_search_list.push_back(get_id(ii+1, jj+1, kk));
        to_search_list.push_back(get_id(ii-1, jj-1, kk+1));
        to_search_list.push_back(get_id(ii-1, jj, kk+1));
        to_search_list.push_back(get_id(ii-1, jj+1, kk+1));
        to_search_list.push_back(get_id(ii, jj-1, kk+1));
        to_search_list.push_back(get_id(ii, jj+1, kk+1));
        to_search_list.push_back(get_id(ii+1, jj-1, kk+1));
        to_search_list.push_back(get_id(ii+1, jj, kk+1));
        to_search_list.push_back(get_id(ii+1, jj+1, kk+1));
        to_search_list.push_back(get_id(ii, jj, kk-1));
        to_search_list.push_back(get_id(ii, jj, kk+1));

        for (int i = 0; i < to_search_list.size(); i++)
        {
            int tmp = to_search_list.at(i);
            if (tmp < 0 || tmp > rows * cols * len - 1)
                continue;
            if (d[tmp] == 0 && topo_tmp[tmp][get_place(tmp, tmp_place)] > 0)//新的点
            {
                d[tmp] = d[tmp_place] + 1;
                search_place[d[tmp]]++;
                bfs_points.push(tmp);
            }
        }
        bfs_points.pop();
    }
    printf("there?\n");
    //开始搞
    tmp_place = s;
    search_point[s] = s;
    int next_place = 0, min_d = INFMAXI;
    for (int i = 0; i < topo_re[tmp_place].size(); i++)
    {
        min_d = min(min_d, d[topo_re[tmp_place].at(i)]);
    }
    d[tmp_place] = min_d;
    while (1)
    {
        //printf("tmp_place: %d\n", tmp_place);
        flag_no_place = true;
        min_d = INFMAXI;
        for (int i = 0; i < topo_re[tmp_place].size(); i++)
        {
            int target_place = topo_re[tmp_place].at(i);
            if (tmp_place == s)
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
            else
            {
                if (topo_tmp[tmp_place][get_place(tmp_place, target_place)] > 0)
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
        if (mode_j[tmp_place] == cols - 1)//到终点了
        {
            int flow_place = tmp_place;
            int zeros, zerot;
            int total = 0;
            while (search_point[flow_place] != s)
            {
                //total++;
                int pre = search_point[flow_place];
                if (min_flow > topo_tmp[pre][get_place(pre, flow_place)])
                {
                    min_flow = topo_tmp[pre][get_place(pre, flow_place)];
                    // zeros = pre;
                    // zerot = flow_place;
                }
                flow_place = search_point[flow_place];
                
            }
            flow_place = tmp_place;
            while (search_point[flow_place] != s)
            {
                int pre = search_point[flow_place];
                topo_tmp[pre][get_place(pre, flow_place)] -= min_flow;
                topo_tmp[flow_place][get_place(flow_place, pre)] += min_flow;
                flow_place = pre;
            }

            //同时重启s
            min_d = INFMAXI;
            next_place = 0;

            for (int i = 0; i < topo_re[s].size(); i++)
            {
                int target_place = topo_re[s].at(i);
                if (min_d > d[target_place])
                {
                    min_d = d[target_place];
                    next_place = target_place;
                }
            }
            tmp_place = s;
            search_point[next_place] = tmp_place;
            tmp_place = next_place;
            
            //恢复原状
            max_flow += min_flow;
            //printf("now max_flow: %d\n", max_flow);
            //printf("s: %d, t: %d\n", zeros, zerot);
            //printf("total: %d\n", total);
            min_flow = INFMAXI;
            continue;
        }
    }
    printf("max flow is: %lld\n", max_flow);
}

void del_one_col ()
{
    for (int k = 0; k < globallen; k++)
    {
        Mat image2(globalrow, globalcol-1, tmpImage[0].type());
        Mat image1;
        tmpImage[k].copyTo(image1);
        //printf("%d\n", k);
        for (int i = 0; i < globalrow; i++)
        {
            int kk = seam_place[i][k];
            //printf("%d\n", kk);
            for (int j = 0; j < kk; j++)  //删除列前的元素复制
            {
                image2.at<Vec3b>(i,j)[0] = image1.at<Vec3b>(i,j)[0];  // 8U 类型的 RGB 彩色图像 [i] 存放bgr的值
                image2.at<Vec3b>(i,j)[1] = image1.at<Vec3b>(i,j)[1];
                image2.at<Vec3b>(i,j)[2] = image1.at<Vec3b>(i,j)[2];
            }
            for (int j = kk; j < image2.cols-1; j++)  //复制删除列后的元素 最后一列默认不处理
            {
                if (j == image2.cols - 1)
                    continue;
                image2.at<Vec3b>(i,j)[0] = image1.at<Vec3b>(i,j+1)[0];
                image2.at<Vec3b>(i,j)[1] = image1.at<Vec3b>(i,j+1)[1];
                image2.at<Vec3b>(i,j)[2] = image1.at<Vec3b>(i,j+1)[2];
            }
        }
        image2.copyTo(outputImage[k]);
    }
}

void video_seam_one()
{
    clock_t start_time, time1, time2, time3, time4, time5;
    start_time = clock();
    globalrow = tmpImage[0].rows;
    globalcol = tmpImage[0].cols;
    globallen = FRAME_NUM;
    globals = globalrow * globalcol * globallen;
    globalt = globalrow * globalcol * globallen + 1;
    int total_points = globalt + 1;
    
    //----->>>>>输入图片
    init_result();
    Mat image_gray[FRAME_NUM];
    for (int i = 0; i < FRAME_NUM; i++)
    {
        Mat image_gray1(globalrow, globalcol, CV_8U, Scalar(0));
        Mat image = tmpImage[i];
        cvtColor(image, image_gray1, CV_BGR2GRAY);
        image_gray1.copyTo(image_gray[i]);
    }
    Mat bigimages[FRAME_NUM+2];
    Mat tmpImage1(globalrow+2, globalcol+2, CV_32F, Scalar(0));
    for (int i = 0; i < FRAME_NUM+2; i++)
        tmpImage1.copyTo(bigimages[i]);
    for (int i = 0; i < globalrow; i++)
        for (int j = 0; j < globalcol; j++)
            for (int k = 0; k < globallen; k++)
            {
                bigimages[k+1].at<float>(i+1, j+1) = float(image_gray[k].at<uchar>(i, j));
            }
    for (int i = 1; i <= globalrow; i++)
        for (int j = 1; j <= globalcol; j++)
        {
            bigimages[0].at<float>(i, j) = bigimages[1].at<float>(i, j);
            bigimages[globallen+1].at<float>(i, j) = bigimages[globallen].at<float>(i, j);
        }
    for (int j = 1; j <= globalcol; j++)
        for (int k = 1; k <= globallen; k++)
        {
            bigimages[k].at<float>(0, j) = bigimages[k].at<float>(1, j);
            bigimages[k].at<float>(globalrow+1, j) = bigimages[k].at<float>(globalrow, j);
        }
    for (int i = 1; i <= globalrow; i++)
        for (int k = 1; k <= globallen; k++)
        {
            bigimages[k].at<float>(i, 0) = bigimages[k].at<float>(i, 1);
            bigimages[k].at<float>(i, globalcol+1) = bigimages[k].at<float>(i, globalcol);
        }
    time1 = clock();
    //----->>>>>构造边集
    for (int i = 0; i < globals; i++)
    {
        int ii = del_i[i];
        int jj = mode_j[i];
        int kk = del_k[i];
        vector<int> to_search_list;
        to_search_list.push_back(get_id(ii-1, jj-1, kk-1));
        to_search_list.push_back(get_id(ii-1, jj, kk-1));
        to_search_list.push_back(get_id(ii-1, jj+1, kk-1));
        to_search_list.push_back(get_id(ii, jj-1, kk-1));
        to_search_list.push_back(get_id(ii, jj+1, kk-1));
        to_search_list.push_back(get_id(ii+1, jj-1, kk-1));
        to_search_list.push_back(get_id(ii+1, jj, kk-1));
        to_search_list.push_back(get_id(ii+1, jj+1, kk-1));
        to_search_list.push_back(get_id(ii-1, jj-1, kk));
        to_search_list.push_back(get_id(ii-1, jj, kk));
        to_search_list.push_back(get_id(ii-1, jj+1, kk));
        to_search_list.push_back(get_id(ii, jj-1, kk));
        to_search_list.push_back(get_id(ii, jj+1, kk));
        to_search_list.push_back(get_id(ii+1, jj-1, kk));
        to_search_list.push_back(get_id(ii+1, jj, kk));
        to_search_list.push_back(get_id(ii+1, jj+1, kk));
        to_search_list.push_back(get_id(ii-1, jj-1, kk+1));
        to_search_list.push_back(get_id(ii-1, jj, kk+1));
        to_search_list.push_back(get_id(ii-1, jj+1, kk+1));
        to_search_list.push_back(get_id(ii, jj-1, kk+1));
        to_search_list.push_back(get_id(ii, jj+1, kk+1));
        to_search_list.push_back(get_id(ii+1, jj-1, kk+1));
        to_search_list.push_back(get_id(ii+1, jj, kk+1));
        to_search_list.push_back(get_id(ii+1, jj+1, kk+1));
        to_search_list.push_back(get_id(ii, jj, kk-1));
        to_search_list.push_back(get_id(ii, jj, kk+1));
        for (int j = 0; j < to_search_list.size(); j++)
        {
            int tmp = to_search_list.at(j);
            if (tmp < 0 || tmp > globals - 1)
                continue;
            else
                topo_re[i].push_back(tmp);
        }
    }

    for (int i = 1; i <= globalrow; i++)
        for (int j = 1; j <= globalcol; j++)
            for (int k = 1; k <= globallen; k++)
    {
        int a1 = get_id(i-1, j-1, k-1);
        int a2 = get_id(i-1, j, k-1);
        int a3 = get_id(i, j-1, k-1);
        int a4 = get_id(i, j, k-1);
        int a5 = get_id(i-1, j-1, k);
        int a6 = get_id(i-1, j, k);
        float i_1j, ij_1, ij1, i1j, jk1, jk_1;
        i_1j = bigimages[k].at<float>(i-1, j);
        ij_1 = bigimages[k].at<float>(i, j-1);
        ij1 = bigimages[k].at<float>(i, j+1);
        i1j = bigimages[k].at<float>(i+1, j);
        jk1 = bigimages[k+1].at<float>(i, j);
        jk_1 = bigimages[k-1].at<float>(i, j);
        if (a1 != INFMAXI && a2 != INFMAXI)
        {
            topo_tmp[a2][get_place(a2, a1)] = INFMAXI;
            topo_tmp[a1][get_place(a1, a2)] = abs(ij1 - ij_1);
        }
        if (a3 != INFMAXI && a2 != INFMAXI)
            topo_tmp[a2][get_place(a2, a3)] = INFMAXI;
        if (a1 != INFMAXI && a4 != INFMAXI)
            topo_tmp[a4][get_place(a4, a1)] = INFMAXI;
        if (a1 != INFMAXI && a6 != INFMAXI)
            topo_tmp[a6][get_place(a6, a1)] = INFMAXI;
        if (a5 != INFMAXI && a2 != INFMAXI)
            topo_tmp[a2][get_place(a2, a5)] = INFMAXI;
        if (a3 != INFMAXI && a1 != INFMAXI)
        {
            topo_tmp[a1][get_place(a1, a3)] = abs(i_1j - ij_1);
            topo_tmp[a3][get_place(a3, a1)] = abs(i1j - ij_1);
        }
        if (a5 != INFMAXI && a1 != INFMAXI)
        {
            topo_tmp[a1][get_place(a1, a5)] = abs(jk_1 - ij_1);
            topo_tmp[a5][get_place(a5, a1)] = abs(jk1 - ij_1);
        }
    }

    for(int i = 1; i <= globalrow; i++)
        for(int k = 1; k <= globallen; k++)
        {
            int a1 = get_id(i-1, 0, k-1);
            int a2 = get_id(i-1, globalcol-1, k-1);
            s_order[(i-1)*globallen+k-1] = a1;
            topo_re[a2].push_back(globalt);
        }
    while (random_num.size() > 0)
    {
        int place = random_num.back();
        topo_re[globals].push_back(s_order[place]);
        random_num.pop_back();
    }
    // int amount = 0;
    // amount += globallen * (globalrow-1) * (globalcol-1) * 2;
    // amount += globalcol * (globalrow-1) * (globallen-1) * 2;
    // amount += globalrow * (globallen-1) * (globalcol-1) * 2;
    // amount += (globalrow-1) * (globalcol-1) * 2;
    // amount += (globallen-1) * (globalcol-1) * 2;
    // printf("amount: %d\n", amount);
    // int total = 0;
    // for (int i = 0; i < globals; i++)
    // {
    //     for (int j = 0; j < topo_re[i].size(); j++)
    //     {
    //         int target = topo_re[i].at(j);
    //         if (topo_tmp[i][get_place(i, target)] > 0)
    //             total++;
    //     }
    // }
    // printf("total: %d\n", total);
    printf("rows: %d, cols: %d\n", globalrow, globalcol);
    time2 = clock();

    //----->>>>>神搜
    sap_dinic_flow();
    time3 = clock();
    //----->>>>>最小割
    queue<int> bfs_points;
    while(!bfs_points.empty())
        bfs_points.pop();
    int tmp_place;
    
    memset(d, 0, sizeof(int)*MAXI_SIZE);
    memset(seam_place, 0, sizeof(int)*35000);
    for (int i = 0; i < globalrow; i++)
        for (int k = 0; k < globallen; k++)
    {
        bfs_points.push(get_id(i, 0, k));
        d[get_id(i, 0, k)] = 1;
    }
    while (bfs_points.size() > 0)
    {
        tmp_place = bfs_points.front();
        int ii = del_i[tmp_place];
        int jj = mode_j[tmp_place];
        int kk = del_k[tmp_place];
        seam_place[ii][kk] = max(seam_place[ii][kk], jj);
        for (int i = 0; i < topo_re[tmp_place].size(); i++)
        {
            int tmp = topo_re[tmp_place].at(i);
            if (d[tmp] == 0 && topo_tmp[tmp_place][get_place(tmp_place, tmp)] > 0)
            {
                d[tmp] = d[tmp_place] + 1;
                bfs_points.push(tmp);
            }
        }
        bfs_points.pop();
    }

    Mat showImage[FRAME_NUM];
    for (int k = 0; k < FRAME_NUM; k++)
    {
        Mat showImage1(globalrow, globalcol, tmpImage[0].type());
        tmpImage[k].copyTo(showImage1);
        for (int i = 0; i < globalrow; i++)
        {
            showImage1.at<Vec3b>(i,seam_place[i][k])[0] = 0;
            showImage1.at<Vec3b>(i,seam_place[i][k])[1] = 0;
            showImage1.at<Vec3b>(i,seam_place[i][k])[2] = 255;
        }
        showImage1.copyTo(showImage[k]);
    }
    int output_row = showImage[0].rows;
    int output_col = showImage[0].cols;
    for (int k = 0; k < FRAME_NUM; k++)
    {
        string name_of_chair = "";
        name_of_chair += "./video/bas/test1_";
        name_of_chair += to_string(k);
        name_of_chair += ".png";

        imwrite(name_of_chair, showImage[k]);
    }
    
    VideoWriter video("./video/yangben.avi", CV_FOURCC('M', 'J', 'P', 'G'), 
                15.0, Size(output_col, output_row));
    for (int i = FRAME_NUM - 1; i >= 0; i--)
    {
        Mat image = showImage[i];
        video << image;
    }
    
    time4 = clock();
    printf("delete?\n");
    //----->>>>>删除
    del_one_col();
    time5 = clock();
    //
    //

    printf("载入图片： %f s\n", (double)(time1 - start_time) / CLOCKS_PER_SEC);
    printf("构造图边： %f s\n", (double)(time2 - time1) / CLOCKS_PER_SEC);
    printf("进行神搜： %f s\n", (double)(time3 - time2) / CLOCKS_PER_SEC);
    printf("寻找割集： %f s\n", (double)(time4 - time3) / CLOCKS_PER_SEC);
    printf("总共时间： %f s\n", (double)(time5 - start_time) / CLOCKS_PER_SEC);
}

void video_seam_carving ()
{
    //----->>>>>读入视频，制作数据结构
    //string filename = "./video/golf.mov";
    string filename = "./video/basketball.avi";
    VideoCapture capture;
    capture.open(filename);
    double rate = capture.get(CV_CAP_PROP_FPS);
    int delay = cvRound(1000.000 / rate);
    printf("rate: %f, delay: %d\n", rate, delay);

    int rate_num = 0;
    while (true)
    {
        Mat frame;
	    capture >> frame;
        frame.copyTo(inputImage[rate_num]);
        frame.copyTo(tmpImage[rate_num]);
        rate_num++;
        if (frame.empty() || rate_num >= FRAME_NUM) 
            break;
    }
    //imwrite("./gray.png", tmpImage[0]);
    //int channel = tmpImage[82].channels();
    //printf("channels: %d\n", channel);
    printf("num: %d\n", rate_num);

    for(int i = 0; i < 50; i++)
    {
        video_seam_one();
        for (int k = 0; k < FRAME_NUM; k++)
        {
            outputImage[k].copyTo(tmpImage[k]);
        }
    }
    // for (int k = 0; k < FRAME_NUM; k++)
    // {
    //     tmpImage[k].copyTo(outputImage[k]);
    // }

    int output_row = outputImage[0].rows;
    int output_col = outputImage[0].cols;
    printf("row: %d, col: %d\n", output_row, output_col);
    VideoWriter video("./video/ans.avi", CV_FOURCC('M', 'J', 'P', 'G'), 
                rate, Size(output_col, output_row));
    for (int i = FRAME_NUM - 1; i >= 0; i--)
    {
        Mat image = outputImage[i];
        video << image;
    }
    
}