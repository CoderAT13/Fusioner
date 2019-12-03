#include "Fusioner.h"


#define PIC_PATH "../pic/aruco6/"
#define PIC_PATH_2 "../pic/aruco3/"
#define PIC_ROOT "../pic/"

#define IDEBUG
#define DEBUG

string pic[] = {"127.jpg","186.jpg","221.jpg","220.jpg"};

Scalar color[] = {Scalar(255,255,255), Scalar(255,255,0), Scalar(0,255,0),Scalar(255,0,255),Scalar(0,0,255)};

Fusioner fusioner;
vector<Mat> s_srcs(4);
Mat res;

#ifdef IDEBUG
vector<vector<Point> > tracks(5);

void onMouse(int event, int x, int y, int flags, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            tracks[4].push_back(Point2f(x,y));
            for(int i = 0; i < 4; i++){
                Point2f p = fusioner.getDstPos(Point2f(x,y), i, true);
                if(p.x >= 0 && p.y >= 0 && p.x < S_WIDTH && p.y < S_HEIGHT){
                    tracks[i].push_back(p);
                    circle(s_srcs[i], p, 3, color[i], -1);
                }else{
                    if(tracks[i].size() > 0){
                        auto t = tracks[i].begin();
                        tracks[i].erase(t);
                    }
                }
            }
            for(int i = 0; i < 5; i++){
                if (tracks[i].size() > 1 && i < 4){
                    line(s_srcs[i], tracks[i][1], tracks[i][0], color[i], 1);
                    auto t = tracks[i].begin();
                    tracks[i].erase(t);
                }else if(tracks[i].size() > 1 && i == 4){
                    line(*im, tracks[i][tracks[i].size()-1], tracks[i][tracks[i].size()-2], color[i], 1);
                }
                    
            }
            circle(*im,Point(x,y),3,color[4],-1);
            imshow("res",*im);
            for(int i = 0; i < 4; i++){
                imshow(to_string(i), s_srcs[i]);
            }
            waitKey(1);
            break;
    }
}
#endif

#ifdef DEBUG
void onMouse0(int event, int x, int y, int flags, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            circle(*im,Point2f(x,y),3,color[0],-1);
            circle(res,fusioner.getDstPos(Point2f(x,y),0),3,color[0],-1);
            imshow("0",*im);
            imshow("res", res);
            waitKey(1);
            break;
    }
}
void onMouse1(int event, int x, int y, int flags, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            LOG(fusioner.getDstPos(Point2f(x,y),1));
            circle(*im,Point2f(x,y),3,color[1],-1);
            circle(res,fusioner.getDstPos(Point2f(x,y),1),3,color[1],-1);
            imshow("1",*im);
            imshow("res", res);
            waitKey(1);
            break;
    }
}
void onMouse2(int event, int x, int y, int flags, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            circle(*im,Point2f(x,y),3,color[2],-1);
            circle(res,fusioner.getDstPos(Point2f(x,y),2),3,color[2],-1);
            imshow("2",*im);
            imshow("res", res);
            waitKey(1);
            break;
    }
}
void onMouse3(int event, int x, int y, int flags, void* param){
    Mat *im = reinterpret_cast<Mat*>(param);
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            circle(*im,Point2f(x,y),3,color[3],-1);
            circle(res,fusioner.getDstPos(Point2f(x,y),3),3,color[3],-1);
            imshow("3",*im);
            imshow("res", res);
            waitKey(1);
            break;
    }
}
#endif


int main(int argc, char*argv[]){
    if(argc < 3){
        LOG("[Error] argv not enough");
        LOG("[Help] Usage:");
        LOG("./main [type] [input image directory] [change image directory]");
        LOG("[Help] Example:");
        LOG("./main DEBUG aruco6");
        LOG("./main IDEBUG aruco6");
        LOG("./main change aruco6 aruco3");
        exit(-1);
    }
    vector<Mat> srcs;
    for(int i = 0; i < 4; i++){
        //Mat t = imread(PIC_PATH+pic[i]);
        Mat t = imread(PIC_ROOT + string(argv[2]) + "/" + pic[i]);
        if(t.empty()){
            LOG(string("[Error] no such file: ") + PIC_ROOT + string(argv[2]) + "/" + pic[i]);
            exit(-1);
        }
        resize(t, s_srcs[i], Size(S_WIDTH, S_HEIGHT));
        srcs.push_back(t);
    }
    fusioner.init(srcs);
    res = fusioner.getResultImage();
    if (string(argv[1]) == "IDEBUG"){
        #ifdef IDEBUG
            namedWindow("res");
            cv::setMouseCallback("res",onMouse,reinterpret_cast<void*> (&res));
        #endif
            for(int i = 0; i < 4; i++){
                imshow(to_string(i), s_srcs[i]);
            }
            imshow("res",res);
            waitKey(0);
    } else if (string(argv[1]) == "DEBUG"){
        #ifdef DEBUG
            namedWindow("0");
            cv::setMouseCallback("0",onMouse0,reinterpret_cast<void*> (&s_srcs[0]));
            namedWindow("1");
            cv::setMouseCallback("1",onMouse1,reinterpret_cast<void*> (&s_srcs[1]));
            namedWindow("2");
            cv::setMouseCallback("2",onMouse2,reinterpret_cast<void*> (&s_srcs[2]));
            namedWindow("3");
            cv::setMouseCallback("3",onMouse3,reinterpret_cast<void*> (&s_srcs[3]));
            for(int i = 0; i < 4; i++){
                imshow(to_string(i), s_srcs[i]);
            }
            imshow("res",res);
            waitKey(0);
        #endif
    }
    else if (string(argv[1]) == "change"){
        fusioner.showResult();
        for(int i = 0; i < 4; i++){
       //Mat t = imread(PIC_PATH_2+pic[i]);
        Mat t = imread(PIC_ROOT + string(argv[3]) + "/" + pic[i]);
            srcs.push_back(t);
        }
        fusioner.changeImage(srcs);
        fusioner.showResult();
    }


    // LOG(computeAngle(arucos[1],arucos[2]));
    // warpRotation(arucos[2].image, arucos[2].image, computeAngle(arucos[1],arucos[2]));
    return 0;
}