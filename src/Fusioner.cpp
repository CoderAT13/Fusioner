#include "Fusioner.h"


bool BE_QUICK = true;
float blend_strength = 5;
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
int expos_comp_nr_feeds = 1;
int expos_comp_nr_filtering = 2;
int expos_comp_block_size = 32;
string seam_find_type = "";
bool try_cuda = true;

vector<pair<int, int>> Fusioner::match(ImageAruco src1, ImageAruco src2)
{
    vector<pair<int, int>> match_idx;
    for (int i = 0; i < src1.ids.size(); i++)
    {
        int id = src2.findIds(src1.ids[i]);
        if (id == -1)
        {
            continue;
        }
        match_idx.push_back(pair<int, int>(i, id));
    }
    return match_idx;
}

double Fusioner::compute(ImageAruco& src0, ImageAruco& src1){
    vector<pair<int, int>> idxMatch = match(src0,src1);
    double angleSum = 0;
    double scale = 0;
    
    if (idxMatch.size() >= 2){
        for(int i = 1; i < idxMatch.size(); i++){
            int id_0 = idxMatch[i].first;
            int id_00 = idxMatch[i-1].first;
            int id_1 = idxMatch[i].second;
            int id_10 = idxMatch[i-1].second;
            Point2f v0,v1;
            {
                Point p0 = src0.corners[id_0][0];
                Point p1 = src0.corners[id_0][2];
                Point p2 = src0.corners[id_00][0];
                Point p3 = src0.corners[id_00][2];
                v0.x = ((p1.x + p0.x) - (p2.x + p3.x))*0.5;
                v0.y = ((p1.y + p0.y) - (p2.y + p3.y))*0.5;
                //line(src0.image,p0,p2,Scalar(0,0,255),3);
            }
            {
                Point p0 = src1.corners[id_1][0];
                Point p1 = src1.corners[id_1][2];
                Point p2 = src1.corners[id_10][0];
                Point p3 = src1.corners[id_10][2];
                v1.x = ((p1.x + p0.x) - (p2.x + p3.x))*0.5;
                v1.y = ((p1.y + p0.y) - (p2.y + p3.y))*0.5;
                //line(src1.image,p0,p2,Scalar(0,0,255),3);
            }
            double label = asin((v0.x*v1.y-v0.y*v1.x) / (sqrt(pow(v0.x,2)+pow(v0.y,2))*sqrt(pow(v1.x,2)+pow(v1.y,2))) );
            double angle = 360*acos(((v0.x*v1.x)+(v0.y*v1.y)) / (sqrt(pow(v0.x,2)+pow(v0.y,2))*sqrt(pow(v1.x,2)+pow(v1.y,2))))/(2*CV_PI);
            angle *= label < 0 ? -1 : 1;
            angleSum += angle;
            scale += sqrt(pow(v0.x,2)+pow(v0.y,2))/sqrt(pow(v1.x,2)+pow(v1.y,2));
        }
        angleSum /= idxMatch.size()-1;
        //scale /= idxMatch.size() -1;
        scale = 1;
    }else{
        for(int i = 0; i < idxMatch.size(); i++){
            int id_0 = idxMatch[i].first;
            int id_1 = idxMatch[i].second;
            Point2f v0,v1;
            {
                Point p0 = src0.corners[id_0][0];
                Point p2 = src0.corners[id_0][2];
                v0.x = p2.x-p0.x;
                v0.y = p2.y-p0.y;
                //line(src0.image,p0,p2,Scalar(0,0,255),3);
            }
            {
                Point p0 = src1.corners[id_1][0];
                Point p2 = src1.corners[id_1][2];
                
                v1.x = p2.x-p0.x;
                v1.y = p2.y-p0.y;
                //line(src1.image,p0,p2,Scalar(0,0,255),3);
            }
            double label = asin((v0.x*v1.y-v0.y*v1.x) / (sqrt(pow(v0.x,2)+pow(v0.y,2))*sqrt(pow(v1.x,2)+pow(v1.y,2))) );
            double angle = 360*acos(((v0.x*v1.x)+(v0.y*v1.y)) / (sqrt(pow(v0.x,2)+pow(v0.y,2))*sqrt(pow(v1.x,2)+pow(v1.y,2))))/(2*CV_PI);
            angle *= label < 0 ? -1 : 1;
            angleSum += angle;
            scale += sqrt(pow(v0.x,2)+pow(v0.y,2))/sqrt(pow(v1.x,2)+pow(v1.y,2));
        }
        angleSum /= idxMatch.size();
        //scale /= idxMatch.size();
        scale = 1;
    }
    
    
    resize(src1.image,src1.image,Size(),scale,scale);
    src1.scale = scale;
    src1.first_size = src1.image.size();
    return angleSum;
}

Rect Fusioner::findRoi(ImageAruco first, ImageAruco second){
    vector<pair<int, int>> idxMatch = match(first, second);
    double dx = 0, dy = 0;
    int x0 = first.roi.x;
    int y0 = first.roi.y;
    for(int i = 0; i < idxMatch.size(); i++){
        int id_0 = idxMatch[i].first;
        int id_1 = idxMatch[i].second;
        for(int j = 0; j < 4; j++){
            dx += first.corners[id_0][j].x - second.corners[id_1][j].x;
            dy += first.corners[id_0][j].y - second.corners[id_1][j].y;
        }
    }
    dx /= (idxMatch.size()*4);
    dy /= (idxMatch.size()*4);
    return Rect(x0 + dx, y0 + dy, second.size.width, second.size.height);
}

void Fusioner::getResult(){
    double time1 = static_cast<double>( getTickCount());
    LOG("[INFO] Preparing Data...");

    vector<Point> corners;
    vector<Size> sizes;
    vector<UMat> masks;
    vector<UMat> uimgs;
    vector<Mat> imgs;
    for(int i = 0; i < arucos.size(); i++){
        corners.push_back(Point(arucos[i].roi.x, arucos[i].roi.y));
        sizes.push_back(Size(arucos[i].roi.width, arucos[i].roi.height));
    }
    Rect origin_dstRoi = resultRoi(corners, sizes);
    dstScaleQ = 1;
    vector<Mat> arucosMask;
    for(int i = 0; i < arucos.size(); i++){
        Mat tmp = arucos[i].mask.clone();
        arucosMask.push_back(tmp);
    }
    if(BE_QUICK){
        double scaleX = 800.0/origin_dstRoi.width;
        double scaleY = 600.0/origin_dstRoi.height;
        double fin_scale;
        
        if (scaleX > scaleY){
            fin_scale = scaleY;
        }else{
            fin_scale =scaleX;
        }
        for(int i = 0; i < arucos.size(); i++){
            resize(arucosMask[i], arucosMask[i], Size(), fin_scale, fin_scale);
            arucos[i].scale = fin_scale;
        }
        dstScaleQ = fin_scale;
    }
    corners.clear();
    sizes.clear();
    for(int i = 0; i < arucos.size(); i++){
        corners.push_back(Point(arucos[i].roi.x * arucos[i].scale, arucos[i].roi.y * arucos[i].scale));
        sizes.push_back(Size(arucos[i].roi.width * arucos[i].scale, arucos[i].roi.height * arucos[i].scale));
        masks.push_back(UMat(arucosMask[i].size(),CV_8U));
        arucosMask[i].copyTo(masks[i]);
        
        // imshow("ad", arucos[i].mask);
        
        Mat rimage;
        warpAffine(arucos[i].image, rimage, arucos[i].R, arucos[i].size);
        resize(rimage, rimage, Size(), arucos[i].scale, arucos[i].scale);
        
        uimgs.push_back(UMat(rimage.size(), 16));
        rimage.copyTo(uimgs[i]);
        imgs.push_back(rimage);
    }



    LOG("[INFO] Compensating exposure...");

    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
    if (dynamic_cast<GainCompensator*>(compensator.get()))
    {
        GainCompensator* gcompensator = dynamic_cast<GainCompensator*>(compensator.get());
        gcompensator->setNrFeeds(expos_comp_nr_feeds);
    }

    if (dynamic_cast<ChannelsCompensator*>(compensator.get()))
    {
        ChannelsCompensator* ccompensator = dynamic_cast<ChannelsCompensator*>(compensator.get());
        ccompensator->setNrFeeds(expos_comp_nr_feeds);
    }

    if (dynamic_cast<BlocksCompensator*>(compensator.get()))
    {
        BlocksCompensator* bcompensator = dynamic_cast<BlocksCompensator*>(compensator.get());
        bcompensator->setNrFeeds(expos_comp_nr_feeds);
        bcompensator->setNrGainsFilteringIterations(expos_comp_nr_filtering);
        bcompensator->setBlockSize(expos_comp_block_size, expos_comp_block_size);
    }

    compensator->feed(corners, uimgs, masks);


    LOG("[INFO] Finding seams...");


    Ptr<SeamFinder> seam_finder;
    {
        if (seam_find_type == "no")
            seam_finder = makePtr<detail::NoSeamFinder>();
        else
        {
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
        }
    }

    vector<UMat> images_warped_f(arucos.size());
    for (int i = 0; i < arucos.size(); ++i)
        uimgs[i].convertTo(images_warped_f[i], CV_32F);

    seam_finder->find(images_warped_f, corners, masks);
    images_warped_f.clear();


    Ptr<Blender> blender;
    Rect dstRoi = resultRoi(corners, sizes);
    for(int i = 0; i < arucos.size(); i++){
        Mat img = imgs[i];
        Mat mask = arucosMask[i];
        Mat img_s;
        Mat dilated_mask, seam_mask;
        compensator->apply(i, corners[i], img, mask);
        img.convertTo(img_s, CV_16S);
        dilate(masks[i], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask.size(), 0, 0, INTER_LINEAR_EXACT);
        mask = seam_mask & mask;
        if (!blender)
        {
            blender = Blender::createDefault(Blender::MULTI_BAND, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        blender->feed(img_s, mask, corners[i]);
        arucos[i].T = addDisplacement(arucos[i].R, arucos[i].roi.x - origin_dstRoi.x, arucos[i].roi.y - origin_dstRoi.y);
        invert(toSquare(arucos[i].T), arucos[i].iT);

    }

    {
        Mat result, result_mask;
        blender->blend(result, result_mask);
        bitwise_not(result_mask,result_mask);
        result_mask.convertTo(result_mask,CV_8UC1);
        result.convertTo(result,CV_8UC3);
        //imshow("rw",result_mask);
        
        
        for(int i = 0; i < result.rows; i++){
            for(int j = 0; j < result.cols; j++){
                Vec3b colors = result.at<Vec3b>(i,j);
                colors[0] += result_mask.ptr<uchar>(i)[j];
                colors[1] += result_mask.ptr<uchar>(i)[j];
                colors[2] += result_mask.ptr<uchar>(i)[j];
                result.at<Vec3b>(i,j) = colors;
            }
        }
        processDst(result);
        resultImage = result.clone();
    }
    corners.clear();
    sizes.clear();
    masks.clear();
    uimgs.clear();
    for(int i = 0; i < arucos.size(); i++){
        arucos[i].image.release();
    }
    LOG("[INFO] Finish! ");
    double time2 = (static_cast<double>( getTickCount()) - time1)/getTickFrequency();
    LOGV("getResult耗时：", time2);
}

void Fusioner::init(vector<Mat>& srcs){
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    vector<Point> corners(2);
    vector<Point> sizes(2);
    arucos.clear();
    LOG("[INFO] Finding Arucos...");
    for (int i = 0; i < srcs.size(); i++){
        ImageAruco tmp;
        Mat inputImage = srcs[i];
        Mat imageCopy = inputImage.clone();
        tmp.image = srcs[i];
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(tmp.image, dictionary, corners, ids);
        tmp.ids = ids;
        tmp.corners = corners;
        if(i == 0) {
            tmp.mask.create(tmp.image.size(), CV_8UC1);
            tmp.mask.setTo(Scalar::all(255));
            tmp.size = tmp.image.size();
            tmp.roi = Rect(0,0,tmp.image.cols, tmp.image.rows);
            tmp.R = Mat(2,3,CV_32FC1, Scalar(0));
            tmp.R.at<float>(0,0) = 1;
            tmp.R.at<float>(1,1) = 1;
            tmp.scale = 1;
        }
        arucos.push_back(tmp);
    }
    srcs.clear();
    int count = arucos.size() - 1;
    vector<bool> arucos_success(arucos.size());
    arucos_success[0] = true;
    for(int i = 1; i < arucos.size(); i++){
        arucos_success[i] = false;
    }
    LOG("[INFO] Matching Arucos...");
    while(count){
        int icount = count;
        for(int i = 1; i < arucos.size() ; i++){
            if(!arucos_success[i]){
                for(int j = 0; j < arucos.size(); j++){
                    if(arucos_success[j] && match(arucos[j],arucos[i]).size() > 0){
                        double angle = compute(arucos[j], arucos[i]);
                        // arucos[i].mask.create(arucos[i].image.size(), CV_8UC1);
                        // arucos[i].mask.setTo(Scalar::all(255));
                        Mat tmp;
                        Mat R = warpRotation(arucos[i].image, tmp, angle);
                        arucos[i].R = R;
                        arucos[i].size = tmp.size();
                        tmp.release();
                        arucos[i].mask.create(arucos[i].first_size, CV_8UC1);
                        arucos[i].mask.setTo(Scalar::all(255));
                        warpRotation(arucos[i].mask, arucos[i].mask, angle);
                        
                        for(int idx = 0; idx < arucos[i].corners.size(); idx++){
                            for(int l = 0; l < 4; l++){
                                change(arucos[i].corners[idx][l], R);
                            }
                        }
                        arucos[i].roi = findRoi(arucos[j], arucos[i]);
                        arucos_success[i] = true;
                        count--;
                        break;
                    }
                }
            }
        }
        if(icount == count){
            LOG("[Error] There is an image alone.(No aruco matched with others')");
            exit(-1);
        }
    }
    getResult();
    

}

void Fusioner::changeImage(vector<Mat>& srcs){
    if(srcs.size() != arucos.size()){
        LOG("[Error] In Fusioner::changeImage, input images' num not fit arucos num");
        exit(-1);
    }
    for(int i = 0; i < srcs.size(); i++){
        arucos[i].image = srcs[i].clone();
        srcs[i].release();
    }
    LOG("[INFO] Fusioner changeImage success!");
    getResult();
    
}

void Fusioner::processDst(Mat& src){
    Mat dst = Mat(DST_HEIGHT, DST_WIDTH, CV_8UC3,Scalar(255,255,255));
    double size_param, angle = 0;
    int width = src.cols, height = src.rows;
    if (width < height){
        angle = 90;
        width = src.rows;
        height = src.cols;
    }
    if(1.0*width/height > 1.0*DST_WIDTH/DST_HEIGHT){
        size_param = 1.0*DST_WIDTH/width;
    }else{
        size_param = 1.0*DST_HEIGHT/height;
    }
    dstScale = size_param;
    resize(src,src,Size(), dstScale, dstScale);
    dstR = warpRotation(src,src,angle);
    Rect roi = Rect((DST_WIDTH-src.cols)/2,(DST_HEIGHT-src.rows)/2,src.cols,src.rows);
    dstR = addDisplacement(dstR, roi.x, roi.y);
    invert(toSquare(dstR),idstR);
    Mat mroi = dst(roi);
    src.copyTo(mroi);
    for(int i = roi.x; i < roi.x+roi.width; i++){
        for(int j = roi.y; j < roi.y+roi.height; j++){
            if (i == roi.x || i == roi.x+roi.width-1 || j == roi.y || j == roi.y+roi.height-1){
                dst.at<Vec3b>(j, i)[0] = 255;
                dst.at<Vec3b>(j, i)[1] = 255;
                dst.at<Vec3b>(j, i)[2] = 255;
            }
        }
    }
    src = dst;

}

Point2f Fusioner::getOrigin(int x, int y, bool isInverse){
    if(isInverse){
        double param_x = 1.0*SRC_WIDTH/S_WIDTH;
        double param_y = 1.0*SRC_HEIGHT/S_HEIGHT;
        return Point2f(x / param_x, y / param_x);
    }
    else{
        double param_x = 1.0*SRC_WIDTH/S_WIDTH;
        double param_y = 1.0*SRC_HEIGHT/S_HEIGHT;
        return Point2f(x * param_x, y * param_x);
    }

}

Point2f Fusioner::getDstPos(Point2f src, int idx, bool isInverse){
    if(isInverse){
        change(src, idstR);
        src = Point2f(1.0 * src.x / (dstScale * dstScaleQ), 1.0 * src.y / (dstScale * dstScaleQ));
        // src.x /= arucos[idx].scale;
        // src.y /= arucos[idx].scale;
        change(src, arucos[idx].iT);
        src = getOrigin(src.x, src.y, true);
        return src;
    }
    else{
        src = getOrigin(src.x, src.y);
        change(src, arucos[idx].T);
        // src.x *= arucos[idx].scale;
        // src.y *= arucos[idx].scale;
        src = Point2f(src.x * (dstScale * dstScaleQ), src.y * (dstScale * dstScaleQ));
        change(src, dstR);
        LOG(src);
        return src;
    }

}

void Fusioner::showResult(){
    imshow("result", resultImage);
    waitKey();
}

Mat Fusioner::getResultImage(){
    return resultImage;
}