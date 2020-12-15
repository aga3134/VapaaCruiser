#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CompressedImage.h"
#include "vapaa_cruiser/objectDetect.h"
#include "vapaa_cruiser/objectDetectArray.h"
#include "darknet.h"
#include "option_list.h"
#include "image.h"
#include "data.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>

network* net = NULL;
char** names = NULL;
int classNum = 0;
ros::Publisher yoloPub, detectPub;
image **alphabet = NULL;

//opencv channel: BGR, yolo image channel: RGB
IplImage *image_to_ipl(image im){
    int x,y,c;
    IplImage *disp = cvCreateImage(cvSize(im.w,im.h), IPL_DEPTH_8U, im.c);
    int step = disp->widthStep;
    for(y = 0; y < im.h; ++y){
        for(x = 0; x < im.w; ++x){
            for(c= 0; c < im.c; ++c){
                float val = im.data[c*im.h*im.w + y*im.w + x];
                disp->imageData[y*step + x*im.c + im.c-1-c] = (unsigned char)(val*255);
            }
        }
    }
    return disp;
}

image ipl_to_image(IplImage* src){
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c); 
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){ 
                im.data[k*w*h + i*w + j] = data[i*step + j*c + c-1-k]/255.;
            }
        }
    }
    return im;
}

//void imageCallback(const sensor_msgs::ImageConstPtr& msg){  //for raw image
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg){  //for compressed image
    //ROS_INFO("got image");
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        //do yolo detection
        IplImage src = (cv_ptr->image);
        image yoloImage = ipl_to_image(&src);
        image sized = letterbox_image(yoloImage, net->w, net->h);
        network_predict(*net, sized.data);
        int nboxes = 0;
        float nms = 0.5;
        
        detection *dets = get_network_boxes(net, yoloImage.w, yoloImage.h, 0.5, 0.5, 0, 1, &nboxes,1);
        if (nms) do_nms_sort(dets, nboxes, classNum, nms);
        float thresh = 0.5;
        draw_detections_v3(yoloImage, dets, nboxes, thresh, names, alphabet, classNum, 0);
        
        //show result in opencv
        IplImage* dst = image_to_ipl(yoloImage);
        //cvShowImage("image",dst);
        //cv::waitKey(1);
        cv_bridge::CvImage cvi;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = cv::cvarrToMat(dst);
        //sensor_msgs::ImagePtr msg = cvi.toImageMsg(); //for raw image
        sensor_msgs::CompressedImagePtr msg = cvi.toCompressedImageMsg();   //for compressed image
        yoloPub.publish(msg);
  
        //publish detected object
        vapaa_cruiser::objectDetectArray odArr;
        odArr.header.stamp = ros::Time::now();
        for(int i=0;i<nboxes;i++){
            float bestP = thresh;
            int bestClass = -1;
            detection det = dets[i];

            for(int j=0;j<det.classes;j++){
                if(det.prob[j] > bestP){
                    bestP = det.prob[j];
                    bestClass = j;
                }
            }

            if(bestClass >= 0){
                vapaa_cruiser::objectDetect od;
                od.id = bestClass;
                od.name = names[bestClass];
                int left = (det.bbox.x - det.bbox.w / 2.)*dst->width;
                int right = (det.bbox.x + det.bbox.w / 2.)*dst->width;
                int top = (det.bbox.y - det.bbox.h / 2.)*dst->height;
                int bottom= (det.bbox.y + det.bbox.h / 2.)*dst->height;

                od.corner[0].x = left;
                od.corner[0].y = top;
                od.corner[1].x = right;
                od.corner[1].y = top;
                od.corner[2].x = right;
                od.corner[2].y = bottom;
                od.corner[3].x = left;
                od.corner[3].y = bottom;

                odArr.object_array.push_back(od);
            }
            
        }
        detectPub.publish(odArr);

        //release memory
        free_detections(dets, nboxes);
        cvReleaseImage(&dst);
        free_image(yoloImage);
        free_image(sized);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv){
    char cwd[256];
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);
    
    ros::init(argc, argv, "yolov4_node");
    ROS_INFO("start yolov4 node");
    ros::NodeHandle nh;

    std::string cfgfile, weightfile,namefile,optionfile;
    std::string nodePath = ros::package::getPath("yolov4");
    nh.param<std::string>("cfgfile", cfgfile, nodePath+"/config/yolov4-tiny.cfg");
    nh.param<std::string>("weightfile", weightfile, nodePath+"/config/yolov4-tiny.weights");
    nh.param<std::string>("namefile", namefile, nodePath+"/config/obj.names");
    nh.param<std::string>("optionfile", optionfile, nodePath+"/config/obj.data");
    alphabet = load_alphabet();
    int names_size = 0;
    names = get_labels_custom(const_cast<char*>(namefile.c_str()), &names_size);
    net = load_network_custom(const_cast<char*>(cfgfile.c_str()), const_cast<char*>(weightfile.c_str()), 0, 1);
    calculate_binary_weights(*net);
    list *options = read_data_cfg(const_cast<char*>(optionfile.c_str()));
    classNum = option_find_int(options, const_cast<char*>("classes"), 0);
    ROS_INFO("class num: %d",classNum);
    srand(2222222);

    //cvNamedWindow("image");
    yoloPub = nh.advertise<sensor_msgs::CompressedImage>("yolov4/detected/compressed", 1);
    detectPub = nh.advertise<vapaa_cruiser::objectDetectArray>("yolov4/object", 1);
    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw/compressed",1,imageCallback);  //for compressed image
    ros::spin();

    const int nsize = 8;
    for(int j = 0; j < nsize; ++j) {
        for(int i = 32; i < 127; ++i) {
            free_image(alphabet[j][i]);
        }
        free(alphabet[j]);
    }
    free(alphabet);
    free_ptrs((void**)names, net->layers[net->n - 1].classes);
    free_list_contents_kvp(options);
    free_list(options);
    free_network(*net);
    return 0;
}
