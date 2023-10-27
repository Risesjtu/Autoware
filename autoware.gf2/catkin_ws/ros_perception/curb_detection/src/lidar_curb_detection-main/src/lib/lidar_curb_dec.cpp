/*
 * @Description:
 * @Author: speedzjy
 * @Date: 2022-04-27 11:55:01
 */

#include "lidar_curb_detection/lidar_curb_dec.hpp"
#include <vector>
namespace CurbDectection
{

double bezierPoint(std::vector<double>& P, double t,unsigned long number)
{
    std::vector<double> P1;
    for(int j=0; j<number; j++)
        P1.push_back((P[j]));
    auto n = number - 1;
    for(int r=0; r<n; r++)
    {
        for(int i=0; i<n-r; i++)
        {
            P1[i] = (1.0 - t) * P1[i] + t * P1[i+1];
        }
    }
    return P1[0];
}

void bezierLine(geometry_msgs::Point p[1000],PointCloudType::Ptr curb)
{
    std::cout<<"debug bezierLine"<<std::endl;

    std::vector<double> Px;
    std::vector<double> Py;
    if(curb->size()< 5) {return;}
    for(int j=0;j<curb->size();j++)
    {
        Px.push_back(curb->points[j].x);
        Py.push_back(curb->points[j].y);
    }
    
    double delta = 1.0 / 1000;
    double t = 0.0;
    for(int i=0; i<1000; i++)
    {
        double x = bezierPoint(Px, t,Px.size());
        double y = bezierPoint(Py, t,Py.size());
        t += delta;
        p[i].x=x;p[i].y=y;p[i].z=-2;
    }
}


void LidarCurbDectection::Syncallback1(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr, 
        const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr2,const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr3)
{
        std::cout<<"debug callback"<<std::endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        PointCloudType::Ptr tmp_points(new PointCloudType);
        pcl::fromROSMsg(*in_cloud_ptr, *tmp_points);
        queue_complete_points.push_back(*tmp_points);

        complete_points->clear();
         ground_points1 ->clear();
         no_ground_points1->clear();


        for (size_t i = 0; i < boundary_points.size(); ++i) 
        {
            boundary_points[i]->clear();
        }
//  //weiming debug
//         for (size_t i = 0; i < boundary_points.size(); ++i) 
//         {
//             if(boundary_points[i]->points.size()>10){
//             boundary_points[i]->clear();
//             }
//         }

        // 读取点云
        PointCloudType::Ptr completeCloud(new PointCloudType);
        if (!queue_complete_points.empty()) {
            *completeCloud = queue_complete_points.front();
            queue_complete_points.pop_front();
        }

        //计算点云laserID, 并将ID存为intensity
        CloudMapper mapper_laserID(cmMsg);
        PointCloudType::Ptr completeCloudMapper(new PointCloudType);
        mapper_laserID.processByOri(completeCloud, completeCloudMapper);
        AINFO << "raw points number: " << completeCloudMapper->points.size() << endl;

        // 地面提取
        // 论文中选取范围: Z x Y x X , [−3, 1] x [−40, 40] x [−70, 70]
        // 代码实际选取: Y x X , [−30, 30] x [−40, 40]
        // 使用 pcl 库进行平面特征点提取
        PointCloudType::Ptr ground_points(new PointCloudType);
        PointCloudType::Ptr ground_points_no(new PointCloudType); //非地面点

        PointCloudType::Ptr tmp_points2(new PointCloudType);
        pcl::fromROSMsg(*in_cloud_ptr2, *tmp_points2);
        queue_complete_points2.push_back(*tmp_points2);

        // 读取点云
        std::cout<<"debug groundCloud"<<std::endl;
        std::cout<<"debug groundCloud"<<queue_complete_points2.size()<<std::endl;

        PointCloudType::Ptr groundCloud(new PointCloudType);
        if (!queue_complete_points2.empty())
         {
            std::cout<<"debug groundCloud_111"<<std::endl;
            *groundCloud = queue_complete_points2.front();
            queue_complete_points2.pop_front();
        }
        *ground_points1 = *groundCloud;
       // *ground_points = *ground_points1;
       AINFO << "ground_points number: " << ground_points->points.size() << endl;


        std::cout<<"debug no_groundCloud"<<std::endl;

        PointCloudType::Ptr tmp_points3(new PointCloudType);
        pcl::fromROSMsg(*in_cloud_ptr3, *tmp_points3);
        queue_complete_points3.push_back(*tmp_points3);
        // 读取点云

        PointCloudType::Ptr no_groundCloud(new PointCloudType);
        if (!queue_complete_points3.empty()) {
            *no_groundCloud = queue_complete_points3.front();
            queue_complete_points3.pop_front();
        }
        *no_ground_points1 = *no_groundCloud;


        
        //*ground_points_no = *no_ground_points1;
        // GroundSegmentation ground(completeCloudMapper, gsMsg);
        // ground.groundfilter(ground_points, ground_points_no);
 
        AINFO << "no_ground_points number: " << ground_points_no->points.size()
            << endl;
            
        //weiming
        CloudMapper mapper_laserID_1(cmMsg);
        mapper_laserID_1.processByOri(ground_points1, ground_points);
        CloudMapper mapper_laserID_2(cmMsg);
        mapper_laserID_2.processByOri(no_ground_points1, ground_points_no);

        //根据之前计算的Intensity对地面点云进行mapper
        CloudMapper mapper2(cmMsg);
        scanIndices scanIDindices;
        PointCloudType::Ptr ground_points_mapper(new PointCloudType);
        mapper2.processByIntensity(ground_points, ground_points_mapper,
            scanIDindices);

        //特征点提取
        pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(
            new pcl::PointCloud<pcl::PointXYZI>);
        FeaturePoints curbExtract(ground_points_mapper, scanIDindices, fpMsg);
        curbExtract.extractFeatures(featurePoints);
        AINFO << "feature points number: " << featurePoints->points.size() << endl;

        //高斯过程提取
        BoundaryPoints refinePoints(*featurePoints, cmMsg, bpMsg);
        // 道路中心线
        MarkerList line_list(2);
        refinePoints.process(ground_points_no, boundary_points, line_list);

 // 道路边缘线
        MarkerList curb_line_list(2);
        for (size_t i = 0; i < curb_line_list.size(); ++i)
        {
            curb_line_list[i].type = visualization_msgs::Marker::LINE_LIST;
            curb_line_list[i].action = visualization_msgs::Marker::ADD;
            curb_line_list[i].id = i;
            curb_line_list[i].ns ="Bline_curb";
            curb_line_list[i].scale.x = 0.1;
            // Line list is green
            if (i == 0)
                 {curb_line_list[i].color.g = 1.0;}
            else
                 {curb_line_list[i].color.b = 1.0;}

            curb_line_list[i].color.a = 1.0;
            // curb_line_list[i].pose.orientation =
            // tf::createQuaternionMsgFromRollPitchYaw(
            // 0, 0, _segmentAngle[i] / 180.0f * M_PI);

            // geometry_msgs::Point p;
            // p.x = p.y = p.z = 0;
            // // The line list needs two points for each line
            // curb_line_list[i].points.push_back(p);
            // p.x += 5.0;
            // curb_line_list[i].points.push_back(p);
        }

        CloudPtrList curb_points(2);

        std::cout << "\n----------------\n";
        std::cout << "\ncurb_points.size(): " << curb_points.size();
        std::cout << "\nboundary_points[0]->points.size(): " << boundary_points[0]->points.size();
        std::cout << "\nboundary_points[1]->points.size(): " << boundary_points[1]->points.size();

        *curb_left += *boundary_points[0];
        *curb_right += *boundary_points[1];

        // std::cout << "\nboundary_points.size(): " << boundary_points.size();
        std::cout << "\ncurb_left->points.size(): " << curb_left->points.size();
        std::cout << "\ncurb_right->points.size(): " << curb_right->points.size();

        std::cout << "\n----------------\n";
        // std::cout << "\ncurb_points[1]->points.size(): " << curb_points[1]->points.size();
       // *curb_right += *boundary_points[1];
       // curb_points = boundary_points;

        // for (size_t i = 0; i < boundary_points.size(); ++i) 
        // {
        //     auto boundary_point = boundary_points[i];
        //     auto& curb_point = curb_points[i];
        //     for(const auto& point: boundary_point->points)
        //     {
        //         curb_point->points.push_back(point);
        //     }

        //     //     curb = 
        //     // for(size_t j = 0; j < boundary_points[i]->points.size(); ++j) 
        //     //     {
                
        //     //       curb_points[i]->points.push_back(boundary_points[i]->points[j]);
        //     //     }
        // }


        
       // curb_points = boundary_points;




        if(boundary_points[0]->points.size()){
            geometry_msgs::Point p_list1[1000];
            bezierLine(p_list1,boundary_points[0]);
            for(int ii=0;ii<999;ii++) {
                curb_line_list[0].points.push_back(p_list1[ii]);
                curb_line_list[0].points.push_back(p_list1[ii+1]);
            }
        }
        if(boundary_points[1]->points.size()){
            geometry_msgs::Point p_list2[1000];
            bezierLine(p_list2,boundary_points[1]);
            for(int ii=0;ii<999;ii++) {
                curb_line_list[1].points.push_back(p_list2[ii]);
                curb_line_list[1].points.push_back(p_list2[ii+1]);
            }
        }

            if (curb_left->points.size()>6) 
        {
            curb_left->points.clear();
        }
        if (curb_right->points.size()>6) 
        {
            curb_right->points.clear();
        }


        //pub
        *complete_points = *completeCloud;
        // 计时
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        AINFO << "compute this frame time cost = " << time_used.count()
            << " seconds. " << endl;

        sensor_msgs::PointCloud2 tmp_rosCloud;
        std::string in_cloud_frame_id = in_cloud_ptr->header.frame_id;

        //  完整点云
        pcl::toROSMsg(*complete_points, tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubCompleteCloud_.publish(tmp_rosCloud);

        // 地面点
        pcl::toROSMsg(*ground_points, tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubGroundCloud_.publish(tmp_rosCloud);

        // 非地面点
        pcl::toROSMsg(*ground_points_no, tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubNoGroundCloud_.publish(tmp_rosCloud);

        // 特征点
        pcl::toROSMsg(*featurePoints, tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubFeatureCloud_.publish(tmp_rosCloud);

        // 左边缘点
        pcl::toROSMsg(*(boundary_points[0]), tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubCurbCloudLeft_.publish(tmp_rosCloud);

        // 右边缘点
        pcl::toROSMsg(*(boundary_points[1]), tmp_rosCloud);
        tmp_rosCloud.header.frame_id = in_cloud_frame_id;
        pubCurbCloudRight_.publish(tmp_rosCloud);

        // 道路中心线
        for (size_t i = 0; i < line_list.size(); ++i) {
            line_list[i].header.frame_id = in_cloud_frame_id;
            pubMarker_.publish(line_list[i]);
        }
          // 道路边缘线
        for (size_t i = 0; i < curb_line_list.size(); ++i){
            curb_line_list[i].header.frame_id = in_cloud_frame_id;
            pubMarker_Bline_.publish(curb_line_list[i]);
        }
}

void debug_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr)
{
    cout << "debug points_raw" << endl;
}


LidarCurbDectection::LidarCurbDectection()
    : complete_points(new PointCloudType), boundary_points(CloudPtrList(2)),
    ground_points1(new PointCloudType),
    no_ground_points1(new PointCloudType),
     curb_left(new PointCloudType) ,
   curb_right(new PointCloudType)  
{
    cout << "\033[1;31m hw1! \033[0m" << endl;

    // 建立需要订阅的消息对应的订阅器
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_1(nh_, "/points_raw", 5000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subGround_PointCloud_1(nh_, "/points_ground", 5000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subNO_ground_PointCloud_1(nh_, "/points_no_ground", 5000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5000), subPointCloud_1, subGround_PointCloud_1, subNO_ground_PointCloud_1); //queue size=10
    sync.registerCallback(boost::bind(&LidarCurbDectection::Syncallback1, this, _1, _2, _3) );

    ros::NodeHandle n;
    ros::Subscriber debug_sub = n.subscribe("/points_raw", 1000, &debug_callback);

    pubCompleteCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/complete_cloud", 10);
    pubGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 10);
     pubNoGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/no_ground_cloud", 10);
    pubFeatureCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/feature_cloud", 10);
    pubCurbCloudLeft_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_left", 10);
    pubCurbCloudRight_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_right", 10);
    pubMarker_ =
      nh_.advertise<visualization_msgs::Marker>("/visual_marker", 10);
    pubMarker_Bline_=
      nh_.advertise<visualization_msgs::Marker>("/visual_marker_Bline", 10);
  
    for (size_t i = 0; i < boundary_points.size(); ++i) {
         boundary_points[i] = boost::make_shared<PointCloudType>();
    }
     ros::spin();
    cout << "\033[1;31m apple! \033[0m" << endl;
}

LidarCurbDectection::~LidarCurbDectection() {}



}