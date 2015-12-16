#include "ros/ros.h"
#include "ros/package.h"

#include "cocoman_vismotcoord/vismotcoord.h"
#include <tf/transform_broadcaster.h>
#include "cocoman_vismotcoord/command.h"

CloudPtr pc_in, pc_transformed_init, pc_transformed_updated, pc_transformed_updated2, pc_eval_scene, pc_eval_transformed, pc_crspd, pc_down;
double lastT, nowT;
double startT;

VisMotCoord* vismot;
tf::TransformListener* tf_listener;
ros::Publisher pub_bodypart;
ros::Publisher pub_pc;
ros::Publisher pub_crspd;
ros::Publisher pub_crsptr;
ros::Publisher pub_eval;
string baseFrame = "/base";
string camFrame_init = "/xtion_frame";
string camFrame_updated = "/xtion_frame_updated";
string focusFrame = "/right_lower_forearm";
tf::TransformBroadcaster* tf_br;

int mode = 0;   // 0:stop, 1: start, 2:left, 3:right

double sample_dist = 0.01;
float boundMargin = 0.05;
void transform(string frame);
void downsampling(double scale);

void cb_rgbd(const pcl::PCLPointCloud2ConstPtr& input)
{
    if(DEBUG_MAIN) FPS_CALC("cb_rgbd");
    static int iter = 0;

    if(mode == 2){
         focusFrame = "/left_lower_forearm";
         vismot->filterFrameNames.clear();
         vismot->filterFrameNames.push_back("/left_lower_forearm");
         vismot->filterFrameNames.push_back("/left_upper_forearm");
    }
    else if(mode == 3){
        focusFrame = "/right_lower_forearm";
        vismot->filterFrameNames.clear();
         vismot->filterFrameNames.push_back("/right_lower_forearm");
         vismot->filterFrameNames.push_back("/right_upper_forearm");
    }

    // input
    pc_in.reset(new Cloud);
    pcl::fromPCLPointCloud2(*input, *pc_in);
    if(DEBUG_MAIN) cout<<endl<<"I heard RGB, # of points: "<<pc_in->points.size()<<endl;
    ros::Time currentT;
    if(iter == 0){
        startT = pcl::getTime();
    }
    //     pc preproc (transformation and downsampling)
    lastT = pcl::getTime();
    downsampling(sample_dist);
    if(DEBUG_MAIN) cout<<"downsampling time: "<<pcl::getTime()-lastT<<", # of points: "<<pc_down->points.size()<<endl;

    lastT = pcl::getTime();
    vismot->bodyTransformSet(focusFrame);
    if(DEBUG_MAIN) cout<<"bodyTransformSet time : "<<pcl::getTime()-lastT<<endl;

    lastT = pcl::getTime();
    vismot->setBoundBox(boundMargin);
    if(DEBUG_MAIN) cout<<"boundbox time : "<<pcl::getTime()-lastT<<endl;

    CloudPtr pc_updated;

    lastT = pcl::getTime();

    if(vismot->isCamUpdate == 0 && mode == 0){
        if(DEBUG_MAIN) cout<<"set scene"<<endl;
        vismot->setScene(pc_down, currentT);
    }
    else if(vismot->isCamUpdate == 0 && mode != 0){
        if(DEBUG_MAIN) cout<<"set scene2"<<endl;
        vismot->setScene2(pc_down, currentT);
    }
    else if(vismot->isCamUpdate == 1){
        if(DEBUG_MAIN) cout<<"update scene"<<endl;
        vismot->updateScene(pc_down, pc_updated);
    }
    if(DEBUG_MAIN) cout<<"set scene end, # of scene points: "<<vismot->numScenePC()<<endl;
    if(DEBUG_MAIN) cout<<"set scene time : "<<pcl::getTime()-lastT<<endl;

    Model *model;
    if(PUB_VISMODEL || PUB_EVAL){
        vismot->bodyTransformOrg();
        model = vismot->getModel();
    }
    if(PUB_EVAL && iter == 0){
        lastT = pcl::getTime();
        pcl::PCLPointCloud2 output_pc_eval;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_eval;
        pc_eval.reset(new pcl::PointCloud<pcl::PointXYZI>);

        // model
        MatrixXf boundBox(3,2);
        int cnt = 0;
        for(int n=0;n<model->numPart();n++){
            for(int l=0;l<vismot->filterFrameNames.size();l++){
                if(model->partFrame(n) == vismot->filterFrameNames.at(l)){
                    BodyPart *part = model->bodyPart(n);
                    Mesh* mesh = part->meshOrg();
                    for(int i=0;i<mesh->numMesh();i++){
                        for(int j=0;j<mesh->numVertex(i);j++){
                            pcl::PointXYZI pt;
                            pt.x = mesh->vertex(i,j)->m_pos[0];
                            pt.y = mesh->vertex(i,j)->m_pos[1];
                            pt.z = mesh->vertex(i,j)->m_pos[2];
                            pt.intensity = 0.;
                            pc_eval->points.push_back(pt);

                            if(cnt == 0){
                                boundBox(0,0) = boundBox(0,1) = pt.x;
                                boundBox(1,0) = boundBox(1,1) = pt.y;
                                boundBox(2,0) = boundBox(2,1) = pt.z;
                            }
                            else{
                                if(pt.x > boundBox(0,0))    boundBox(0,0) = pt.x;
                                if(pt.x < boundBox(0,1))    boundBox(0,1) = pt.x;
                                if(pt.y > boundBox(1,0))    boundBox(1,0) = pt.y;
                                if(pt.y < boundBox(1,1))    boundBox(1,1) = pt.y;
                                if(pt.z > boundBox(2,0))    boundBox(2,0) = pt.z;
                                if(pt.z < boundBox(2,1))    boundBox(2,1) = pt.z;
                            }
                            cnt++;
                        }
                    }
                }
            }
        }

        // scene
        boundBox << boundBox(0,0) + boundMargin, boundBox(0,1)-boundMargin,
                boundBox(1,0) + boundMargin, boundBox(1,1)-boundMargin,
                boundBox(2,0) + boundMargin, boundBox(2,1)-boundMargin;
        pc_eval_scene.reset(new Cloud);
        tf::StampedTransform camPos_eval = vismot->getPosUpdated_cam();
        pcl_ros::transformPointCloud(*pc_in, *pc_eval_scene, camPos_eval);

        for(int i=0;i<pc_eval_scene->points.size();i++){
            PointT pt_scene = pc_eval_scene->points.at(i);
            pcl::PointXYZI pt;
            if(pt_scene.x<boundBox(0,0) && pt_scene.x>boundBox(0,1) && pt_scene.y<boundBox(1,0) && pt_scene.y>boundBox(1,1) && pt_scene.z<boundBox(2,0) && pt_scene.z>boundBox(2,1)){
                pt.x = pt_scene.x;
                pt.y = pt_scene.y;
                pt.z = pt_scene.z;
                pt.intensity = 1.;
                pc_eval->points.push_back(pt);
            }
        }
        pcl::toPCLPointCloud2(*pc_eval, output_pc_eval);
        output_pc_eval.header.frame_id = baseFrame;
        pub_eval.publish (output_pc_eval);
    }

    lastT = pcl::getTime();
    if(mode != 0)
        vismot->coordinate();
    if(DEBUG_MAIN) cout<<"coordinate time : "<<pcl::getTime()-lastT<<endl;

    // output calibrated cam tf
    //    if(vismot->isCamUpdated()) {
    tf::Transform cam_calibrated = vismot->getPosUpdated_cam();
    tf_br->sendTransform(tf::StampedTransform(cam_calibrated, ros::Time::now(), "base", "cam_calibrated"));
    if(PUB_TIME){
        cout<<pcl::getTime()-startT<<","<<endl;
    }
    Eigen::Matrix4f eigen_cam_calibrated;
    transformAsMatrix (cam_calibrated, eigen_cam_calibrated);
//            cout<<eigen_cam_calibrated<<endl;

    lastT = pcl::getTime();

    if(PUB_VISMODEL){        
        // body mesh
        visualization_msgs::Marker bodypart;
        bodypart.header.frame_id = baseFrame;
        //        bodypart.header.stamp = ros::Time()::now();
        bodypart.header.stamp = ros::Time::now();
        bodypart.ns = "cocoman";
        bodypart.action = visualization_msgs::Marker::ADD;
        bodypart.pose.orientation.w = 1.0;

        bodypart.id = 1;
        bodypart.type = visualization_msgs::Marker::LINE_LIST;

        //        bodypart.lifetime = ros::Duration(0);
        bodypart.scale.x = 0.0002;
        //        bodypart.scale.y = 0.1;
        //    bodypart.color.r = .0f;
        bodypart.color.g = 1.0f;
        //    bodypart.color.b = .0f;
        bodypart.color.a = 0.5;

        for(int n=0;n<model->numPart();n++){
            for(int l=0;l<vismot->filterFrameNames.size();l++){
                if(model->partFrame(n) == vismot->filterFrameNames.at(l)){
                    BodyPart *part = model->bodyPart(n);
                    // visualization of model from base
                    Mesh* mesh = part->meshOrg();
                    //                    // visualization of model from focus
                    //                    Mesh* mesh = part->meshFocused();
                    for(int i=0;i<mesh->numMesh();i++){
                        for(int j=0;j<mesh->numFilteredFaces(i);j++){
                            int faceId = mesh->filteredFaceId(i,j);
                            geometry_msgs::Point p[3];
                            for(int k=0;k<3;k++){
                                p[k].x = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[0];
                                p[k].y = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[1];
                                p[k].z = mesh->vertex(i,mesh->face(i,faceId)->mIndices[k])->m_pos[2];
                            }
                            bodypart.points.push_back(p[0]);
                            bodypart.points.push_back(p[1]);
                            bodypart.points.push_back(p[1]);
                            bodypart.points.push_back(p[2]);
                            bodypart.points.push_back(p[2]);
                            bodypart.points.push_back(p[0]);
                        }
                    }
                }
            }
        }
        pub_bodypart.publish(bodypart);
    }
    if(DEBUG_MAIN) cout<<"visualization time : "<<pcl::getTime()-lastT<<endl;

    if(PUB_EVAL){
        lastT = pcl::getTime();
        pcl::PCLPointCloud2 output_pc_eval;
        //        if(vismot->isCamUpdated()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_eval;
        pc_eval.reset(new pcl::PointCloud<pcl::PointXYZI>);

        // model
        MatrixXf boundBox(3,2);
        int cnt = 0;
        for(int n=0;n<model->numPart();n++){
            for(int l=0;l<vismot->filterFrameNames.size();l++){
                if(model->partFrame(n) == vismot->filterFrameNames.at(l)){
                    BodyPart *part = model->bodyPart(n);
                    Mesh* mesh = part->meshOrg();
                    for(int i=0;i<mesh->numMesh();i++){
                        for(int j=0;j<mesh->numVertex(i);j++){
                            pcl::PointXYZI pt;
                            pt.x = mesh->vertex(i,j)->m_pos[0];
                            pt.y = mesh->vertex(i,j)->m_pos[1];
                            pt.z = mesh->vertex(i,j)->m_pos[2];
                            pt.intensity = 0.;
                            pc_eval->points.push_back(pt);

                            if(cnt == 0){
                                boundBox(0,0) = boundBox(0,1) = pt.x;
                                boundBox(1,0) = boundBox(1,1) = pt.y;
                                boundBox(2,0) = boundBox(2,1) = pt.z;
                            }
                            else{
                                if(pt.x > boundBox(0,0))    boundBox(0,0) = pt.x;
                                if(pt.x < boundBox(0,1))    boundBox(0,1) = pt.x;
                                if(pt.y > boundBox(1,0))    boundBox(1,0) = pt.y;
                                if(pt.y < boundBox(1,1))    boundBox(1,1) = pt.y;
                                if(pt.z > boundBox(2,0))    boundBox(2,0) = pt.z;
                                if(pt.z < boundBox(2,1))    boundBox(2,1) = pt.z;
                            }
                            cnt++;
                        }
                    }
                }
            }
        }

        // scene
        boundBox << boundBox(0,0) + boundMargin, boundBox(0,1)-boundMargin,
                boundBox(1,0) + boundMargin, boundBox(1,1)-boundMargin,
                boundBox(2,0) + boundMargin, boundBox(2,1)-boundMargin;
        pc_eval_scene.reset(new Cloud);
        tf::StampedTransform camPos_eval = vismot->getPosUpdated_cam();
        pcl_ros::transformPointCloud(*pc_in, *pc_eval_scene, camPos_eval);

        for(int i=0;i<pc_eval_scene->points.size();i++){
            PointT pt_scene = pc_eval_scene->points.at(i);
            pcl::PointXYZI pt;
            if(pt_scene.x<boundBox(0,0) && pt_scene.x>boundBox(0,1) && pt_scene.y<boundBox(1,0) && pt_scene.y>boundBox(1,1) && pt_scene.z<boundBox(2,0) && pt_scene.z>boundBox(2,1)){
                pt.x = pt_scene.x;
                pt.y = pt_scene.y;
                pt.z = pt_scene.z;
                pt.intensity = 1.;
                pc_eval->points.push_back(pt);
            }
        }
        pcl::toPCLPointCloud2(*pc_eval, output_pc_eval);
        output_pc_eval.header.frame_id = baseFrame;
        pub_eval.publish (output_pc_eval);
        //        }

        if(DEBUG_MAIN) cout<<"pub_eval time : "<<pcl::getTime()-lastT<<endl;
    }

    iter ++;
}

bool command(cocoman_vismotcoord::command::Request &req, cocoman_vismotcoord::command::Request &res)
{
    if (req.command == "start"){
        mode = 1;
        cout<<"start"<<endl;
        vismot->isCamUpdate = 0;
    }
    else if(req.command == "left"){
        mode = 2;
        cout<<"left"<<endl;
        vismot->isCamUpdate = 0;
    }
    else if(req.command == "right"){
        mode = 3;
        cout<<"right"<<endl;
        vismot->isCamUpdate = 0;
    }
    else if(req.command == "stop"){
        mode = 0;
        cout<<"stop"<<endl;
        vismot->isCamUpdate = 0;
    }

    return true;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cocoman_vismotcoord");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    tf_br = new tf::TransformBroadcaster();
    ros::Subscriber sub_rgbd = n.subscribe("/xtion/depth_registered/points", 1, cb_rgbd);
    ros::ServiceServer service = n.advertiseService("cocoman_command", command);
//    ros::Subscriber
    pub_bodypart = n.advertise<visualization_msgs::Marker>("cocoman/bodypart", 1);
    pub_pc = n.advertise<pcl::PCLPointCloud2>("cocoman/pc", 1);
    //    pub_crspd = n.advertise<visualization_msgs::Marker>("cocoman/crspd", 1);
    pub_crsptr = n.advertise<pcl::PCLPointCloud2>("cocoman/crsptr", 1);
    pub_eval = n.advertise<pcl::PCLPointCloud2>("cocoman/eval", 1);

    // input: pointcloud, joint state of baxter, cad models
    // output posdiff of the cam

    vector<string> vecPartFrames;
    vecPartFrames.push_back("/left_lower_forearm");
    vecPartFrames.push_back("/left_upper_forearm");
    //    vecPartFrames.push_back("/left_wrist");
    vecPartFrames.push_back("/right_lower_forearm");
    vecPartFrames.push_back("/right_upper_forearm");
    //    vecPartFrames.push_back("/right_wrist");

    vector<string> vecPartPath;
    string path_baxter = ros::package::getPath("baxter_description");
    string path_lower_forearm = path_baxter + "/meshes/lower_forearm/W1.DAE";
    string path_upper_forearm = path_baxter + "/meshes/upper_forearm/W0.DAE";
    //    string path_wrist = path_baxter + "/meshes/wrist/W2.DAE";
    vecPartPath.push_back(path_lower_forearm);
    vecPartPath.push_back(path_upper_forearm);
    //    vecPartPath.push_back(path_wrist);
    vecPartPath.push_back(path_lower_forearm);
    vecPartPath.push_back(path_upper_forearm);
    //    vecPartPath.push_back(path_wrist);

    vismot = new VisMotCoord(tf_listener, baseFrame, camFrame_init, camFrame_updated);
    vismot->setBodyModel(vecPartFrames, vecPartPath);
    ros::spin();

    return 0;
}

void downsampling(double scale)
{
    pc_down.reset(new Cloud);
    CloudConstPtr pcConst = (CloudConstPtr)pc_in;

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(pcConst);
    vg.setLeafSize(scale, scale, scale); // down sampling using a leaf size of 'scale'
    vg.filter(*pc_down);
}


//void transform(string frame)
//{
//    pc_transformed.reset(new Cloud);
//    tf_listener->waitForTransform(frame, pc_in->header.frame_id, ros::Time::now(), ros::Duration(5.0));
//    pcl_ros::transformPointCloud(frame, *pc_in, *pc_transformed, *tf_listener);

////    pc_in.reset(new Cloud);
//}
