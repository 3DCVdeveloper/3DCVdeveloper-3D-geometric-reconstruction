#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

using namespace std;

struct CAM_PARAM
{
    const double fx_ = 595.4;
    const double fy_ = 593.6;
    const double cx_ = 311.7;
    const double cy_ = 246.7;
    const double factor_ = 1000.f;
};

void run();
void readKFTrajectory(const string &keyframeFile, const string &sequencePath, vector<cv::Mat> &vImRGB, vector<cv::Mat> &vImD, vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &vTwc);
void unProject(const cv::Mat &color_, const cv::Mat &depth_, const CAM_PARAM &cam_, PointCloud::Ptr cloud_);

cpu_tsdf::TSDFVolumeOctree::Ptr tsdf(new cpu_tsdf::TSDFVolumeOctree);

int main()
{
    vector<cv::Mat> vImRGB, vImD;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vTwc;
    readKFTrajectory("orbbec_data/KeyFrameTrajectory.txt", "orbbec_data/20201221stairs", vImRGB, vImD, vTwc);
    if (vImRGB.size() != vImD.size() || vImRGB.size() != vTwc.size())
    {
        PCL_ERROR("Size of RGB, depth, pose do not match, %d, %d, %d, respectively.\n", vImRGB.size(), vImD.size(), vTwc.size());
        return -1;
    }
    cout << "Trajectory size: " << vImRGB.size() << endl;

    CAM_PARAM camIntrinsic;
    PointCloud::Ptr globalMap(new PointCloud);
    globalMap->reserve(99999999);
    pcl::VoxelGrid<PointT> voxel_grid;

    tsdf->setGridSize(20., 20., 20.);      // 20m x 20m x 20m
    tsdf->setResolution(2048, 2048, 2048); // Smallest cell size = 20m / 2048 = about a centimeter
    tsdf->setImageSize(vImRGB[0].cols, vImRGB[0].rows);
    tsdf->setCameraIntrinsics(camIntrinsic.fx_, camIntrinsic.fy_, camIntrinsic.cx_, camIntrinsic.cy_);
    tsdf->setNumRandomSplts(1);
    tsdf->setSensorDistanceBounds(0, 3);
    tsdf->setIntegrateColor(true); // Set to true if you want the TSDF to store color
    tsdf->setDepthTruncationLimits(0.03, 0.03);
    tsdf->reset(); // Initialize it to be empty

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PC Visualizer"));
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    pcl::console::TicToc time;
    time.tic();
    for (size_t i = 0; i < vImRGB.size(); i++)
    {
        PointCloud::Ptr cloud(new PointCloud);
        unProject(vImRGB[i], vImD[i], camIntrinsic, cloud);

// no fusion
        // pcl::transformPointCloud(*cloud, *cloud, vTwc[i].matrix());
        // voxel_grid.setInputCloud(cloud);
        // voxel_grid.setLeafSize(0.02, 0.02, 0.02);
        // voxel_grid.filter(*cloud);
        // globalMap->insert(globalMap->end(), cloud->begin(), cloud->end());
        // cout << "idx " << i << " global map size " << globalMap->size() << endl;

// tsdf
        if (cloud->height != vImRGB[0].rows || cloud->width != vImRGB[0].cols)
        {
            PCL_ERROR("Error: cloud %d has size %d x %d, but TSDF is initialized for %d x %d pointclouds\n", i + 1, cloud->width, cloud->height, vImRGB[0].cols, vImRGB[0].rows);
            return -1;
        }
        Eigen::Affine3d Twc(vTwc[i].affine());
        tsdf->integrateCloud(*cloud, pcl::PointCloud<pcl::Normal>(), Twc);
        cout << "idx " << i << endl;
    }
    cout << "Total time consumption: " << time.toc() / 1000 << "s" << endl;
    cout << "Average time consumption: " << (time.toc() / 1000) / vImRGB.size() << "s" << endl;

    // cout << "Integration done. Saving TSDF..." << endl;
    // tsdf->save("globalTSDF.vol");
    // cout << "Global TSDF saved." << endl;
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setMinWeight(0);
    mc.setInputTSDF(tsdf);
    mc.setColorByRGB(true);
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    cout << "Reconstructing..." << endl;
    mc.reconstruct(*mesh);
    cout << "Reconstruction done." << endl;
    // cout << "Saving mesh..." << endl;
    // pcl::io::savePLYFile("globalMesh.ply", *mesh);
    // cout << "Global mesh saved." << endl;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PC Visualizer"));
    // viewer->addCoordinateSystem(0.1);
    // viewer->initCameraParameters();
    viewer->removeAllPointClouds();
    viewer->addPolygonMesh(*mesh);
    viewer->spin();

    // voxel_grid.setInputCloud(globalMap);
    // voxel_grid.setLeafSize(0.02, 0.02, 0.02);
    // voxel_grid.filter(*globalMap);
    // viewer->removeAllPointClouds();
    // viewer->addPointCloud<PointT>(globalMap);
    // viewer->spin();
    // pcl::io::savePLYFileASCII("globalMap.ply", *globalMap);
}

void readKFTrajectory(const string &keyframeFile, const string &sequencePath, vector<cv::Mat> &vImRGB, vector<cv::Mat> &vImD, vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &vTwc)
{
    ifstream infile;
    infile.open(keyframeFile.data());
    assert(infile.is_open());

    std::string l;
    int cnt = 0;
    while (getline(infile, l))
    {
        string time, tx, ty, tz, x, y, z, w;
        stringstream ss(l);
        ss >> time >> tx >> ty >> tz >> x >> y >> z >> w;
        cout << "reading idx: " << cnt << endl;
        cnt++;
        // if (cnt > 1200)
        //     break;
        if (cnt > 50)
            break;

        // pose
        Eigen::Quaterniond q(stod(w), stod(x), stod(y), stod(z));
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.rotate(q.toRotationMatrix());
        Twc.pretranslate(Eigen::Vector3d(stod(tx), stod(ty), stod(tz)));
        vTwc.emplace_back(Twc);

        // keyframes
        cv::Mat imRGB = cv::imread(sequencePath + "/rgb/" + time + ".png");
        cv::Mat imD = cv::imread(sequencePath + "/depth/" + time + ".png", -1);
        vImRGB.emplace_back(imRGB);
        vImD.emplace_back(imD);
    }
    infile.close();
}

void unProject(const cv::Mat &color_, const cv::Mat &depth_, const CAM_PARAM &cam_, PointCloud::Ptr cloud_)
{
	const ushort *depthPtr;
	const cv::Vec3b *rgbPtr;
	for (int m = 0; m < depth_.rows; m++)
	{
		depthPtr = depth_.ptr<ushort>(m);
		rgbPtr = color_.ptr<cv::Vec3b>(m);
		for (int n = 0; n < depth_.cols; n++)
		{
			ushort d = depthPtr[n];
			// if (d == 0)
			// 	continue;
			PointT p;

			p.z = double(d) / cam_.factor_;
			p.x = (n - cam_.cx_) * p.z / cam_.fx_;
			p.y = (m - cam_.cy_) * p.z / cam_.fy_;

			cv::Vec3b bgr = rgbPtr[n];
			p.b = static_cast<uchar>(bgr[0]);
			p.g = static_cast<uchar>(bgr[1]);
			p.r = static_cast<uchar>(bgr[2]);
			// p.b = 255;
			// p.g = 255;
			// p.r = 255;

			cloud_->points.push_back(p);
		}
	}
    cloud_->height = depth_.rows;
    cloud_->width = depth_.cols; //cloud_->points.size();
    cloud_->is_dense = true;
}