#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudType; 
struct st_vals
{
  int lid_idx;
  int st_idx;
  float corr_vals;
  double ang;
 // bool operator<(const st_vals& rhs) const { corr_vals < rhs.corr_vals; }
};
//comparator
bool dist_compare(st_vals lhs, st_vals rhs) { return lhs.corr_vals < rhs.corr_vals; }
bool ang_compare(st_vals lhs, st_vals rhs) { return lhs.ang > rhs.ang; }
namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
  return p.z;
      }
    };
} 

class LSCalibration 
{
 public: 
   // int getPCDdata(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target)
  LSCalibration(typename pcl::PointCloud<pcl::PointXYZ>::Ptr S_Stereo,
                  typename pcl::PointCloud<pcl::PointXYZ>::Ptr T_Lidar)
   : Source_Stereo(S_Stereo),
     Target_Lidar(T_Lidar),
     Source_Stereo_segmented(new pcl::PointCloud<pcl::PointXYZ>),
     Target_Lidar_segmented(new pcl::PointCloud<pcl::PointXYZ>),
     Source_Stereo_keypoints(new pcl::PointCloud<pcl::PointXYZ>),
     Target_Lidar_keypoints(new pcl::PointCloud<pcl::PointXYZ>),
     Source_Stereo_features (new pcl::PointCloud<pcl::FPFHSignature33>),
     Target_Lidar_features (new pcl::PointCloud<pcl::FPFHSignature33>),
     correspondences_ (new pcl::Correspondences),
     Source_Stereo_correntropy(new pcl::PointCloud<pcl::PointXYZ>),
     Target_Lidar_correntropy(new pcl::PointCloud<pcl::PointXYZ>),
     source_transformed_(new pcl::PointCloud<pcl::PointXYZ>),
     source_registered_(new pcl::PointCloud<pcl::PointXYZ>)

   {
     visualizer_.registerKeyboardCallback(&LSCalibration::keyboard_callback, *this, 0);
     
     // Step1 : Remove nan Values 
     removenan();
     
     // Step:2 Perform RANSAC segmenation to detect the planes on both the Source(Stereo) and the Target(Lidar)

     segmentation(Source_Stereo,Source_Stereo_segmented,0.002,0.002,1000,25000);
     segmentation(Target_Lidar,Target_Lidar_segmented,0.002,2.0,1000,25000);

     // Step3: Detect Keypoints using Sift3d.

     detectKeypoints_sift3d(Source_Stereo_segmented,Source_Stereo_keypoints,0.0009f,4,4,0.0005f);
     detectKeypoints_sift3d(Target_Lidar_segmented,Target_Lidar_keypoints,0.009f,4,16,0.0005f);
    
     CalculateCorrentropy(Target_Lidar_keypoints,Source_Stereo_keypoints,Target_Lidar_correntropy,Source_Stereo_correntropy,1000);
     // // S
     extractDescriptors(Source_Stereo_correntropy,Source_Stereo_correntropy,Source_Stereo_features);
     extractDescriptors(Target_Lidar_correntropy,Target_Lidar_correntropy,Target_Lidar_features);
     
     findCorrespondences(Source_Stereo_features,Target_Lidar_features,source2target_);
      findCorrespondences(Target_Lidar_features,Source_Stereo_features,target2source_);
     
     filterCorrespondences ();
     
      

      determineInitialTransformation ();

    // CalculateCorrentropy(Target_Lidar_keypoints,Source_Stereo_keypoints,Target_Lidar_correntropy,Source_Stereo_correntropy,1000);
    

     determineFinalTransformation ();


     
   }
   void removenan()
   {
      std::vector<int> li_indices,st_indices;
      pcl::removeNaNFromPointCloud(*Source_Stereo, *Source_Stereo, st_indices);
      pcl::removeNaNFromPointCloud(*Target_Lidar, *Target_Lidar, li_indices);
      std::cout<< "\nStereo Point Cloud size = "<<Source_Stereo->points.size() <<std::endl;
      std::cout<< "\nLidar Point Cloud size = " <<Target_Lidar->points.size() <<std::endl;
      // boost::shared_ptr<std::vector<int>> st_indices(new std::vector<int>);
      // boost::shared_ptr<std::vector<int>> li_indices(new std::vector<int>);
      // pcl::removeNaNFromPointCloud(*Source_Stereo, *st_indices);
      // pcl::ExtractIndices<pcl::PointXYZ> extract;
      // extract.setInputCloud(Source_Stereo);
      // extract.setIndices(st_indices);
      // extract.setNegative(true);
      // extract.filter(*Source_Stereo);

      // pcl::removeNaNFromPointCloud(*Target_Lidar, *li_indices);
      // extract.setInputCloud(Target_Lidar);
      // extract.setIndices(li_indices);
      // extract.filter(*Target_Lidar);
   }

 

   void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr data_points, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr segmented,
                     double dist_thr, 
                     double cluster_tol, 
                     double cluster_min_sz,
                     double cluster_max_sz)
   {
    std::cout<<"Perform Segmentation";
    // fit plane and keep points above that plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (dist_thr);

    seg.setInputCloud (data_points);
    seg.segment (*inliers, *coefficients);
    //std::cout<<"\ninlierssize= "<<inliers->indices.size();
    if (inliers->indices.size () == 0)
     {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return ;
     }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (data_points);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*segmented);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
    std::cout << "OK sz= " << segmented->points.size()<<std::endl;


   // std::cout << "clustering..." << std::flush;
  // euclidean clustering
    // typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud (segmented);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    // clustering.setClusterTolerance (cluster_tol); // 2cm
    // clustering.setMinClusterSize (cluster_min_sz);
    // clustering.setMaxClusterSize (cluster_max_sz);
    // clustering.setSearchMethod (tree);
    // clustering.setInputCloud(segmented);
    // clustering.extract (cluster_indices);

    // if (cluster_indices.size() > 0)//use largest cluster
    // {
    //  cout << cluster_indices.size() << " clusters found";
    //  if (cluster_indices.size() > 1) cout <<" Using largest one...";
    //  cout << endl;
    //  typename pcl::IndicesPtr indices (new std::vector<int>);
    //  *indices = cluster_indices[0].indices;
    //  extract.setInputCloud (segmented);
    //  extract.setIndices (indices);
    //  extract.setNegative (false);

    //  extract.filter (*segmented);
    //  }
   }

   void detectKeypoints_sift3d(pcl::PointCloud<pcl::PointXYZ>::Ptr data_points, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                        double min_scale,int n_octaves,int n_scales_per_octave, double min_contrast)
   {
    // Using Sift
     pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
     pcl::PointCloud<pcl::PointWithScale> result;
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
     sift.setSearchMethod(tree);
     sift.setScales(min_scale, n_octaves, n_scales_per_octave);
     sift.setMinimumContrast(min_contrast);
     sift.setInputCloud(data_points);
     sift.compute(result);

     std::cout << "No of SIFT points in the result are " << result.size () << std::endl;
      // Copying the pointwithscale to pointxyz so as visualize the cloud
     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
     copyPointCloud(result, *keypoints);
   }

   void extractDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr data_points,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features)
   {
     typename pcl::PointCloud<pcl::PointXYZ>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZ>);
     kpts->points.resize(keypoints->points.size());
     pcl::copyPointCloud(*keypoints, *kpts);
     
     std::cout << "normal estimation..." << std::flush;
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
     normal_estimation.setSearchMethod (tree);
     normal_estimation.setRadiusSearch (2.0);
     normal_estimation.setInputCloud (kpts);
     normal_estimation.compute (*normals);

     

     pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
     fpfh_estimation.setInputCloud (kpts);
     fpfh_estimation.setInputNormals (normals);
     fpfh_estimation.setSearchMethod(tree);
     fpfh_estimation.setRadiusSearch (2.0);
     fpfh_estimation.compute (*fpfh_features);

     std::cout << "\noutput size (): " << fpfh_features->size () << std::endl;

     std::cout<<"\nOK";
     // pcl::Feature<pcl::PointXYZ, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
     // pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> > (feature_extractor);

     // //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
     // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     // feature_extractor->setSearchMethod (tree);
     // feature_extractor->setRadiusSearch (0.05);
     
     // feature_extractor->setSearchSurface(data_points);
     // feature_extractor->setInputCloud(kpts);
     
     // feature_extractor->setInputNormals(normals);
     // std::cout << "OK" << endl;
     // feature_extractor->compute(*fpfh_features);
    
   }

   void findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features_source,
                            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features_target,
                            std::vector<int>& correspondences)
   {
    cout << "correspondence assignment..." << std::flush;
    correspondences.resize (fpfh_features_source->size());
    std::cout<<"\nfeature_size= "<<fpfh_features_target->size();
    // Use a KdTree to search for the nearest matches in feature space
    
    pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (fpfh_features_target);
    
    std::cout<<"\nS2;";
    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
     const int k = 1;
     std::vector<int> k_indices (k);
     std::vector<float> k_squared_distances (k);
      for (int i = 0; i < static_cast<int> (fpfh_features_source->size ()); ++i)
      {
       descriptor_kdtree.nearestKSearch (*fpfh_features_source, i, k, k_indices, k_squared_distances);
       correspondences[i] = k_indices[0];
      }
       cout << "\nCorrespondences estimation ->OK" << endl;
  }

  void filterCorrespondences()
  {
     cout << "correspondence rejection..." << std::flush;
     std::vector<std::pair<unsigned, unsigned> > correspondences;
     for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
      if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
        correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

     correspondences_->resize (correspondences.size());
     for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
      {
       (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
       (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
      }

     pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
     rejector.setInputSource (Source_Stereo_correntropy);
     rejector.setInputTarget (Target_Lidar_correntropy);
     rejector.setInputCorrespondences(correspondences_);
     rejector.getCorrespondences(*correspondences_);
     cout << "\nFiltering correspondences -> OK" << endl;
  }

void CalculateCorrentropy(pcl::PointCloud<pcl::PointXYZ>::Ptr inplicloud,pcl::PointCloud<pcl::PointXYZ>::Ptr inpstcloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr outlicloud,pcl::PointCloud<pcl::PointXYZ>::Ptr outstcloud, int npoints)
{

 // calculate the correntropy.
  float sigma=0.1;
  float max_val=-9999999;
  float max_angle=999999999;
  double corr_th=0.999;
  double ang_th=0.12;
  std::cout<<"\nLicloud size= "<<inplicloud->points.size()<<" st cloud size= "<<inpstcloud->points.size()<<std::endl; 
  int *idx=new int[inpstcloud->points.size()];
  st_vals *sv=new st_vals[inpstcloud->points.size()];
  st_vals *sv_dummy=new st_vals[inpstcloud->points.size()];
  float *corr_vals=new float[inpstcloud->points.size()];
  // int *lidarpts_ctr= new int[inplicloud->points.size()];
  //  int *stereopts_ctr= new int[inpstcloud->points.size()];
  //  int k1=0;
  // for(int j=0;j<inplicloud->points.size();j++)
  //   {
  //     lidarpts_ctr[j]=0;
  //     stereopts_ctr[j]=0;
  //   }
  
  
  for(std::size_t i=0;i<inpstcloud->points.size();i++)
  {
      float st_d=sqrt(((inpstcloud->points[i].x)*(inpstcloud->points[i].x))+
                   ((inpstcloud->points[i].y)*(inpstcloud->points[i].y))+
                   ((inpstcloud->points[i].z)*(inpstcloud->points[i].z)));
      sv[i].st_idx=i;
    for(std::size_t j=0;j<inplicloud->points.size();j++)
    {
      

       float li_d=sqrt(((inplicloud->points[j].x)*(inplicloud->points[j].x))+
                   ((inplicloud->points[j].y)*(inplicloud->points[j].y))+
                   ((inplicloud->points[j].z)*(inplicloud->points[j].z)));
       
     //calculate correntropy
      float correntropy_val=exp(-pow(sqrt((st_d-li_d)*(st_d-li_d)),2)/(2*sigma*sigma));
 
     

      double angle=std::acos((inplicloud->points[j].x*inpstcloud->points[i].x + inplicloud->points[j].y*inpstcloud->points[i].y 
        + inplicloud->points[j].z*inpstcloud->points[i].z)/(li_d*st_d));

      // float distance 
      //float correntropy_val=abs(st_d-li_d);
      // if(correntropy_val>corr_th && angle<ang_th)
      // {
      //   lidarpts_ctr[j]++;
      //   stereopts_ctr[i]++;
        
      // }
      if(correntropy_val>max_val)
      {
        idx[i]=j;
        max_val=correntropy_val;
        sv[i].corr_vals=correntropy_val;
        sv[i].lid_idx=j;
        sv[i].ang=angle;
        //corr_vals[i]=correntropy_val;
        
      }
      
      //std::cout<<"\nCM("<<i<<","<<j<<")="<<CM(i,j)<<" "<<"ctr= "<<ctr;
      //ctr++;
    }
    //if(stereopts_ctr[i]>0)k1++;
    //std::cout<<"idx["<<i<<"] = "<<idx[i]<<std::endl;
    max_val=-99999;
    max_angle=99999999999;
  }
  std::sort(sv,sv+inpstcloud->points.size(),dist_compare);
  memcpy(&sv_dummy,&sv,sizeof(sv));
  std::sort(sv_dummy,sv_dummy+inpstcloud->points.size(),ang_compare);
  //std::sort(sv_dummy,)
  //for (int i=0;i<inpstcloud->points.size();i++)
    //std::cout<<"[st_idx = "<<sv_dummy[i].st_idx<<", li_idx = "<<sv_dummy[i].lid_idx<<" ]= "<<sv_dummy[i].corr_vals<<" , ang = "<<sv_dummy[i].ang<<std::endl;

//for (int i=0;i<inpstcloud->points.size();i++)
  //  std::cout<<"[st_idx = "<<sv[i].st_idx<<", li_idx = "<<sv[i].lid_idx<<" ]= "<<sv[i].corr_vals<<" , ang = "<<sv[i].ang<<std::endl;

 
 double angle_thr=0.2;
 double corr_thresh=0.99;
 int k=0;
 
  for (int j=0;j<inpstcloud->points.size();j++)
  {
     if(sv[inpstcloud->points.size()-1-j].corr_vals>corr_th &&  sv_dummy[inpstcloud->points.size()-1-j].ang<ang_th) k++;
  }

  outstcloud->width=k;
  outstcloud->height=1;
  outstcloud->is_dense=false;
  outstcloud->resize(k*1);

  outlicloud->width=k;
  outlicloud->height=1;
  outlicloud->is_dense=false;
  outlicloud->resize(k*1);
  k=0;
  //Older implmentation///
  for (int j=0;j<inpstcloud->points.size();j++)
  {
    
    //std::cout<<"\nspt= "<<sv[inpstcloud->points.size()-2-j].st_idx;
    //if((sv[inpstcloud->points.size()-1-j].st_idx == sv_dummy[inpstcloud->points.size()-1-j].st_idx) && 
      //(sv[inpstcloud->points.size()-1-j].lid_idx == sv_dummy[inpstcloud->points.size()-1-j].lid_idx) )
    if(sv[inpstcloud->points.size()-1-j].corr_vals>corr_th &&  sv_dummy[inpstcloud->points.size()-1-j].ang<ang_th)
    {
    // std::cout<<"orig = [st_idx = "<<sv[inpstcloud->points.size()-1-j].st_idx<<", li_idx = "<<sv[inpstcloud->points.size()-1-j].lid_idx<<" ]= , corrval = "<<sv[inpstcloud->points.size()-1-j].corr_vals << " , ang = "<<sv[inpstcloud->points.size()-1-j].ang  << 
      //        " dummy = [st_idx = "<<sv_dummy[inpstcloud->points.size()-1-j].st_idx<<", li_idx = "<<sv_dummy[inpstcloud->points.size()-1-j].lid_idx<<" ]= "<<sv_dummy[inpstcloud->points.size()-1-j].ang<<" , ang = "<<sv_dummy[inpstcloud->points.size()-1-j].ang<<std::endl;
     outstcloud->points[k]=inpstcloud->points[sv[inpstcloud->points.size()-1-j].st_idx];
     outlicloud->points[k]=inplicloud->points[sv[inpstcloud->points.size()-1-j].lid_idx];
     k++;
    }
    //if(k==npoints-1) break;
  }

  ///Newer implementation
  // for (int j=0;j<inpstcloud->points.size();j++)
  // {
  //   if(stereopts_ctr[j]>0)
  //   {
  //     outstcloud->points[k]=inpstcloud->points[j]; 
  //     k++;
  //   }

  // }
  // k=0;
  // for (int j=0;j<inplicloud->points.size();j++)
  // {
  //   if(lidarpts_ctr[j]>0)
  //   {
  //     outlicloud->points[k]=inplicloud->points[j];
  //     k++;
  //   }
  // }
  //outlicloud->width=npoints;
  //outlicloud->height=1;



}
  void determineInitialTransformation()
  {
     cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);

  transformation_estimation->estimateRigidTransformation (*Source_Stereo_correntropy, *Target_Lidar_correntropy, *correspondences_, initial_transformation_matrix_);

 pcl::copyPointCloud(*Source_Stereo_correntropy,*source_transformed_);
  
  //pcl::transformPointCloud(*Source_Stereo_correntropy, *source_transformed_, initial_transformation_matrix_);
  cout << "OK" << endl;
  }

  void determineFinalTransformation()
  {

  cout << "final registration..." << std::flush;
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
  trans_svd= pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);
  std::cout<<"\nPoint cloud size for registration, Licloud size= "<<Target_Lidar_correntropy->points.size()<<" st cloud size= "<<Source_Stereo_correntropy->points.size()<<std::endl; 
  registration->setInputSource (source_transformed_);
  //registration->setInputSource (source_segmented_);
  registration->setInputTarget (Target_Lidar_correntropy);
  registration->setMaxCorrespondenceDistance(2.0);
  registration->setRANSACOutlierRejectionThreshold (2.0);
  registration->setTransformationEpsilon (0.0001);
  registration->setMaximumIterations (100000);
  registration->setTransformationEstimation(trans_svd);

  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  std::cout<<"\n Src size="<< Source_Stereo_correntropy->points.size()<<"Tget size="<<Target_Lidar_correntropy->points.size()<<"\n";
  std:cout<<"\nInitial Transfromation matrix= \n"<<initial_transformation_matrix_;
  std::cout<<"\ntransformation_matrix=\n"<<transformation_matrix_<<std::endl;
  cout << "OK" << endl;
  }
   void run()
   {
   
    visualizer_.spin ();
   }

   void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
   {
    if (event.keyUp())
    {
    switch (event.getKeyCode())
    {
      case '1':
        if (!visualizer_.removePointCloud("Source_Stereo_points"))
        {

          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> st_color(Source_Stereo, 255, 0, 0);
          visualizer_.addPointCloud(Source_Stereo,st_color, "Source_Stereo_points");
        }
        break;

      case '2':
        if (!visualizer_.removePointCloud("Target_Lidar_points"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> li_color(Target_Lidar, 0, 255, 0);
          visualizer_.addPointCloud(Target_Lidar, "Target_Lidar_points");
        }
        break;

      case '3':
        if (!visualizer_.removePointCloud("Source_Stereo_segmented"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> st_color(Source_Stereo_segmented, 255, 0, 0);
          visualizer_.addPointCloud(Source_Stereo_segmented,st_color, "Source_Stereo_segmented");
        }
        break;
      case '4':
        if (!visualizer_.removePointCloud("Target_Lidar_segmented"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> li_color(Target_Lidar_segmented, 0, 255, 0);
          visualizer_.addPointCloud(Target_Lidar_segmented, li_color,"Target_Lidar_segmented");
        }
        break;
      case '5':
        if (!visualizer_.removePointCloud("Source_Stereo_keypoints"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> st_color(Source_Stereo_keypoints, 255, 0, 0);
          visualizer_.addPointCloud(Source_Stereo_keypoints,st_color, "Source_Stereo_keypoints");
        }
        break;
      case '6':
        if (!visualizer_.removePointCloud("Target_Lidar_keypoints"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> li_color(Target_Lidar_keypoints, 0, 255, 0);
          visualizer_.addPointCloud(Target_Lidar_keypoints, li_color,"Target_Lidar_keypoints");
        }
        break;


      case '7':
        if (!show_source2target_)
          visualizer_.addCorrespondences<pcl::PointXYZ>(Source_Stereo_keypoints, Target_Lidar_keypoints, source2target_, "source2target");
        else
          visualizer_.removeCorrespondences("source2target");

        show_source2target_ = !show_source2target_;
        break;

      case '8':
        if (!show_target2source_)
          visualizer_.addCorrespondences<pcl::PointXYZ>(Target_Lidar_keypoints, Source_Stereo_keypoints, target2source_, "target2source");
        else
          visualizer_.removeCorrespondences("target2source");

        show_target2source_ = !show_target2source_;
        break;
       case 'c':
        if (!visualizer_.removePointCloud("Source_Stereo_correntropy"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> st_color(Source_Stereo_correntropy, 255, 0, 0);
          visualizer_.addPointCloud(Source_Stereo_correntropy,st_color, "Source_Stereo_correntropy");
        }
        break;
      case 'd':
        if (!visualizer_.removePointCloud("Target_Lidar_correntropy"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> li_color(Target_Lidar_correntropy, 0, 255, 0);
          visualizer_.addPointCloud(Target_Lidar_correntropy, li_color,"Target_Lidar_correntropy");
        }
        break;
      case 'i':
      case 'I':
        if (!visualizer_.removePointCloud("transformed"))
          visualizer_.addPointCloud(Source_Stereo_correntropy, "transformed");
        break;

      case 'r':
      case 'R':
        if (!visualizer_.removePointCloud("registered"))
          visualizer_.addPointCloud(Target_Lidar_correntropy, "registered");
        break;

      }
    }
  }
   private: 
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Source_Stereo;
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Lidar;
   
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Source_Stereo_segmented;
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Lidar_segmented;

   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Source_Stereo_keypoints;
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Lidar_keypoints;

   typename pcl::PointCloud<pcl::FPFHSignature33>::Ptr Source_Stereo_features;
   typename pcl::PointCloud<pcl::FPFHSignature33>::Ptr Target_Lidar_features;

   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Source_Stereo_correntropy;
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr Target_Lidar_correntropy;

   typename pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed_;
   typename pcl::PointCloud<pcl::PointXYZ>::Ptr source_registered_;


    std::vector<int> source2target_;
    std::vector<int> target2source_;

   Eigen::Matrix4f initial_transformation_matrix_;
   Eigen::Matrix4f transformation_matrix_;
   
   bool show_source2target_;
   bool show_target2source_;
   bool show_correspondences;

   pcl::CorrespondencesPtr correspondences_;
   pcl::visualization::PCLVisualizer visualizer_;
  
   
};


int main(int argc, char ** argv)
{
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr S_Stereo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *S_Stereo);

  pcl::PointCloud<pcl::PointXYZ>::Ptr T_Lidar (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[2], *T_Lidar);
  
  LSCalibration lsc(S_Stereo,T_Lidar);
  lsc.run();
   
  return 0;

}