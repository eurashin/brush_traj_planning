#include "opencv2/surface_matching.hpp"
#include <iostream>
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

static void help(const string& errorMessage)
{
    cout << "Program init error : "<< errorMessage << endl;
    cout << "\nUsage : ppf_matching [input model file] [input scene file]"<< endl;
    cout << "\nPlease start again with new parameters"<< endl;
}

int main(int argc, char** argv)
{
    // welcome message
    cout << "****************************************************" << endl;
    cout << "* Surface Matching demonstration : demonstrates the use of surface matching"
             " using point pair features." << endl;
    cout << "* The sample loads a model and a scene, where the model lies in a different"
             " pose than the training.\n* It then trains the model and searches for it in the"
             " input scene. The detected poses are further refined by ICP\n* and printed to the "
             " standard output." << endl;
    cout << "****************************************************" << endl;
    
    if (argc < 3)
    {
        help("Not enough input arguments");
        exit(1);
    }
    
#if (defined __x86_64__ || defined _M_X64)
    cout << "Running on 64 bits" << endl;
#else
    cout << "Running on 32 bits" << endl;
#endif
    
#ifdef _OPENMP
    cout << "Running with OpenMP" << endl;
#else
    cout << "Running without OpenMP and without TBB" << endl;
#endif
    
    string modelFileName = (string)argv[1];
    string sceneFileName = (string)argv[2];
    string finalFileName = (string)argv[3];

    // Initialize pointcloud values
    double viewpoint[3] = {0,0,0};
    Mat pc_normals, pc_test_normals;
   
    // Read the model 
    Mat pc = loadPLYSimple(modelFileName.c_str(), 0);
    computeNormalsPC3d(pc, pc_normals, 6, false, viewpoint);
    
    // Read the scene
    Mat pc_test = loadPLYSimple(sceneFileName.c_str(), 0);
    computeNormalsPC3d(pc_test, pc_test_normals, 6, false, viewpoint);
    
    writePLY(pc_test_normals, "aretherenormals.ply");

    // Now train the model
    cout << "Training..." << endl;
    int64 tick1 = cv::getTickCount();
    ppf_match_3d::PPF3DDetector detector(0.025, 0.05);
    detector.trainModel(pc_normals);
    int64 tick2 = cv::getTickCount();
    cout << endl << "Training complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl << "Loading model..." << endl;
         

    // Match the model to the scene and get the pose
    cout << endl << "Starting matching..." << endl;
    vector<Pose3DPtr> results;
    tick1 = cv::getTickCount();
    detector.match(pc_test_normals, results, 0.05, 0.05);
    tick2 = cv::getTickCount();
    cout << endl << "PPF Elapsed Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
        

    // Get only first N results
    int N = 1;
    vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);
  
    // Create an instance of ICP
    ICP icp(100, 0.005f, 2.5f, 8);
    int64 t1 = cv::getTickCount();
   
   /* 
    // Register for all selected poses
    cout << endl << "Performing ICP on " << N << " poses..." << endl;
    icp.registerModelToScene(pc_normals, pc_test_normals, resultsSub);
    int64 t2 = cv::getTickCount();
    
    cout << endl << "ICP Elapsed Time " <<
         (t2-t1)/cv::getTickFrequency() << " sec" << endl;
     */
    // debug first five poses
    Pose3DPtr result = resultsSub[0];
    result->printPose();
    Mat pct = transformPCPose(pc_normals, result->pose);
    
    cout << finalFileName << endl; 
    writePLY(pct, finalFileName.c_str());
    
    return 0;
}  
