#include <vector>
#include "Vec3.h"

struct Scanline { 
  std::vector<int> s_queue;
  std::vector<int> e_queue;
  std::vector<int> labels;
  std::vector< Vec3 > points;
};

class PointCloudSegmenter {
  private:
    std::vector<Vec3> p_gnd;  //Pts belonging to ground surface
    std::vector<Vec3> p_ngnd; //Pts not belonging to ground surface


  public:
    std::vector<Vec3> p_all;
    double th_seeds; //Threshold for points considered to be initial seeds
    double th_dist;  //Threshold distance from plane

    int n_iter; //Num of iterations (segments in x dir)
    int n_lpr;  //Num of pts used to determine LPR
    int n_segs; //Num of segments to split data into

    int n_scanlines;
    int new_label;
    double th_run;
    double th_merge;

    int max_x;
    int max_y;

    PointCloudSegmenter(int m_x, int m_y, int iterations, int num_lpr, int num_segs, double seed_thresh, double dist_thresh, int n_scan, double run_thresh, double merge_thresh) { //Constructor
      n_iter = iterations;
      n_lpr = num_lpr;
      n_segs = num_segs;
      th_seeds = seed_thresh;
      th_dist = dist_thresh;
      max_x = m_x;
      max_y = m_y;
      n_scanlines = n_scan;
      th_run = run_thresh;
      th_merge = merge_thresh;
    } 

    PointCloudSegmenter() {
      n_iter = 0;
      n_lpr = 0;
      n_segs = 0;
      th_seeds = 0;
      th_dist = 0;
      max_x = 40;
      max_y = 20;
      n_scanlines = 120;
      th_run = 0.5;
      th_merge = 1.0;
    }

    ~PointCloudSegmenter() {} //Deconstructor

    //********************** GPF ***************************

    void GroundPlaneFitting( std::vector<Vec3>& cloud ); //Main Loop
    
    void ExtractInitialSeeds(std::vector<Vec3>& cloud_seg, std::vector<Vec3>& seeds); //Returns inital seeds to be used in first plane model estimation

    Vec3 CalculatePlaneNormal(std::vector<Vec3>& cur_p_gnd); //Returns the normal of the estimated ground plane model

    std::vector<Vec3> GetGroundPoints( void ) {  //Returns points that have been classified as ground points
      return p_gnd;
    }

    std::vector<Vec3> GetNonGroundPoints( void ) { //Returns points that have been classified as non ground points
      return p_ngnd;
    }

    //********************** SLR ***************************

    std::vector<int> next;
    std::vector<int> tail;
    std::vector<int> rtable;

    std::vector<Vec3> ScanLineRun(std::vector<Vec3>& cloud); //Main loop

    void FindRuns(Scanline& cur_scanline);

    void UpdateLabels(Scanline &scan_current, Scanline &scan_above);

    void SetRunLabel(Scanline &scan_current, int start_index, int end_index, int label);

    int FindNearestNeighbor(Scanline& scan_current, Scanline& scan_above, int start_index, int end_index, int point_index);

    void MergeLabels(std::vector<int>& labels_to_merge, int min_label);

    void MergeOperation(int u, int v);

    void ResolveMerge(int x, int y);

    std::vector<Vec3> ExtractClusters( std::vector<Scanline>& scanlines );


};

