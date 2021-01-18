#include "slam6d/scan.h"
#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"

#include "slam6d/icp6Dapx.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6Dortho.h"
#include "slam6d/icp6Dhelix.h"
#include "slam6d/icp6Ddual.h"
#include "slam6d/icp6Dlumeuler.h"
#include "slam6d/icp6Dlumquat.h"
#include "slam6d/icp6Dquatscale.h"
#include "slam6d/icp6Dnapx.h"
#include "slam6d/icp6D.h"
#include "slam6d/lum6Deuler.h"
#include "slam6d/lum6Dquat.h"
#include "slam6d/ghelix6DQ2.h"
#include "slam6d/elch6Deuler.h"
#include "slam6d/elch6Dquat.h"
#include "slam6d/elch6DunitQuat.h"
#include "slam6d/elch6Dslerp.h"
#include "slam6d/graphSlam6D.h"
#include "slam6d/gapx6D.h"
#include "slam6d/graph.h"
#include "slam6d/globals.icc"

#include <csignal>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS


#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#define WANT_STREAM ///< define the WANT stream :)

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;

#include <boost/filesystem.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class My_slam6d {

public:

    void do_slam6d();
    void improve_slam6d();
    void finish();

    ~My_slam6d();

private:

    std::string dir_path = "/home/jannik/Bachelorarbeit/data/hannover1/";

    IOType file_format = UOS;

    int start = 0;
    int end = 65;

    int improve_start = 0;
    int improve_end = 0;

    double epsilon_ICP = 0.00001;
    int max_it_ICP = 10;
    double max_p2p_dist_ICP = 75.0;
    
    double epsilon_SLAM = 0.5;
    int max_it_SLAM = 50;
    double max_p2p_dist_SLAM = 250.0;
    double max_p2p_dist_finalSLAM = -1.0;

    int max_it_Loop = 100;
    double max_p2p_dist_Loop = 700.0;
    double max_dist_Loop = 750.0;
    double max_dist_final_Loop = 750.0;
    int loopsize = 20;

    int nns_method = 0;
    int bucket_size = 20;
    PairingMode pairing_mode = CLOSEST_POINT;

    double red_voxel_size = -1.0;
    int octree_red = 0;
    int random_red = -1;

    bool quiet = true;
    bool very_quiet = true;

    bool match_meta = false;
    bool extrapolate_pose = true;
    bool scanserver = false;
    int anim = -1;

    icp6Dminimizer* my_icp6Dminimizer;
    graphSlam6D *my_graphSlam6D;
    icp6D *my_icp;
    loopSlam6D *my_loopSlam6D;

    std::vector<double*> old_transmats;

    void updateAlgorithms();

    /**
     * This function is does all the matching stuff
     * it iterates over all scans using the algorithm objects to calculate new poses
     * objects could be NULL if algorithm should not be used
     */
    void matchGraph6Dautomatic(double cldist,
                            int loopsize,
                            vector <Scan *> allScans,
                            icp6D *my_icp6D,
                            bool meta_icp,
                            int nns_method,
                            loopSlam6D *my_loopSlam6D,
                            graphSlam6D *my_graphSlam6D,
                            int nrIt,
                            double epsilonSLAM,
                            double mdml,
                            double mdmll,
                            double graphDist,
                            bool &eP,
                            IOType type,
                            int start,
                            int end);
};