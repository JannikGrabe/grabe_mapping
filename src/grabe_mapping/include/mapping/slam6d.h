#ifndef GRABE_MAPPING_SLAM6D_H
#define GRABE_MAPPING_SLAM6D_H

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

#endif