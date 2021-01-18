#include "my_slam6d/my_slam6d.h"

My_slam6d::~My_slam6d() {

  // clean up
  delete my_icp;
  delete my_loopSlam6D;
  delete my_graphSlam6D;
  delete my_icp6Dminimizer;

  Scan::closeDirectory();
}

void My_slam6d::updateAlgorithms() {
    /* set icp minimization method to dual quaternion based
    * constructor param: quiet mode?
    */
    if(this->my_icp6Dminimizer != nullptr) delete this->my_icp6Dminimizer;
    this->my_icp6Dminimizer = new icp6D_QUAT(quiet);

    /* set slam method
    * param1: icp minimizer 
    * param2: max p2p dist ICP
    * param3: max p2p dist SLAM
    * param4: max iterations ICP
    * param5: quiet mode?
    * param6: match against meta scan?
    * param7: randomized point reduction (every n-th point or -1 for disabled))
    * param8: extrapolate pose?
    * param9: use only every n-th point for animation or -1 for disabled
    * param10: epsilon ICP
    * param11: nns_method (0 = simple ks, 1 = cached kd)
    * param12: epsilon SLAM
    */
    if(this->my_graphSlam6D != nullptr) delete this->my_graphSlam6D;
    this->my_graphSlam6D = new lum6DEuler(
        my_icp6Dminimizer,
        max_p2p_dist_ICP, 
        max_p2p_dist_SLAM, 
        max_it_ICP, 
        quiet, 
        match_meta, 
        random_red, 
        extrapolate_pose,
        anim,
        epsilon_ICP, 
        nns_method, 
        epsilon_SLAM
    );

    /* create icp
    * param1: icp minimizer 
    * param2: max p2p dist ICP
    * param3: max iterations ICP
    * param4: quiet mode?
    * param5: match against meta scan?
    * param6: randomized point reduction (every n-th point or -1 for disabled)
    * param7: extrapolate pose?
    * param8: use only every n-th point for animation or -1
    * param8: epsilon ICP
    * param10: nns_method (0 = simple ks, 1 = cached kd)
    */
    if(this->my_icp != nullptr) delete this->my_icp; 
    this->my_icp = new icp6D(my_icp6Dminimizer,
        max_p2p_dist_ICP, 
        max_it_ICP,
        quiet,
        match_meta, 
        random_red, 
        extrapolate_pose,
        anim, 
        epsilon_ICP, 
        nns_method
    );

    /* create loop closure
    * param1: very quiet mode?
    * param2: icp minimizer
    * param3: max p2p distance loop closing
    * param4: max loop closing iterations
    * param5: random point reduction (every n-th point or -1 for disabled)
    */
    if(this->my_loopSlam6D != nullptr) delete this->my_loopSlam6D;
    this->my_loopSlam6D = new elch6Dquat(
        very_quiet, 
        my_icp6Dminimizer, 
        max_p2p_dist_Loop, 
        max_it_Loop, 
        random_red, 
        extrapolate_pose, 
        10, // anim = 10 
        epsilon_ICP, 
        nns_method
    );
}

/**
 * Main program for 6D SLAM.
 * Usage: bin/slam6D 'dir',
 * with 'dir' the directory of a set of scans
 * ...
 */
void My_slam6d::do_slam6d()
{
    cout << "in do_slam6d" << endl;

    cout << "slam6D - "
        << "A highly efficient SLAM implementation based on scan matching"
        << endl
        << "         with 6 degrees of freedom" << endl
        << "(c) University of Wuerzburg, Germany, since 2013" << endl
        << "    Jacobs University Bremen gGmbH, Germany, 2009 - 2013" << endl
        << "    University of Osnabrueck, Germany, 2006 - 2009" << endl << endl;

    /* read scans from directory
    * param1: scanserver active?
    * param2: path
    * param3: scan file format
    * param4: first scan
    * param5: last scan
    */ 
    Scan::openDirectory(scanserver, dir_path, file_format, start, end); 
    
    // after open directory all scans will be in static variable Scan::allScans
    if(Scan::allScans.size() == 0) {
        cerr << "No scans found. Did you use the correct format?" << endl;
        exit(-1);
    }

    // set searchtree and reduction parameter for all scans
    for(ScanVector::iterator it = Scan::allScans.begin();
        it != Scan::allScans.end();
        ++it) {
        Scan* scan = *it;
        unsigned int types = 0; // if pairing mode != CLOSEST_POINT: types ? PointTypes::USE_NORMAL
        scan->setReductionParameter(red_voxel_size, octree_red, PointType(types));
        scan->setSearchTreeParameter(nns_method, bucket_size);
    }
    this->max_it_ICP = 50;

    this->updateAlgorithms();

    /* do 6DSLAM
    * param1: max distance for loop closing
    * param2: loop size
    * param3: scans
    * param4: icp
    * param5: match against meta?
    * param6: nns method (0 = simple kd, 1 = cached kd)
    * param7: loop algo
    * param8: slam algo
    * param9: max iterations slam
    * param10: epsilon SLAM
    * param11: max p2p dist SLAM
    * param12: max p2p dist final SLAM
    * param13: max dist for final loop closing
    * param14: extrapolate pose?
    * param15: scan file format
    */ 
    this->matchGraph6Dautomatic(
        max_dist_Loop, 
        loopsize, 
        Scan::allScans, 
        my_icp, 
        match_meta, 
        nns_method, 
        my_loopSlam6D, 
        my_graphSlam6D, 
        max_it_SLAM, 
        epsilon_SLAM, 
        max_p2p_dist_SLAM, 
        max_p2p_dist_finalSLAM, 
        max_dist_final_Loop, 
        extrapolate_pose,
        file_format,
        start,
        end
    );
}

void My_slam6d::improve_slam6d() {
    cout << "in improve_slam6d" << endl;
    
    this->max_it_ICP = 50;

    this->updateAlgorithms();

    this->improve_start = 20;
    this->improve_end = 40;

    std::vector<Scan*> scans_for_improving;
    int start_index = start - this->improve_start;
    int end_index = end - this->improve_start;

    // remember pose of end node
    double old_end[16]; 
    memcpy(old_end, Scan::allScans[end_index]->get_transMat(), sizeof(old_end)); 

    if(this->my_icp != nullptr) {
            // move scans back to origin 
        for(int i = start_index; i <= end_index; i++) {
            Scan* scan = Scan::allScans[i];
            double trans[16];
            M4inv(scan->getDAlign(), trans);
            scan->transform(trans, Scan::ICP);
            scans_for_improving.push_back(scan);
        }
    }
   
    // merge first scan position with position of scan before
    if(start > this->improve_start) {
        Scan::allScans[start_index]->mergeCoordinatesWithRoboterPosition(Scan::allScans[start_index-1]);
    }
 
    this->matchGraph6Dautomatic(
        max_dist_Loop, 
        loopsize, 
        scans_for_improving, 
        my_icp, 
        match_meta, 
        nns_method, 
        my_loopSlam6D, 
        my_graphSlam6D, 
        max_it_SLAM, 
        epsilon_SLAM, 
        max_p2p_dist_SLAM, 
        max_p2p_dist_finalSLAM, 
        max_dist_final_Loop, 
        extrapolate_pose,
        file_format,
        improve_start,
        improve_end
    );

    // update transmat of any scans that come after the section to improve
    if(end == this->improve_end) {
        return;
    }
    

    double temp[16];
    M4inv(old_end, temp);
    double delta[16];
    MMult(Scan::allScans[end_index]->get_transMat(), temp, delta);
    
    for(int i = end_index + 1; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->transform(delta, Scan::ICP);
    }

    
}

void My_slam6d::finish() {
    
    double id[16];
    M4identity(id);

    for(size_t i= 0; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->clearFrames();
    }

    for(size_t i= 0; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->transform(id, Scan::ICP, 0);
    }

    // write frames to file:
    const double* p;
    ofstream redptsout("/home/jannik/Bachelorarbeit/data/hannover1/loopclose.pts");
    for(ScanVector::iterator it = Scan::allScans.begin();
        it != Scan::allScans.end();
        ++it)
    {
        Scan* scan = *it;
        p = scan->get_rPos();
        Point x(p[0], p[1], p[2]);
        redptsout << x << endl;
        scan->saveFrames(false);
    }
    redptsout.close();
}
/**
 * This function is does all the matching stuff
 * it iterates over all scans using the algorithm objects to calculate new poses
 * objects could be NULL if algorithm should not be used
 *
 * @param cldist maximal distance for closing loops
 * @param loopsize minimal loop size
 * @param allScans Contains all laser scans
 * @param my_icp6D the ICP implementation
 * @param meta_icp math ICP against a metascan
 * @param nns_method Indicates the nearest neigbor search method to be used
 * @param my_loopSlam6D used loopoptimizer
 * @param my_graphSlam6D used global optimization
 * @param nrIt The number of iterations the global SLAM-algorithm will run
 * @param epsilonSLAM epsilon for global SLAM iteration
 * @param mdml maximal distance match for global SLAM
 * @param mdmll max distance match for global SLAM after all scans are matched
 */
void My_slam6d::matchGraph6Dautomatic(double cldist,
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
                        int end)
{
    double cldist2 = sqr(cldist);

    // list of scan for metascan
    vector < Scan* > metas;

    // graph for loop optimization
    graph_t g;

    int n = allScans.size();

    int loop_detection = 0;
    double dist, min_dist = -1;
    int first = 0, last = 0;

    // add edges from overall start to start variable
    // for(int i = start + 1; i <= start; i++) {
    //     add_edge(i-1, i, g);
    // }

    for(int i = 1 ; i < n ; i++) {
        cout << start + i << "/" << end << endl;

        add_edge(i-1, i, g);

        if(eP && my_icp6D != NULL) {
            allScans[i]->mergeCoordinatesWithRoboterPosition(allScans[i-1]);
        }

        //Hack to get all icp transformations into the .frames Files
        if(i == n-1 && my_icp6D != NULL && my_icp6D->get_anim() == -2) {
            my_icp6D->set_anim(-1);
        }

        /*if(i == 85 || i == 321 || i == 533) {
        my_icp6D->set_anim(1);
        }*/

        if(my_icp6D != NULL){
        cout << "ICP" << endl;
        // Matching strongly linked scans with ICPs
        if(meta_icp) {
            metas.push_back(allScans[i - 1]);
            MetaScan* meta_scan = new MetaScan(metas);
            my_icp6D->match(meta_scan, allScans[i]);
            delete meta_scan;
        } else {
            switch(type) {
            case UOS_MAP:
            case UOS_MAP_FRAMES:
            my_icp6D->match(allScans[0], allScans[i]);
            break;
            case RTS_MAP:
            //untested (and could not work)
            //if(i < 220-22 && i > 250-22) match(allScans[0], CurrentScan);
            my_icp6D->match(allScans[0], allScans[i]);
            break;
            default:
            my_icp6D->match(allScans[i - 1], allScans[i]);
            break;
            }
        }
        } else {
            double id[16];
            M4identity(id);
            allScans[i]->transform(id, Scan::ICP, 0);
        }

        /*if(i == 85 || i == 321 || i == 533) {
        my_icp6D->set_anim(-2);
        }*/

        if(loop_detection == 1) {
            loop_detection = 2;
        }

        for(int j = 0; j < i - loopsize; j++) {
            dist = Dist2(allScans[j]->get_rPos(), allScans[i]->get_rPos());
            if(dist < cldist2) {
                loop_detection = 1;
                if(min_dist < 0 || dist < min_dist) {
                min_dist = dist;
                first = j;
                last = i;
                }
            }
        }

        if(loop_detection == 2) {
            loop_detection = 0;
            min_dist = -1;

            if(my_loopSlam6D != NULL) {
                cout << "Loop close: " << first << " " << last << endl;
                my_loopSlam6D->close_loop(allScans, first, last, g);
                add_edge(first, last, g);
            }

            if(my_graphSlam6D != NULL && mdml > 0) {
                int j = 0;
                double ret;
                do {
                    // recalculate graph
                    Graph *gr = new Graph(i + 1, cldist2, loopsize);
                    cout << "Global: " << j << endl;
                    ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
                    delete gr;
                    j++;
                } while (j < nrIt && ret > epsilonSLAM);
            }
        }
    }

    if(loop_detection == 1 && my_loopSlam6D != NULL) {
        cout << "Loop close: " << first << " " << last << endl;
        my_loopSlam6D->close_loop(allScans, first, last, g);
        add_edge(first, last, g);
    }

    

    if(my_graphSlam6D != NULL && mdml > 0.0) {
        int j = 0;
        double ret;
        do {
            // recalculate graph
            Graph *gr = new Graph(n, cldist2, loopsize);
            cout << "Global: " << j << endl;
            ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
            delete gr;
            j++;
        } while (j < nrIt && ret > epsilonSLAM);
    }

    if(my_graphSlam6D != NULL && mdmll > 0.0) {
        my_graphSlam6D->set_mdmll(mdmll);
        int j = 0;
        double ret;
        do {
            // recalculate graph
            Graph *gr = new Graph(n, sqr(graphDist), loopsize);
            cout << "Global: " << j << endl;
            ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
            delete gr;
            j++;
        } while (j < nrIt && ret > epsilonSLAM);
    }
}

