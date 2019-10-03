// Plan for a MuJoCo environment with OMPL

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>
#include "yaml-cpp/yaml.h"

#include "mujoco_wrapper.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;


int main(int argc, char** argv) {
    string xml_filename = "";
    string prob_config_filename = "";
    if (argc >= 3) {
        xml_filename = argv[1];
        prob_config_filename = argv[2];
    } else {
        cerr << "Format: plan <MuJoCo XML config> <yaml problem spec> [time limit]"
             << endl;
        return -1;
    }

    // Optional time limit
    double timelimit = 1.0;
    if (argc >= 4) {
        stringstream ss;
        ss << argv[3];
        ss >> timelimit;
    }

    // Load yaml information
    //   This should contain instructions on how to setup the planning problem
    vector<double> start_vec;
    vector<double> goal_vec;
    if (prob_config_filename != "") {
        YAML::Node node = YAML::LoadFile(prob_config_filename);

        // Copy variables
        if (node["start"]) {
            start_vec = node["start"].as<vector<double> >();
        }
        if (node["goal"]) {
            goal_vec = node["goal"].as<vector<double> >();
        }
    }

    // Create MuJoCo Object
    string mjkey_filename = strcat(getenv("HOME"), "/.mujoco/mjkey.txt");
    auto mj(make_shared<MuJoCo>(mjkey_filename));

    // Get xml file name
    // TODO: make this more modern (argparsing, regex)
    //   could use boost.program_options
    if (xml_filename.find(".xml") == string::npos) {
        cerr << "XML model file is required" << endl;
        return -1;
    }

    // Load Model
    cout << "Loading MuJoCo config from: " << xml_filename << endl;
    if (!mj->loadXML(xml_filename)) {
        cerr << "Could not load XML model file" << endl;
        return -1;
    }

    // Make data
    if (!mj->makeData()) {
        cerr << "Could not allocate mjData" << endl;
        return -1;
    }

    // Setup OMPL environment
    auto si = MujocoStatePropagator::createSpaceInformation(mj->m);
    auto mj_state_prop(make_shared<MujocoStatePropagator>(si, mj));
    si->setStatePropagator(mj_state_prop);

    // Create some candidate planners
    auto sst_planner(make_shared<oc::SST>(si));
    auto pdst_planner(make_shared<oc::PDST>(si));
    auto est_planner(make_shared<oc::EST>(si));
    auto kpiece_planner(make_shared<oc::KPIECE1>(si));

    // Create a SimpleSetup object
    oc::SimpleSetup ss(si);

    // TODO: change the optimization objective?
    // auto opt_obj(make_shared<ob::OptimizationObjective>(si));
    // ss.setOptimizationObjective(opt_obj);

    ss.setPlanner(sst_planner);
    // ss.setPlanner(pdst_planner);
    // ss.setPlanner(est_planner);
    // est_planner->setup();
    // ss.setPlanner(kpiece_planner);

    // Set start and goal states
    ob::ScopedState<> start_ss(ss.getStateSpace());
    for(int i=0; i < start_vec.size(); i++) {
        start_ss[i] = start_vec[i];
    }
    ob::ScopedState<> goal_ss(ss.getStateSpace());
    for(int i=0; i < goal_vec.size(); i++) {
        goal_ss[i] = goal_vec[i];
    }
    double threshold = 0.1;
    ss.setStartAndGoalStates(start_ss, goal_ss, threshold);

    // Call the planner
    ob::PlannerStatus solved = ss.solve(timelimit);

    if (solved) {
        cout << "Found Solution with status: " << solved.asString() << endl;
        //ss.getSolutionPath().print(cout);

        // Write solution to file
        ofstream out_file;
        out_file.open("plan.out");
        ss.getSolutionPath().printAsMatrix(out_file);
        out_file.close();
    }

    return 0;
}

