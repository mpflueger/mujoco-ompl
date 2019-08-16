// Plan for a MuJoCo environment with OMPL

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/sst/SST.h>

#include "mujoco_wrapper.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;


int main(int argc, char** argv) {
    if (argc >= 1) {
        string xml_filename = argv[1];
    } else {
        cerr << "Please provide a MuJoCo config." << endl;
        return -1;
    }

    // Create MuJoCo Object
    string mjkey_filename = strcat(getenv("HOME"), "/.mujoco/mjkey.txt");
    auto mj(make_shared<MuJoCo>(mjkey_filename));

    // Get xml file name
    // TODO: make this more modern (argparsing, regex)
    //   could use boost.program_options
    string xml_filename = argv[1];
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

    // Create a SimpleSetup object with custom planner
    auto sst_planner(make_shared<oc::SST>(si));
    oc::SimpleSetup ss(si);
    ss.setPlanner(sst_planner);

    // TODO: Set start and goal states
    /*******************
    .
    .
    .
    ****************/

    // Call the planner
    //ob::PlannerStatus solved = ss.solve(1.0);
    bool solved = false;

    if (solved) {
        cout << "Found Solution!" << endl;
        //ss.getSolutionPath().print(cout);

        // Write solution to file
        ofstream out_file;
        out_file.open("plan.out");
        ss.getSolutionPath().printAsMatrix(out_file);
        out_file.close();
    }

    return 0;
}

