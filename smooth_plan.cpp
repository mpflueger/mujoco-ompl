/// Rollout a plan in MuJoCo to save the trajectory with a finer and constant
/// time resolution.

#include <iostream>
#include <fstream>
#include <string>

#include "mujoco_wrapper.h"
#include "mujoco_ompl_interface.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;

/// This reads a plan of the type produced by OMPL's printAsMatrix function.
/// Coordinates are space separated doubles, no extra lines, no comment marks.
vector<vector<double> > readPlan(istream& file) {
    vector<vector<double> > matrix;

    string line;
    while(getline(file, line)) {
        vector<double> row;
        stringstream ls(line);
        while(true) {
            double x;
            ls >> x;
            row.push_back(x);

            if (!ls.good()) break;
        }
        matrix.push_back(row);
    }

    return matrix;
}

vector<double>& operator<<(vector<double>& v, const MuJoCoState& s) {
    for(auto const& i : s.qpos) {
        v.push_back(i);
    }
    for(auto const& i : s.qvel) {
        v.push_back(i);
    }
    for(auto const& i : s.ctrl) {
        v.push_back(i);
    }
    v.push_back(s.time);

    return v;
}


int main(int argc, char** argv) {
    string xml_filename;
    string plan_filename;
    if (argc >= 2) {
        xml_filename = argv[1];
        plan_filename = argv[2];
    } else {
        cerr << "Format: smooth_plan <MuJoCo XML config> <plan file>" << endl;
        return -1;
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
    cerr << "Loading MuJoCo config from: " << xml_filename << endl;
    if (!mj->loadXML(xml_filename)) {
        cerr << "Could not load XML model file" << endl;
        return -1;
    }

    // Make data
    if (!mj->makeData()) {
        cerr << "Could not allocate mjData" << endl;
        return -1;
    }

    // Read the plan file
    ifstream plan_file(plan_filename);
    auto plan = readPlan(plan_file);
    plan_file.close();

    auto si = MjOmpl::createSpaceInformation(mj->m);

    // Initialize the simulation state
    auto state = si->allocState();
    auto control = si->allocControl();
    double duration = 0;
    MjOmpl::readOmplState(plan[0],
                  si.get(),
                  state->as<ob::CompoundState>(),
                  control->as<oc::RealVectorControlSpace::ControlType>(),
                  duration);
    MjOmpl::copyOmplStateToMujoco(
        state->as<ob::CompoundState>(), si.get(), mj->m, mj->d);

    // Start stepping and rendering
    vector<vector<double> > smooth_plan;
    for(size_t i=1; i < plan.size(); i++) {
        // realOmplState(plan[i], si, state, control, duration);
        MjOmpl::readOmplState(plan[i],
                      si.get(),
                      state->as<ob::CompoundState>(),
                      control->as<oc::RealVectorControlSpace::ControlType>(),
                      duration);
        MjOmpl::copyOmplControlToMujoco(
            control->as<oc::RealVectorControlSpace::ControlType>(),
            si.get(), mj->m, mj->d);

        // Find the number of timesteps for this control setting
        int steps = ceil(duration / mj->getMaxTimestep());
        mj->m->opt.timestep = duration / steps;
        for(int i=0; i < steps; i++) {
            // Save step
            vector<double> state_vec;
            state_vec << mj->getState();
            smooth_plan.push_back(state_vec);

            // Step
            mj->step();
        }
    }

    vector<double> state_vec;
    state_vec << mj->getState();
    smooth_plan.push_back(state_vec);

    // Write smoothed plan to cout
    for(auto const& i : smooth_plan) {
        if (i.size() > 0) {
            cout << i[0];
        }
        for(size_t j=1; j < i.size(); j++) {
            cout << " " << i[j];
        }
        cout << endl;
    }

    return 0;
}