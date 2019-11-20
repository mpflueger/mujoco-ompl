/// Rollout a plan in MuJoCo to save the trajectory with a finer and constant
/// time resolution.

#include <iostream>
#include <fstream>
#include <string>

#include <cxxopts.hpp>
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
    // Parse args with cxxargs
    cxxopts::Options options(
        "smooth_plan",
        "Take a plan with irregular time steps and smooth it out into uniform"
        " short time steps by running it through the simulator");
    options.add_options()
        ("plan", "Plan file", cxxopts::value<string>())
        ("extra_positional", "", cxxopts::value<vector<string> >())
        ("m,mjxml", "MuJoCo XML config file", cxxopts::value<string>())
        ("o,output", "Output file name", cxxopts::value<string>())
        ("j,json", "Output in JSON format")
        //("c,csv", "Output in CSV format")
        //("raw", "Output in raw format (space seperated values)")
        ;
    options.parse_positional({"plan", "extra_positional"});
    options.positional_help("<Plan file>");

    string plan_fn = "";
    string mjxml_fn = "";
    string output_fn = "";
    bool output_json = false;
    //bool output_csv = false;
    bool help = false;
    try {
        auto result = options.parse(argc, argv);

        if (result.count("plan")) {
            plan_fn = result["plan"].as<string>();
        } else {
            help = true;
        }

        if (result.count("mjxml")) {
            mjxml_fn = result["mjxml"].as<string>();
        }

        if (result.count("output")) {
            output_fn = result["output"].as<string>();
        }

        if (result.count("extra_positional")) {
            cerr << "Unknown positional argument" << endl;
            help = true;
        }

        if (result.count("json")) {
            output_json = result["json"].as<bool>();
        }

        // if (result.count("csv")) {
        //     output_csv = result["csv"].as<bool>();
        // }

        if (help) {
            cerr << options.help();
            return -1;
        }
    } catch(cxxopts::OptionException e) {
        cerr << e.what() << endl;
        cerr << options.help();
        return -1;
    }

    // TODO: use regex
    if (output_json) {
        if (output_fn.find(".json") == string::npos) {
            cerr << "Please output JSON to a .json file" << endl;
            return -1;
        }
    }
    // if (output_csv) {
    //     if (output_fn.find(".csv") == string::npos) {
    //         cerr << "Please output CSV to a .csv file" << endl;
    //         return -1;
    //     }
    // }

    // Create MuJoCo Object
    string mjkey_filename = strcat(getenv("HOME"), "/.mujoco/mjkey.txt");
    auto mj(make_shared<MuJoCo>(mjkey_filename));

    // Check xml filename
    if (mjxml_fn.find(".xml") == string::npos) {
        cerr << "MuJoCo XML model file is required" << endl;
        return -1;
    }

    // Load Model
    cerr << "Loading MuJoCo config from: " << mjxml_fn << endl;
    if (!mj->loadXML(mjxml_fn)) {
        cerr << "Could not load XML model file" << endl;
        return -1;
    }

    // Make data
    if (!mj->makeData()) {
        cerr << "Could not allocate mjData" << endl;
        return -1;
    }

    // Read the plan file
    ifstream plan_file(plan_fn);
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

    if (output_json) {
        // write data in JSON format
        // sample:
        // {
        //   "state_dims": [
        //     joint1,
        //     joint1vel,
        //     control1,
        //     time
        //   ]
        //   "path": [
        //     [0, 0, 0, 0],
        //     [0, 0, 0, 1],
        //   ]
        // }
        //
        ofstream out_file(output_fn);
        out_file << "{\n"
                 << "  \"state_dims\": [\n";
        // write joint names
        for(auto const& i : getJointInfo(mj->m)) {
            out_file << "    \"" << i.name << "\",\n";
        }
        if (mj->m->nq != mj->m->nv) {
            cerr << "ERROR: nq != nv" << endl;
        }
        // names for joint velocity
        for(auto const& i : getJointInfo(mj->m)) {
            out_file << "    \"" << i.name << "vel\",\n";
        }
        // control dimensions
        for(int i=1; i < mj->m->nu + 1; i++) {

            out_file << "    \"control" << i << "\",\n";
        }
        out_file << "    \"time\"\n"
                 << "  ],\n"
                 << "  \"path\": [\n";

        // write path data
        for(auto const& i : smooth_plan) {
            // write data from step i
            out_file << "    [";
            if (i.size() > 0) {
                out_file << i[0];
            }
            for(size_t j=1; j < i.size(); j++) {
                out_file << ", " << i[j];
            }
            out_file << "],\n";
        }

        out_file << "  ]\n"
                 << "}";
        out_file.close();
    }
    // else if (output_csv) {
    //     cerr << "ERROR: CSV output is not implemented yet" << endl;
    //     ofstream out_file(output_fn);
    //     // Write headers
    //     // joint names
    //     for(auto const& i : getJointInfo(mj->m)) {
    //         out_file << i.name << ", ";
    //     }
    //     // joint velocities
    //     // control dims
    //     // time
    // }
    else {
        // Write smoothed plan in space-separated format
        ofstream out_file(output_fn);
        for(auto const& i : smooth_plan) {
            if (i.size() > 0) {
                out_file << i[0];
            }
            for(size_t j=1; j < i.size(); j++) {
                out_file << " " << i[j];
            }
            out_file << endl;
        }
    }

    return 0;
}
