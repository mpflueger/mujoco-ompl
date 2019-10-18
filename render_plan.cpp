// Rollout and render a plan in MuJoCo

#include <iostream>
#include <fstream>
#include <string>

#include "glfw3.h"

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


int main(int argc, char** argv) {
    string xml_filename;
    string plan_filename;
    if (argc >= 2) {
        xml_filename = argv[1];
        plan_filename = argv[2];
    } else {
        cerr << "Format: render_plan <MuJoCo XML config> <plan file>" << endl;
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

    // Setup for rendering
    mjvCamera cam;
    mjvPerturb pert;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    // init GLFW, create window, make OpenGL context current, request v-sync
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(600, 500, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultPerturb(&pert);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj->m, &scn, 1000); // space for 1000 objects
    mjr_makeContext(mj->m, &con, mjFONTSCALE_100); // model-specific context

    // Read the plan file
    ifstream plan_file(plan_filename);
    auto plan = readPlan(plan_file);
    plan_file.close();

    auto si = MujocoStatePropagator::createSpaceInformation(mj->m);

    // Initialize the simulation state
    auto state = si->allocState();
    auto control = si->allocControl();
    double duration = 0;
    readOmplState(plan[0],
                  si.get(),
                  state->as<ob::CompoundState>(),
                  control->as<oc::RealVectorControlSpace::ControlType>(),
                  duration);
    MujocoStatePropagator::copyOmplStateToMujoco(
        state->as<ob::CompoundState>(), si.get(), mj->m, mj->d);

    // Start stepping and rendering
    for(size_t i=1; i < plan.size(); i++) {
        // realOmplState(plan[i], si, state, control, duration);
        readOmplState(plan[i],
                      si.get(),
                      state->as<ob::CompoundState>(),
                      control->as<oc::RealVectorControlSpace::ControlType>(),
                      duration);
        MujocoStatePropagator::copyOmplControlToMujoco(
            control->as<oc::RealVectorControlSpace::ControlType>(),
            si.get(), mj->m, mj->d);

        // Find the number of timesteps for this control setting
        int steps = ceil(duration / mj->getMaxTimestep());
        mj->m->opt.timestep = duration / steps;
        for(int i=0; i < steps; i++) {
            if (glfwWindowShouldClose(window)) break;

            // Step
            mj->step();

            // Render
            // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(mj->m, mj->d, &opt, &pert, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }

        if (glfwWindowShouldClose(window)) break;
    }

    // close GLFW, free visualization storage
    //glfwTerminate(); // apparently this crashes with Linux NVidia drivers
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    return 0;
}