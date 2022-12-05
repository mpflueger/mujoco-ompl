// Rollout and render a plan in MuJoCo

#include <iostream>
#include <fstream>
#include <string>

#include <GLFW/glfw3.h>

#include "mujoco_wrapper.h"
#include "mujoco_ompl_interface.h"
#include <chrono>

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;

MuJoCo *mj = NULL;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(mj->m, mj->d);
        mj_forward(mj->m, mj->d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(mj->m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj->m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


/// This reads a plan of the type produced by OMPL's printAsMatrix function.
/// Coordinates are space separated doubles, no extra lines, no comment marks.
vector<vector<double> > readPlan(istream& file, int dim) {
    vector<vector<double> > matrix;

    string line;
    while(getline(file, line)) {
        vector<double> row;
        stringstream ls(line);
        for (int i=0; i< dim; i++) {
            double x;
            ls >> x;
            row.push_back(x);
        }
        matrix.push_back(row);
    }

    return matrix;
}

// make default abstract geom
void v_defaultGeom(mjvGeom* geom)
{
    geom->type = mjGEOM_NONE;
    geom->dataid = -1;
    geom->objtype = mjOBJ_UNKNOWN;
    geom->objid = -1;
    geom->category = mjCAT_DECOR;
    geom->texid = -1;
    geom->texuniform = 0;
    geom->texrepeat[0] = 1;
    geom->texrepeat[1] = 1;
    geom->emission = 0;
    geom->specular = 0.5;
    geom->shininess = 0.5;
    geom->reflectance = 0;
    geom->label[0] = 0;
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
    mj = new MuJoCo(mjkey_filename);

    // Get xml file name
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
    mjvPerturb pert;

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

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    auto si = MjOmpl::createSpaceInformationKinematic(mj->m);
    auto state = si->allocState();

    // Read the plan file
    ifstream plan_file(plan_filename);
    auto plan = readPlan(plan_file, si->getStateSpace()->getDimension());
    plan_file.close();

    // Initialize the simulation state
    int n_repeat = 2.0;
    double dt = 1e0;

    MjOmpl::readOmplStateKinematic(plan[0],
                          si.get(),
                          state->as<ob::CompoundState>());
    MjOmpl::copyOmplStateToMujoco(state->as<ob::CompoundState>(), si.get(), mj->m, mj->d, false);


    chrono::system_clock::time_point tm_start = chrono::system_clock::now();
    // Start stepping and rendering
    size_t i=0;
    while (i < plan.size()*n_repeat)
    {
        size_t idx = i % plan.size();

        // sleep for dt seconds to make animation a bit slower

        if (chrono::duration<double>(chrono::system_clock::now() - tm_start).count() > dt) {
            tm_start = chrono::system_clock::now();
            MjOmpl::readOmplStateKinematic(plan[idx],
                                           si.get(),
                                           state->as<ob::CompoundState>());
            MjOmpl::copyOmplStateToMujoco(state->as<ob::CompoundState>(), si.get(), mj->m, mj->d, false);
            mj_fwdPosition(mj->m, mj->d);
            i++;
        }

        // Render
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(mj->m, mj->d, &opt, &pert, &cam, mjCAT_ALL, &scn);

        for(size_t j=0; j < idx; j++) {
            mjvGeom *g = scn.geoms + scn.ngeom++;
            g->type = mjGEOM_SPHERE;
            g->dataid = -1;
            g->objtype = mjOBJ_UNKNOWN;
            g->objid = -1;
            g->category = mjCAT_DECOR;
            g->texid = -1;
            g->texuniform = 0;
            g->texrepeat[0] = 1;
            g->texrepeat[1] = 1;
            g->emission = 0;
            g->specular = 0.5;
            g->shininess = 0.5;
            g->reflectance = 0;
            g->label[0] = 0;
            g->size[0] = 0.01f;
            g->size[1] = 0.01f;
            g->size[2] = 0.01f;
            g->rgba[0] = 0.0f;
            g->rgba[1] = 0.5f;
            g->rgba[2] = 0.5f;
            g->rgba[3] = 0.8f;
            g->pos[0] = plan[j][1];
            g->pos[1] = 0.;
            g->pos[2] = plan[j][0];
            mjtNum mat[9];
            mjtNum randomQuat[4] = {1, 0, 0, 0};
            mju_quat2Mat(mat, randomQuat);
            mju_n2f(g->mat, mat, 9);
        }

        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // close GLFW, free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
    return 0;
}
