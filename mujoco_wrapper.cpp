#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "mujoco_wrapper.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;


//////////////////////////////
// Init static variables
//////////////////////////////

int MuJoCo::mj_instance_count = 0;
mutex MuJoCo::mj_instance_count_lock;

//////////////////////////////
// Define functions
//////////////////////////////

std::ostream& operator<<(std::ostream& os, const JointInfo& ji) {
    os << "Joint( name: \"" << ji.name << "\", "
       << "type: " << ji.type << ", "
       << "limited: " << ji.limited << ", "
       << "range: (" << ji.range[0] << ", " << ji.range[1] << ") "
       << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const MuJoCoState& s) {
    os << "{time: " << s.time << ", "
       << "qpos: [";
    for(auto const& i : s.qpos) {
        os << i << ", ";
    }
    os << "] qvel: [";
    for(auto const& i : s.qvel) {
        os << i << ", ";
    }
    os << "] act: [";
    for(auto const& i : s.act) {
        os << i << ", ";
    }
    os << "]}";
    return os;
}

std::vector<JointInfo> getJointInfo(const mjModel* m) {
    std::vector<JointInfo> joints;
    for (size_t i=0; i < m->njnt; i++) {
        JointInfo joint;
        joint.name = std::string(m->names + m->name_jntadr[i]);
        joint.type = m->jnt_type[i];
        joint.limited = (bool) m->jnt_limited[i];
        joint.range[0] = m->jnt_range[2*i];
        joint.range[1] = m->jnt_range[2*i + 1];
        joint.qposadr = m->jnt_qposadr[i];
        joint.dofadr = m->jnt_dofadr[i];
        joints.push_back(joint);
    }
    return joints;
}

StateRange getCtrlRange(const mjModel* m, size_t i) {
    StateRange r;
    r.limited = (bool) m->actuator_ctrllimited[i];
    r.range[0] = m->actuator_ctrlrange[2*i];
    r.range[1] = m->actuator_ctrlrange[2*i + 1];
    return r;
}

shared_ptr<oc::SpaceInformation>
MuJoCoStatePropagator::createSpaceInformation(const mjModel* m) {
    int control_dim = m->nu;

    //////////////////////////////////////////////
    // Create the state space (including velocity)

    // Iterate over joints
    auto joints = getJointInfo(m);
    auto space(make_shared<ob::CompoundStateSpace>());
    // Add a subspace matching the topology of each joint
    vector<shared_ptr<ob::StateSpace> > vel_spaces;
    int next_qpos = 0;
    for(const auto& joint : joints) {
        ob::RealVectorBounds bounds(1);
        bounds.setLow(joint.range[0]);
        bounds.setHigh(joint.range[1]);

        // Check that our assumptions are ok
        if (joint.qposadr != next_qpos) {
            cerr << "Uh oh......" << endl;
            throw invalid_argument(
                "Joints are not in order of qposadr ... write more code!");
        }
        next_qpos++;

        // Crate an appropriate subspace
        shared_ptr<ob::StateSpace> joint_space;
        switch(joint.type) {
          case mjJNT_FREE:
            joint_space = make_shared<ob::SE3StateSpace>();
            vel_spaces.push_back(make_shared<ob::RealVectorStateSpace>(6));
            next_qpos += 6;
            cerr << "Error: FREE joints are not yet supported!" << endl;
            throw invalid_argument(
                "FREE joints are not yet supported.");
            break;

          case mjJNT_BALL:
            // MuJoCo quaterions take 4d in pos space and 3d in vel space
            joint_space = make_shared<ob::SO3StateSpace>();
            if (joint.limited) {
                cerr << "ERROR: OMPL bounds on SO3 spaces are not implemented!"
                     << endl;
            }
            vel_spaces.push_back(make_shared<ob::RealVectorStateSpace>(3));
            next_qpos += 3;
            cerr << "Error: BALL joints are not yet supported!" << endl;
            throw invalid_argument(
                "BALL joints are not yet supported.");
            break;

          case mjJNT_HINGE:
            if (joint.limited) {
                // A hinge with limits is R^1
                joint_space = make_shared<ob::RealVectorStateSpace>(1);
                static_pointer_cast<ob::RealVectorStateSpace>(joint_space)
                    ->setBounds(bounds);
            } else {
                // A hinge with continuous rotation needs to be treated as
                // SO2 so OMPL knows that it rotates back to the original
                // position
                joint_space = make_shared<ob::SO2StateSpace>();
            }
            vel_spaces.push_back(make_shared<ob::RealVectorStateSpace>(1));
            break;

          case mjJNT_SLIDE:
            joint_space = make_shared<ob::RealVectorStateSpace>(1);
            if (joint.limited) {
                static_pointer_cast<ob::RealVectorStateSpace>(joint_space)
                    ->setBounds(bounds);
            }
            vel_spaces.push_back(make_shared<ob::RealVectorStateSpace>(1));
            break;

          default:
            cerr << "ERROR: Unknown joint type!" << endl;
            throw invalid_argument("Unknown joint type");
            break;
        }
        space->addSubspace(joint_space, 1.0);
    } 
    if (next_qpos != m->nq) {
        cerr << "ERROR: joint dims: " << next_qpos
             << " vs nq: " << m->nq << endl;
        throw invalid_argument("Total joint dimensions are not equal to nq");
    }

    // Add on all the velocity spaces
    for(const auto& s : vel_spaces) {
        space->addSubspace(s, 1.0);
    }
    space->lock();  // We are done

    ////////////////////////////////
    // Create the control space
    auto c_space(make_shared<oc::RealVectorControlSpace>(space, control_dim));
    // Set bounds
    ob::RealVectorBounds c_bounds(control_dim);
    c_bounds.setLow(-1);
    c_bounds.setHigh(1);
    // Handle specific bounds
    for(size_t i=0; i < control_dim; i++) {
        auto range = getCtrlRange(m, i);
        if (range.limited) {
            c_bounds.setLow(i, range.range[0]);
            c_bounds.setHigh(i, range.range[1]);
        }
    }
    c_space->setBounds(c_bounds);

    //////////////////////////////////////////
    // Combine into the SpaceInformation
    auto si(make_shared<oc::SpaceInformation>(space, c_space));
    return si;
}

void MuJoCoStatePropagator::propagate( const ob::State* state,
                                       const oc::Control* control,
                                       double duration,
                                       ob::State* result) const {
    mj_lock.lock();
    // Copy state and control to mj->d
    // Set duration on mj->m
    // mj::step()
    // Copy result to ob::State*
    mj_lock.unlock();
}
