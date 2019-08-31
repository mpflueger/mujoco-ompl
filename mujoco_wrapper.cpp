#include <ompl/base/StateSpace.h>

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
MujocoStatePropagator::createSpaceInformation(const mjModel* m) {
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
        // Apparently OMPL needs bounds
        // 350 m/s is just a bit supersonic
        // 50 m/s is pretty fast for most robot parts
        s->as<ob::RealVectorStateSpace>()->setBounds(-50, 50);
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
    si->setPropagationStepSize(m->opt.timestep);
    return si;
}


void MujocoStatePropagator::copyOmplStateToMujoco(
        const ob::CompoundState* state,
        const oc::SpaceInformation* si,
        const mjModel* m,
        mjData* d) {
    // Iterate over subspaces to copy data from state to mjData
    // Copy position state to d->qpos
    // Copy velocity state to d->qvel
    assert(si->getStateSpace()->isCompound());
    auto css(si->getStateSpace()->as<ob::CompoundStateSpace>());
    int qpos_i = 0;
    int qvel_i = 0;
    for(size_t i=0; i < css->getSubspaceCount(); i++) {
        auto subspace(css->getSubspace(i));
        const ob::State* substate = (*state)[i];

        // Choose appropriate copy code based on subspace type
        size_t n;
        switch (subspace->getType()) {
          case ob::STATE_SPACE_REAL_VECTOR:
            n = subspace->as<ob::RealVectorStateSpace>()->getDimension();

            // Check if the vector does not align on the size of qpos
            // If this happens an assumption has been violated
            if (qpos_i < m->nq && (qpos_i + n) > m->nq) {
                throw invalid_argument(
                    "RealVectorState does not align on qpos");
            }

            // Copy vector
            for(size_t i=0; i < n; i++) {
                // Check if we copy to qpos or qvel
                if (qpos_i < m->nq) {
                    d->qpos[qpos_i] = substate
                        ->as<ob::RealVectorStateSpace::StateType>()->values[i];
                    qpos_i++;
                } else {
                    d->qvel[qvel_i] = substate
                        ->as<ob::RealVectorStateSpace::StateType>()->values[i];
                    qvel_i++;
                }
            }
            break;

          case ob::STATE_SPACE_SO2:
            if (qpos_i >= m->nq) {
                throw invalid_argument(
                    "SO2 velocity state should not happen.");
            }

            d->qpos[qpos_i]
                = substate->as<ob::SO2StateSpace::StateType>()->value;
            qpos_i++;
            break;

          case ob::STATE_SPACE_SO3:
            if (qpos_i + 4 > m->nq) {
                throw invalid_argument("SO3 space overflows qpos");
            }

            copySO3State(
                substate->as<ob::SO3StateSpace::StateType>(),
                d->qpos + qpos_i);
            // That's right MuJoCo, ponter math is what you get when
            // you write an API like that :P
            qpos_i += 4;
            break;

          case ob::STATE_SPACE_SE3:
            if (qpos_i + 7 > m->nq) {
                throw invalid_argument("SE3 space overflows qpos");
            }

            copySE3State(
                substate->as<ob::SE3StateSpace::StateType>(),
                d->qpos + qpos_i);
            qpos_i += 7;
            break;

          default:
            throw invalid_argument("Unhandled subspace type.");
            break;
        }
    }

    if (!(qpos_i == m->nq && qvel_i == m->nv)) {
        throw invalid_argument(
            "Size of data copied did not match m->nq and m->nv");
    }
}


void MujocoStatePropagator::copyMujocoStateToOmpl(
        const mjModel* m,
        const mjData* d,
        const oc::SpaceInformation* si,
        ob::CompoundState* state) {
    // Iterate over subspaces and copy data from mjData to CompoundState
    assert(si->getStateSpace()->isCompound());
    auto css(si->getStateSpace()->as<ob::CompoundStateSpace>());
    int qpos_i = 0;
    int qvel_i = 0;
    for(size_t i=0; i < css->getSubspaceCount(); i++) {
        auto subspace(css->getSubspace(i));

        // Choose appropriate copy code based on subspace type
        size_t n;
        switch (subspace->getType()) {
          case ob::STATE_SPACE_REAL_VECTOR:
            n = subspace->as<ob::RealVectorStateSpace>()->getDimension();

            // Check if the vector does not align on the size of qpos
            // If this happens an assumption has been violated
            if (qpos_i < m->nq && (qpos_i + n) > m->nq) {
                throw invalid_argument(
                    "RealVectorState does not align on qpos");
            }

            // Copy vector
            for(size_t j=0; j < n; j++) {
                // Check if we copy to qpos or qvel
                if (qpos_i < m->nq) {
                    (*state)[i]->as<ob::RealVectorStateSpace::StateType>()
                        ->values[j] = d->qpos[qpos_i];
                    qpos_i++;
                } else {
                    (*state)[i]->as<ob::RealVectorStateSpace::StateType>()
                        ->values[j] = d->qvel[qvel_i];
                    qvel_i++;
                }
            }
            break;

          case ob::STATE_SPACE_SO2:
            if (qpos_i >= m->nq) {
                throw invalid_argument(
                    "SO2 velocity state should not happen.");
            }

            (*state)[i]->as<ob::SO2StateSpace::StateType>()
                ->value = d->qpos[qpos_i];
            qpos_i++;
            break;

          case ob::STATE_SPACE_SO3:
            if (qpos_i + 4 > m->nq) {
                throw invalid_argument("SO3 space overflows qpos");
            }

            copySO3State(
                d->qpos + qpos_i,
                (*state)[i]->as<ob::SO3StateSpace::StateType>());
            qpos_i += 4;
            break;

          case ob::STATE_SPACE_SE3:
            if (qpos_i + 7 > m->nq) {
                throw invalid_argument("SE3 space overflows qpos");
            }

            copySE3State(
                d->qpos + qpos_i,
                (*state)[i]->as<ob::SE3StateSpace::StateType>());
            qpos_i += 7;
            break;

          default:
            throw invalid_argument("Unhandled subspace type.");
            break;
        }
    }

    if (!(qpos_i == m->nq && qvel_i == m->nv)) {
        throw invalid_argument(
            "Size of data copied did not match m->nq and m->nv");
    }
}


void MujocoStatePropagator::copyOmplControlToMujoco(
        const oc::RealVectorControlSpace::ControlType* control,
        const oc::SpaceInformation* si,
        const mjModel* m,
        mjData* d) {
    int dim = si->getControlSpace()->as<oc::RealVectorControlSpace>()
        ->getDimension();
    if (dim != m->nu) {
        throw invalid_argument(
            "SpaceInformation and mjModel do not match in control dim");
    }

    for(size_t i=0; i < dim; i++) {
        d->ctrl[i] = control->values[i];
    }
}


void MujocoStatePropagator::copySO3State(
        const ob::SO3StateSpace::StateType* state,
        double* data) {
    data[0] = state->w;
    data[1] = state->x;
    data[2] = state->y;
    data[3] = state->z;
}


void MujocoStatePropagator::copySO3State(
        const double* data,
        ob::SO3StateSpace::StateType* state) {
    state->w = data[0];
    state->x = data[1];
    state->y = data[2];
    state->z = data[3];
}


void MujocoStatePropagator::copySE3State(
        const ob::SE3StateSpace::StateType* state,
        double* data) {
    data[0] = state->getX();
    data[1] = state->getY();
    data[2] = state->getZ();
    copySO3State(&state->rotation(), data + 3);
}


void MujocoStatePropagator::copySE3State(
        const double* data,
        ob::SE3StateSpace::StateType* state) {
    state->setX(data[0]);
    state->setY(data[1]);
    state->setZ(data[2]);
    copySO3State(data + 3, &state->rotation());
}


void MujocoStatePropagator::propagate( const ob::State* state,
                                       const oc::Control* control,
                                       double duration,
                                       ob::State* result) const {
    //cout << " -- propagate asked for a timestep of: " << duration << endl;

    mj_lock.lock();

    copyOmplStateToMujoco(state->as<ob::CompoundState>(), si_, mj->m, mj->d);
    copyOmplControlToMujoco(
        control->as<oc::RealVectorControlSpace::ControlType>(),
        si_,
        mj->m,
        mj->d);

    mj->sim_duration(duration);

    // Copy result to ob::State*
    copyMujocoStateToOmpl(mj->m, mj->d, si_, result->as<ob::CompoundState>());

    mj_lock.unlock();
}
