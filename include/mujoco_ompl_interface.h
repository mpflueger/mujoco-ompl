/// Provide linkages between MuJoCo and OMPL

#pragma once

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "mujoco_wrapper.h"

namespace MjOmpl {

/// Read a vectorized OMPL planning state back into data structures
/// Note: state and control must be pre-allocated from the space info
void readOmplState(
    const std::vector<double>& x,
    const ompl::control::SpaceInformation* si,
    ompl::base::CompoundState* state,
    ompl::control::RealVectorControlSpace::ControlType* control,
    double& duration);

std::shared_ptr<ompl::control::SpaceInformation>
createSpaceInformation(const mjModel* m);

std::shared_ptr<ompl::base::SpaceInformation>
createSpaceInformationKinomatic(const mjModel* m);

void copyOmplStateToMujoco(
    const ompl::base::CompoundState* state,
    const ompl::control::SpaceInformation* si,
    const mjModel* m,
    mjData* d);

void copyMujocoStateToOmpl(
    const mjModel* m,
    const mjData* d,
    const ompl::control::SpaceInformation* si,
    ompl::base::CompoundState* state);

void copyOmplControlToMujoco(
    const ompl::control::RealVectorControlSpace::ControlType* control,
    const ompl::control::SpaceInformation* si,
    const mjModel* m,
    mjData* d);

/// Copy SO3State to double array with no bounds checks
void copySO3State(
    const ompl::base::SO3StateSpace::StateType* state,
    double* data);

/// Copy double array to SO3 state with no bounds checks
void copySO3State(
    const double* data,
    ompl::base::SO3StateSpace::StateType* state);

/// Copy SE3State to double array with no bounds checks
void copySE3State(
    const ompl::base::SE3StateSpace::StateType* state,
    double* data);

/// Copy double array to SE3 state with no bounds checks
void copySE3State(
    const double* data,
    ompl::base::SE3StateSpace::StateType* state);


class MujocoStatePropagator : public ompl::control::StatePropagator {
  public:
    MujocoStatePropagator(
            std::shared_ptr<ompl::control::SpaceInformation> si,
            std::shared_ptr<MuJoCo> mj)
            : StatePropagator(si),
              mj(mj) {
    } 

    const ompl::control::SpaceInformation* getSpaceInformation() const {
        return si_;
    }

    // To override this function from oc::StatePropagator, this has to be a
    // const function, but we need to modify the mjModel and mjData objects
    // to use MuJoCo to propagate a state
    // Use a preallocatd object, protect with mutex lock in case OMPL does
    // threading
    void propagate(
        const ompl::base::State* state,
        const ompl::control::Control* control,
        double duration,
        ompl::base::State* result) const override;

    bool canPropagateBackward() const override {
        return false;
    }

    bool canSteer() const override {
        return false;
    }

  private:
    // These have to be mutable because the tyrrany of OMPL makes
    // propagate a const function and I don't want to reallocate them
    mutable std::shared_ptr<MuJoCo> mj;
    mutable std::mutex mj_lock;
};

} // MjOmpl namespace
