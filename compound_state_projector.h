/// Provide a state projection for compound states.
/// This only uses the elements of the state that are expressed as a vector
/// of doubles.

#pragma once

#include <iostream>
#include <vector>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateProjections.h>

// class CompoundStateProjector : public ompl::base::ProjectionEvaluator {
class CompoundStateProjector
        : public ompl::base::RealVectorRandomLinearProjectionEvaluator {
  public:
    // CompoundStateProjector(const ompl::base::CompoundStateSpace* space)
    //         : ompl::base::ProjectionEvaluator(space),
    //         dim(3) {
    CompoundStateProjector(const ompl::base::CompoundStateSpace* space,
        std::shared_ptr<ompl::base::RealVectorStateSpace> real_space_,
        int dim_)
            : RealVectorRandomLinearProjectionEvaluator(real_space_, dim_),
            real_space(real_space_),
            dim(dim_) {
        // Design a real space and allocate projection
        // real_space = std::make_shared<ompl::base::RealVectorStateSpace>(
        //     space->getDimension());
        // real_space->setBounds(-3.14, 3.14);
        // std::vector<double> cell_sizes(dim, 0.1);
        // rand_proj = std::make_shared<
        //     ompl::base::RealVectorRandomLinearProjectionEvaluator>(
        //         real_space, dim);

        // Copy protected variables?
        // *space_ = *(rand_proj->space_);
        // cellSizes_ = rand_proj->cellSizes_;
        // bounds_ = rand_proj->bounds_;
        // estimatedBounds_ = rand_proj->estimatedBounds_;
        // defaultCellSizes_ = rand_proj->defaultCellSizes_;
        // cellSizesWereInferred_ = rand_proj->cellSizesWereInferred_;
        // params_ = rand_proj->params_;
    }

    CompoundStateProjector(const ompl::base::CompoundStateSpace* space,
        std::shared_ptr<ompl::base::RealVectorStateSpace> real_space_,
        std::vector<double> cell_sizes)
            : RealVectorRandomLinearProjectionEvaluator(real_space_, cell_sizes),
            real_space(real_space_),
            dim(cell_sizes.size()) {}

    static std::shared_ptr<CompoundStateProjector> makeCompoundStateProjector(
        const ompl::base::CompoundStateSpace* space)
    {
        auto real_space = std::make_shared<ompl::base::RealVectorStateSpace>(
            space->getDimension());
        // real_space->setBounds(-3.14, 3.14);
        // real_space->setBounds(-1, 1);
        int dim = 3;
        std::vector<double> cell_sizes(dim, 0.1);
        auto csp(std::make_shared<CompoundStateProjector>(
            space, real_space, cell_sizes));
        return csp;
    }

    unsigned int getDimension() const override {
        return dim;
    }

    void project(
        const ompl::base::State* state,
        Eigen::Ref< Eigen::VectorXd > projection) const override
    {
        // Create a real vector state
        // std::vector<double> reals;
        // real_space->copyToReals(reals, state);
        // auto rv_state = real_space->allocState();
        // for(size_t i=0; i < reals.size(); i++) {
        //     rv_state->as<ompl::base::RealVectorStateSpace::StateType>()
        //         ->values[i] = reals[i];
        // }

        auto rv_state = getRealVectorState(state);

        // Use a real space projection

        // *(rand_proj->space_) = *space_;
        // rand_proj->cellSizes_ = cellSizes_;
        // rand_proj->bounds_ = bounds_;
        // rand_proj->estimatedBounds_ = estimatedBounds_;
        // rand_proj->defaultCellSizes_ = defaultCellSizes_;
        // rand_proj->cellSizesWereInferred_ = cellSizesWereInferred_;
        // rand_proj->params_ = params_;

        //rand_proj->project(rv_state, projection);
        // ((ompl::base::RealVectorRandomLinearProjectionEvaluator*) this)
        //     ->project(rv_state, projection);
        ompl::base::RealVectorRandomLinearProjectionEvaluator::project(
            rv_state, projection);

        // Cleanup
        real_space->freeState(rv_state);
    }

    // void setCellSizes(const std::vector<double>& cellSizes) override {
    //     rand_proj->setCellSizes(cellSizes);
    // }

    // void setCellSizes(unsigned int dim, double cellSize) override {
    //     rand_proj->setCellSizes(dim, cellSize);
    // }

    // void mulCellSizes(double factor) override {
    //     rand_proj->mulCellSizes(factor);
    // }

    // bool userConfigured() const {
    //     return rand_proj->userConfigured();
    // }

    // const std::vector<double>& getCellSizes() const override {
    //     return rand_proj->getCellSizes();
    // }

    // double getCellSizes(unsigned int dim) const override {
    //     return rand_proj->getCellSizes();
    // }

    // void checkCellSizes() const override {
    //     rand_proj->checkCellSizes();
    // }

    // void inferCellSizes() override {
    //     rand_proj->inferCellSizes();
    // }

    // void defaultCellSizes() override {
    //     rand_proj->defaultCellSizes();
    // }

    // void checkBounds() const override {
    //     rand_proj->checkBounds();
    // }

    // bool hasBounds() const override {
    //     return rand_proj->hasBounds();
    // }

    // void setBounds(const ompl::base::RealVectorBounds& bounds) override {
    //     rand_proj->setBounds(bounds);
    // }

    // const ompl::base::RealVectorBounds& getBounds() const override {
    //     return rand_proj->getBounds();
    // }

    // void inferBounds() override {
    //     rand_proj->inferBounds();
    // }

    // void setup() override {
    //     rand_proj->setup();
    // }

    // void computeCoordinates(
    //     const Eigen::Ref<Eigen::VectorXd>& projection,
    //     Eigen::Ref<Eigen::VectorXi> coord) const override
    // {
    //     rand_proj->computeCoordinates(projection, coord);
    // }

    // void computeCoordinates(
    //     const ompl::base::State* state,
    //     Eigen::Ref<Eigen::VectorXi> coord) const override
    // {
    //     auto rv_state = getRealVectorState(state)
    //     rand_proj->computeCoordinates(rv_state, coord);

    //     // Cleanup
    //     real_space->freeState(rv_state);
    // }

    // ompl::base::ParamSet& params() override {
    //     return rand_proj->params();
    // }

    // const ompl::base::ParamSet& params() const override {
    //     return rand_proj->params();
    // }

    // void printSettings(std::ostream& out=std::cout) const override {
    //     rand_proj->printSettings(out);
    // }

    // void printProjection(
    //     const Eigen::Ref<Eigen::VectorXd>& projection,
    //     std::ostream& out=std::cout) const override
    // {
    //     rand_proj->printProjection(projection, out);
    // }

  protected:
    // void estimateBounds() override {
    //     rand_proj->estimateBounds();
    // }

  private:
    ompl::base::State* getRealVectorState(const ompl::base::State* state) const {
        // Create a real vector state
        std::vector<double> reals;
        real_space->copyToReals(reals, state);
        auto rv_state = real_space->allocState();
        for(size_t i=0; i < reals.size(); i++) {
            rv_state->as<ompl::base::RealVectorStateSpace::StateType>()
                ->values[i] = reals[i];
        }

        return rv_state;
    }

    // std::shared_ptr<ompl::base::RealVectorRandomLinearProjectionEvaluator>
    //     rand_proj;
    std::shared_ptr<ompl::base::RealVectorStateSpace> real_space;
    const int dim;
};
