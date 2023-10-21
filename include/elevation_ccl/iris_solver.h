#pragma once

#include <iris.h>
#include <plane_segmentation/ccl_solver.h>

namespace plane_segmentation
{
class IrisSolver
{
public:
    IrisSolver();
    ~IrisSolver();

    bool initialize();
    bool solve();
    const Eigen::MatrixXd& getA() const {return region_.getPolyhedron().getA();}
    const Eigen::VectorXd& getB() const {return region_.getPolyhedron().getB();}
    const Eigen::MatrixXd& getC() const {return region_.getEllipsoid().getC();}
    const Eigen::VectorXd& getD() const {return region_.getEllipsoid().getD();}

private:
    iris::IRISProblem problem_;
    iris::IRISOptions options_;
    iris::IRISDebugData debug_;
    iris::IRISRegion region_;
};
}