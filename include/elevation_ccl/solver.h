#pragma once
#include <ssb_ccl.h>

namespace ccl
{
class Region 
{
public:
    Region(int label, const Matrix& s);
    void addCell(const Vector& s);
    void addRegion(const Region& r);

    int label_;
    int component_num_;
    Vector mean_;
};

class Solver : public SSbCCL
{
public:
    // check the connectivity according to the states. 
    bool canConnect(const int& row, const int& col, const int& label) const override;

    // process connection between a region including cell(i,j) and another one labeled by label.
    // update state of the region 
    void connect(const int& row, const int& col, const int& label) override;
    
    // process initialize the state of the new region (new label).
    void newRegion(const int& row, const int& col) override;

    void setDistanceThres(const float thres) {distance_threshold_ = thres;}
    const Region& getRegion(const int label) const {return regions_[label-1];}
    Region& getRegionRef(const int label) {return regions_[label-1];}
private:
    std::vector<Region> regions_;
    float distance_threshold_;
};
}