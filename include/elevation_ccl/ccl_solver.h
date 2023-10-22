#pragma once
#include <ssb_ccl.h>

namespace elevation_ccl
{
class Region 
{
public:
    Region(int label, const ccl::Vector& s);
    void addCell(const ccl::Vector& s);
    void addRegion(Region& r);

    int label_;
    int component_num_;
    ccl::Vector mean_;
    ccl::Matrix variance_;
    bool is_root_;
};

class CclSolver : public ccl::SSbCCL
{
public:
    bool initialize(const ccl::StateMatrix& state, const ccl::ScoreMatrix& score, ccl::LabelMatrix& labels) override;
    bool isValid(const int& row, const int& col) const override;
    
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
    int getRegionNum() const {return region_num_;}
    const std::vector<Region>& getRegions() const {return regions_;}
private:
    std::vector<Region> regions_;
    int region_num_ = 0;
    float distance_threshold_;
};
}