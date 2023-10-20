#include <elevation_ccl/solver.h>
#include <iostream>

namespace ccl
{

Region::Region(int label, const Matrix& s)
{
    label_ = label;
    component_num_ = 1;
    mean_ = s;
    variance_.resize(s.size(), s.size());
    variance_.setZero();
}

void Region::addCell(const Vector& s)
{
    mean_ = (mean_*component_num_ + s)/(component_num_);
    ++component_num_;
}

void Region::addRegion(const Region& r)
{
    mean_ = (mean_*component_num_ + r.mean_*r.component_num_)/(component_num_ + r.component_num_);
    component_num_ += r.component_num_;
}

/*
    Override functions of Solver
*/
bool Solver::isVaild(const int& row, const int& col) const
{
    for (const auto& map: *score_) 
    {
        if (map.first(row, col) == NAN) return false; 
        if (map.first(row, col) > map.second) return false;
    }
    return true;
}

bool Solver::canConnect(const int& row, const int& col, const int& label) const
{
    const int cell_label = labels_->coeff(row, col);
    // connect with a cell
    if (cell_label==0)
    {   
        // std::cout << "Check if region " << label << " can connect with a cell(" << row << "," << col << ")" << std::endl;
        // std::cout << "region mean: " << getRegion(label).mean_ << std::endl; 
        // std::cout << "state vector: " << state_->at(row)[col] << std::endl;
        const ccl::Vector& state = state_->at(row).at(col);
        if ((getRegion(label).mean_ - state).squaredNorm() < distance_threshold_) return true;
        else return false;
    }
    else // connect with a region 
    {
        // std::cout << "Check if region " << label << " can connect with region " << cell_label << std::endl;
        if ((getRegion(label).mean_ - getRegion(cell_label).mean_).squaredNorm() < distance_threshold_) return true;
        else return false;
    }
    
    return false;
}

void Solver::connect(const int& row, const int& col, const int& label)
{
    const int cell_label = labels_->coeff(row, col);
    // connect with a cell
    if (cell_label==0)
    {   
        const Vector& state = state_->at(row).at(col);
        getRegionRef(label).addCell(state);
    }
    else // connect with a region 
    {
        if (label < cell_label) getRegionRef(label).addRegion(getRegionRef(cell_label));
        else  getRegionRef(cell_label).addRegion(getRegionRef(label));
    }    
}
    
void Solver::newRegion(const int& row, const int& col) 
{
    regions_.emplace_back(regions_.size()+1, state_->at(row).at(col));
    // std::cout << "Create new region. label: " << regions_.size() << "mean: " << regions_[regions_.size()-1].mean_ << std::endl;
}
}