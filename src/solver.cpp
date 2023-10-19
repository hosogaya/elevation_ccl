#include <elevation_ccl/solver.h>

namespace ccl
{
Region::Region(int label, const Matrix& s)
{
    label_ = label;
    component_num_ = 1;
    mean_ = s;
    // variance_.resize(s.size(), s.size());
    // variance_.setZero();
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


bool Solver::canConnect(const int& row, const int& col, const int& label) const
{
    const int cur_label = labels_->coeff(row, col);
    // connect with a cell
    if (cur_label==0)
    {   
        const ccl::Vector& state = state_->at(row).at(col);
        if ((regions_[label].mean_ - state).squaredNorm() < distance_threshold_) return true;
        else return false;
    }
    else // connect with a region 
    {
        if ((regions_[label].mean_ - regions_[cur_label].mean_).squaredNorm() < distance_threshold_) return true;
        else return false;
    }
    
    return false;
}

void Solver::connect(const int& row, const int& col, const int& label)
{
    const int cur_label = labels_->coeff(row, col);
    // connect with a cell
    if (cur_label==0)
    {   
        const Vector& state = state_->at(row).at(col);
        regions_[label].addCell(state);
    }
    else // connect with a region 
    {
        if (label < cur_label) regions_[label].addRegion(regions_[cur_label]);
        else  regions_[cur_label].addRegion(regions_[label]);
    }    
}
    
void Solver::newRegion(const int& row, const int& col) 
{
    regions_.emplace_back(regions_.size(), state_->at(row).at(col));
}
}