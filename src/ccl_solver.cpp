#include <elevation_ccl/ccl_solver.h>
#include <iostream>

namespace elevation_ccl
{

Region::Region(int label, const ccl::Vector& s)
{
    label_ = label;
    component_num_ = 1;
    mean_ = s;
    variance_.resize(s.size(), s.size());
    variance_.setZero();
    is_root_ = true;
}

void Region::addCell(const ccl::Vector& s)
{
    mean_ = (mean_*component_num_ + s)/(component_num_);
    ++component_num_;
    // std::cout << "mean: " << mean_ << " " << "num: " << component_num_ << std::endl;
}

void Region::addRegion(Region& r)
{
    mean_ = (mean_*component_num_ + r.mean_*r.component_num_)/(component_num_ + r.component_num_);
    component_num_ += r.component_num_;
    r.is_root_ = false;
    r.label_ = label_;
    // std::cout << "mean: " << mean_ << " " << "num: " << component_num_ << std::endl;
}

/*
    Override functions of CclSolver
*/
bool CclSolver::isValid(const int& row, const int& col) const
{
    for (const auto& map: *score_) 
    {
        if (map.first(row, col) == NAN) return false; 
        if (map.first(row, col) > map.second) return false;
    }
    if (row < 2 || row > rows_-3) return false;
    if (col < 2 || col > cols_-3) return false;
    return true;
}

bool CclSolver::canConnect(const int& row, const int& col, const int& label) const
{
    const int cell_label = labels_->coeff(row, col);
    // connect with a cell
    if (cell_label==0)
    {   
        // std::cout << "Check if region " << label << " can connect with a cell(" << row << "," << col << ")" << std::endl;
        const ccl::Vector& state = state_->at(row).at(col);
        if ((getRegion(label).mean_ - state).squaredNorm() < distance_threshold_) return true;
        else {
            // std::cout << "region mean: " << getRegion(label).mean_ << std::endl;    
            // std::cout << "state vector: " << state_->at(row)[col] << std::endl;
            // std::cout << "distance: " << (getRegion(label).mean_ - state).squaredNorm() << std::endl;
            return false;
        }
    }
    else // connect with a region 
    {
        // std::cout << "Check if region " << label << " can connect with region " << cell_label << std::endl;
        if ((getRegion(label).mean_ - getRegion(cell_label).mean_).squaredNorm() < distance_threshold_) return true;
        else return false;
    }
    
    return false;
}

void CclSolver::connect(const int& row, const int& col, const int& label)
{
    const int cell_label = labels_->coeff(row, col);
    if (cell_label == label) return;
    // connect with a cell
    if (cell_label==0)
    {   
        // std::cout << "add cell" << std::endl;
        const ccl::Vector& state = state_->at(row).at(col);
        getRegionRef(label).addCell(state);
    }
    else // connect with a region 
    {
        // std::cout << "add region" << std::endl;
        if (label < cell_label) getRegionRef(label).addRegion(getRegionRef(cell_label));
        else  getRegionRef(cell_label).addRegion(getRegionRef(label));
        --region_num_;
    }    
}
    
void CclSolver::newRegion(const int& row, const int& col) 
{
    regions_.emplace_back(regions_.size()+1, state_->at(row).at(col));
    ++region_num_;
    // std::cout << "Create new region. label: " << regions_.size() << "mean: " << regions_[regions_.size()-1].mean_ << std::endl;
}

bool CclSolver::initialize(const ccl::StateMatrix& state, const ccl::ScoreMatrix& score, ccl::LabelMatrix& labels)
{
    if (score.size() == 0) return false;
    rows_ = score[0].first.rows();
    cols_ = score[0].first.cols();

    if (state.size() != rows_ || !std::all_of(state.begin(), state.end(), [this](const auto& x){return x.size() == cols_;})) 
    {
        std::cout << "matrixes in x have different size." << std::endl;
        return false;
    }

    // set vars
    state_ = &state;
    score_ = &score;
    labels_ = &labels;
    labels_->resize(rows_, cols_);
    labels_->setZero();
    table_.resize(1);
    table_[0] = std::numeric_limits<int>::infinity();
    m_ = 1;
    // for (int i=0; i<state.size(); ++i)
    //     for (int j=0; j<state[i].size(); ++j)
    //         std::cout << "size: " << state[i][j].size() << " " <<  "cell (" << i << "," << j << "): " << state[i][j].transpose() << std::endl;
    // for (int i=0; i<score.size(); ++i)
    //     std::cout << score[i].first << std::endl << std::endl;

    region_num_ = 0;
    regions_.clear();
    regions_.shrink_to_fit();

    return true;
}
}