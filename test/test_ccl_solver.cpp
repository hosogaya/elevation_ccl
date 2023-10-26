#include <elevation_ccl/ccl_solver.h>
#include <iostream>

int main()
{
    elevation_ccl::CclSolver solver;    

    ccl::StateMatrix state(6);
    ccl::ScoreMatrix score;
    ccl::LabelMatrix label(6, 6);


    ccl::Matrix slope(6, 6);
    ccl::Matrix roughness(6, 6);

    for (size_t i=0; i<state.size(); ++i)
    {
        state[i].resize(6);
        for (size_t j=0; j<state[i].size(); ++j)
        {
            state[i][j].resize(3);
            if (i < 3) 
            {
                state[i][j](0) = 0.0;
                state[i][j](1) = 0.0;
                state[i][j](2) = 1.0;
            }
            else
            {
                state[i][j](0) = 0.0;
                state[i][j](1) = std::sqrt(0.5);
                state[i][j](2) = std::sqrt(0.5);
            }
            // std::cout << "size: " << state[i][j].size() << " " <<  "cell (" << i << "," << j << "): " << state[i][j].transpose() << std::endl;
            slope(i,j) = std::acos(state[i][j](2));
            roughness(i,j) = 0.0;
        }
    }
    // std::cout << "slope: " << std::endl << slope << std::endl;
    // std::cout << "roughness: " << std::endl << roughness << std::endl;

    score.emplace_back(std::make_pair(roughness, 1.0));
    score.emplace_back(std::make_pair(slope, 1.0));

    solver.setDistanceThres(0.1);
    solver.initialize(state, score, label);
    int scan_num = 0;
    solver.firstScan();
    scan_num++;
    while (1)
    {
        ++scan_num;
        if (!solver.backwardScan()) break;
        ++scan_num;
        if (!solver.forwardScan()) break;
    }
    std::cout << "label: " << std::endl;
    std::cout << label << std::endl;
    std::cout << "region num: " << solver.getRegionNum() <<  std::endl;
    std::cout << "scan num: " << scan_num << std::endl;

    return 0; 
}