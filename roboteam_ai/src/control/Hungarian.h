///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>
#include "roboteam_ai/src/world/Robot.h"
#include "roboteam_utils/Vector2.h"

using namespace std;

namespace rtt {

class HungarianAlgorithm {
public:
    HungarianAlgorithm() = default;
    double Solve(vector<vector<double> >& DistMatrix, vector<int>& Assignment);
    std::map<int, Vector2> getRobotPositions(std::vector<int> robotIds, bool ourTeam, std::vector<Vector2> targetLocations);

private:
    std::vector<std::pair<Vector2, Vector2>> calculateClosestPathsFromTwoSetsOfPoints(std::vector<Vector2> set1,
                                                                                      std::vector<Vector2> set2);
    bool validateInput(std::vector<Vector2> const &set1, std::vector<Vector2> const &set2);

    void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
    void buildassignmentvector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nOfRows);
    void step2a(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
            bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
            bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
            bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
            bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
            bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
};

}
#endif