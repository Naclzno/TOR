#ifndef EWM_HH
#define EWM_HH

#include <vector>
#include <cmath>
#include <numeric>

std::vector<int> calculateScores(const std::vector<std::vector<double>>& matrix, double scoreThreshold) {
    
    double sum1 = 0, sum2 = 0, sum3 = 0;
    double eSum1 = 0, eSum2 = 0, eSum3 = 0;
    double e1, e2, e3;
    double w1, w2, w3;

    for (const auto& row : matrix) { 
        sum1 += row[0]; 
        sum2 += row[1]; 
        sum3 += row[2]; 
    }

    std::vector<std::vector<double>> normalizedMatrix;
    for (const auto& row : matrix) {
        std::vector<double> newRow = { row[0] / sum1, row[1] / sum2, row[2] / sum3 };
        normalizedMatrix.push_back(newRow);
    }

    std::vector<std::vector<double>> logMatrix;
    
    for (const auto& row : normalizedMatrix) {
        std::vector<double> newRow(3, 0.0); 
        newRow[0] = row[0] == 0 ? 0 : std::log(row[0]); 
        newRow[1] = row[1] == 0 ? 0 : std::log(row[1]); 
        newRow[2] = row[2] == 0 ? 0 : std::log(row[2]); 
        logMatrix.push_back(newRow);
    }

    for (size_t i = 0; i < normalizedMatrix.size(); ++i) {
        eSum1 += normalizedMatrix[i][0] * logMatrix[i][0]; 
        eSum2 += normalizedMatrix[i][1] * logMatrix[i][1];
        eSum3 += normalizedMatrix[i][2] * logMatrix[i][2];
    }

    double factor = -1 / std::log(3);
    e1 = factor * eSum1;
    e2 = factor * eSum2;
    e3 = factor * eSum3;

    double sum = (1 - e1) + (1 - e2) + (1 - e3);
    w1 = (1 - e1) / sum;
    w2 = (1 - e2) / sum;
    w3 = (1 - e3) / sum;

    std::vector<int> highScoreIndices;

    for (size_t i = 0; i < matrix.size(); ++i) { 
        const auto& row = matrix[i]; 
        double score_i = w1 * (row[0] / (row[0] + 1)) + w2 * (row[1] / (row[1] + 1)) + w3 * (row[2] / (row[2] + 1));
        if (score_i > scoreThreshold) {
            highScoreIndices.push_back(i);  
        }
    }    

    return highScoreIndices; 
}

#endif // EWM_HH
