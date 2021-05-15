//
// Created by Ivan Morozov on 11.04.2021.
//

#ifndef TPRT_C_IJ_MATRIX_H
#define TPRT_C_IJ_MATRIX_H

#include <Eigen/Dense>

using namespace Eigen;

class CIJMatrix {
public:
    static MatrixXf iso_c_ij(float vp, float vs, float rho) {
        MatrixXf c_ij = MatrixXf::Zero(6, 6);
        float diag_value = 2 * rho * vs * vs;
        MatrixX3f part1 = Vector3f(diag_value, diag_value, diag_value).asDiagonal();
        c_ij.block(0, 0, 3, 3) = part1.array() + rho * (float)(pow(vp, 2) - 2 * pow(vs, 2));
        c_ij.block(3, 3, 3, 3) =
                Vector3f(rho * pow(vs, 2), rho * pow(vs, 2), rho * pow(vs, 2)).asDiagonal();
        return c_ij;
    }

    static Tensor<float, 4> c_ijkl(MatrixXf c_ij) {
        Tensor<float, 4> c_ijkl(3,3,3,3);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    for (int l = 0; l < 3; l++) {
                        int from_i = i * (i == j) + (6 - i - j) * (i != j);
                        int from_j = k * (k == l) + (6 - k - l) * (k != l);

                        c_ijkl(i, j, k, l) = c_ij(from_i, from_j);
                    }
                }
            }
        }
        return c_ijkl;
    }
};

#endif //TPRT_C_IJ_MATRIX_H
