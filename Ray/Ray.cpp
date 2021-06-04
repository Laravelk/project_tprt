#include "Ray.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/CXX11/Tensor>
#include "../include/LBFGSB.h"  // Note the different header file
#include "../Math/C_IJ_Matrix.h"
#include "../Math/Optimize.h"
#include "RayData.h"

using Eigen::VectorXd;
using namespace LBFGSpp;

namespace ray_tracing {
    void Ray::optimizeTrajectory() {
        std::vector<double> vector;

        for (unsigned long i = 1; i < trajectory.size() - 1; i++) {
            vector.push_back(trajectory.at(i)[0]);
            vector.push_back(trajectory.at(i)[1]);
        }

        VectorXd x(vector.size());
        for (int i = 0; i < vector.size(); i++) {
            x(i) = vector[i];
        }

        auto *ray_data = new RayData(this);

        Optimize optimize(ray_data);

        LBFGSParam<double> param;
        param.epsilon = 1e-6;
        param.ftol = 1e-9;
        param.epsilon_rel = 1e-17;
        param.max_iterations = 20;
        double fx;
        LBFGSSolver<double> solver(param);
        try {
            int niter = solver.minimize(optimize, x, fx);
            std::cerr << niter << " iterations" << std::endl;
            std::cerr << "x = \n" << x.transpose() << std::endl;
            std::cerr << "f(x) = " << fx << std::endl;
        } catch (std::exception exception) {
            std::cout << "catch exception" << std::endl;
            for (int i = 0; i < trajectory.size(); i++) {
                std::cout << trajectory[i][0] << " " << trajectory[i][1] << " " << trajectory[i][2] << std::endl;
            }
            param.epsilon = 1e-2;
            int niter = solver.minimize(optimize, x, fx);
//            std::cerr << niter << " iterations" << std::endl;
//            std::cerr << "x = \n" << x.transpose() << std::endl;
//            std::cerr << "f(x) = " << fx << std::endl;
        }

        traveltime = fx;
        trajectory = ray_data->trajectory;
//        for (int i = 0; i < trajectory.size(); i++) {
//            std::cerr << trajectory[i][0] << " " << trajectory[i][1] << " "
//                      << trajectory[i][2] << std::endl;
//        }

        delete ray_data;
    }

/// compute ray in layer and create segments
    void Ray::computeSegmentsRay() {
        auto source_location = source.getLocation();
        auto receiver_location = receiver.getLocation();

        float diff_x = receiver_location[0] - source_location[0];
        float diff_y = receiver_location[1] - source_location[1];
        ulong trajectory_part_count = ray_code.size();
        float step_x = diff_x / trajectory_part_count;
        float step_y = diff_y / trajectory_part_count;
        float x = 0, y = 0, z = 0;
        x = source_location[0];
        y = source_location[1];

        trajectory.push_back({x, y, source_location[2]});
        for (ulong i = 0; i < trajectory_part_count - 1; i++) {
            x += step_x;
            y += step_y;
            Horizon *hor =
                    velocity_model->getLayer(ray_code.at(i + 1).layerNumber)->getTop();
            z = hor->getDepth({x, y});
            trajectory.push_back({x, y, z});
        }
        trajectory.push_back(
                {receiver_location[0], receiver_location[1], receiver_location[2]});
//
//        for (int i = 0; i < trajectory.size(); i++) {
//            std::cerr << trajectory[i][0] << " " << trajectory[i][1] << " "
//                      << trajectory[i][2] << std::endl;
//        }
    }

    void Ray::generateCode(const std::vector<std::array<int, 3>> rayCode) {
        for (auto ray_element : rayCode) {
            Direction direction;
            WaveType type;
            if (Direction::DOWN == ray_element[1]) {
                direction = Direction::DOWN;
            } else {
                direction = Direction::UP;
            }
            if (WaveType::SWave == ray_element[2]) {
                type = WaveType::SWave;
            } else {
                type = WaveType::PWave;
            }
            Code code(ray_element[0], direction, type);
            ray_code.push_back(code);
        }
    }

    void Ray::computePathWithRayCode() {
        computeSegmentsRay();
        optimizeTrajectory();
    }

    rapidjson::Document Ray::toJSON() {
        rapidjson::Document doc;
        /*rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        json_val.CopyFrom(source.toJSON(), allocator);
        doc.AddMember("Source", json_val, allocator);

        json_val.CopyFrom(receiver.toJSON(), allocator);
        doc.AddMember("Receiver", json_val, allocator);

        json_val.SetArray();
        auto trajectory = getTrajectoryP();
        for (const auto &tr_seg : trajectory) {
          tmp_json_val.SetArray();
          tmp_json_val.PushBack(tr_seg[0], allocator)
              .PushBack(tr_seg[1], allocator)
              .PushBack(tr_seg[2], allocator);
          json_val.PushBack(tmp_json_val, allocator);
        }
        doc.AddMember("TrajectoryP", json_val, allocator);

        json_val.SetArray();
        trajectory = getTrajectoryS();
        for (const auto &tr_seg : trajectory) {
          tmp_json_val.SetArray();
          tmp_json_val.PushBack(tr_seg[0], allocator)
              .PushBack(tr_seg[1], allocator)
              .PushBack(tr_seg[2], allocator);
          json_val.PushBack(tmp_json_val, allocator);
        }
        doc.AddMember("TrajectoryS", json_val, allocator);

        json_val.SetFloat(amplitudeP);
        doc.AddMember("AmplitudeP", json_val, allocator);

        json_val.SetFloat(timeP);
        doc.AddMember("TimeP", json_val, allocator);

        json_val.SetFloat(amplitudeS);
        doc.AddMember("AmplitudeS", json_val, allocator);

        json_val.SetFloat(timeS);
        doc.AddMember("TimeS", json_val, allocator);

        json_val.SetString("NONE");
        doc.AddMember("Record", json_val, allocator);
        */
        return doc;
    }

    Vector3f Ray::rayPolarization() {
        using namespace Eigen;

        int row_count = velocity_model->getLayersCount() - 2;

        float vel0 = velocity_model->getLayer(0)->Vp;
        float rho0 = velocity_model->getLayer(0)->density;
        float sou_factor = pow(1 / (4 * M_PI) * (1000 / rho0) * (1000 / vel0), 3);
        Vector3f polariz0 = source.unitPolarization(trajectory[1], waveType);

        if (velocity_model->getLayersCount() > 1) {
            auto vecs = getVectors();
            auto vels = getVels(velocity_model->getLayersCount() - 2);

            MatrixX3f inc_slows = MatrixX3f::Zero(vecs.size() - 1, 3);

            for (int i = 0; i < vecs.size() - 1; i++) {
                inc_slows.row(i) = vecs[i] / vels[i];
            }

//            for (int i = 0; i < vecs.size(); i++) {
//                std::cerr << vecs[i] << std::endl << std::endl;
//            }

//            for (int i = 0; i < vecs.size(); i++) {
//                std::cerr << vels[i] << std::endl << std::endl;
//            }

//            std::cerr << inc_slows << std::endl << std::endl;
            std::vector<Layer::Properties> props;
            for (int i = 0; i < velocity_model->getLayersCount() - 1; i++) {
                Layer::Properties prop(velocity_model->getLayers()[i]->Vp,
                                       velocity_model->getLayers()[i]->Vs,
                                       velocity_model->getLayers()[i]->density);
                props.push_back(prop);
            }

            MatrixX3f ezs = MatrixX3f::Zero(row_count, 3);

            for (int i = 1; i < velocity_model->getLayersCount() - 1; i++) {
                std::vector<float> normal =
                        velocity_model->getLayers()[i]->top->getNormal(
                                {trajectory[i][0], trajectory[i][1]});
                for (int j = 0; j < 3; j++) {
                    ezs(i - 1, j) = normal[j] * (-1);
                }
            }

            ////////// EXS Block
            MatrixXf exs = MatrixXf::Random(
                    row_count, 3); // NOTE: It's working, diff in derivatives

            for (int i = 0; i < row_count; i++) {
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 3; k++) {
                        exs(i, k) = vecs[i][j] * ezs(i, j) * ezs(i, k);
                    }
                }
            }

            for (int i = 0; i < row_count; i++) {
                for (int j = 0; j < 3; j++) {
                    exs(i, j) = vecs[i][j] - exs(i, j);
                }
            }

            MatrixX3f new_ezs = MatrixX3f::Zero(row_count, 3);
            for (int i = 0; i < velocity_model->getLayersCount() - 2; i++) {
                new_ezs(i, 0) = ezs(i, 2);
                new_ezs(i, 1) = 0;
                new_ezs(i, 2) = ezs(i, 0) * (-1);
            }

            VectorXf all_exs_vector = VectorXf::Zero(row_count);

            for (int i = 0; i < row_count; i++) {
                for (int j = 0; j < 3; j++) {
                    new_ezs(i, j) = all_exs_vector(i) * new_ezs(i, j);
                }
            }

            for (int i = 0; i < row_count; i++) {
                for (int j = 0; j < 3; j++) {
                    exs(i, j) = exs(i, j) + new_ezs(i, j);
                }
            }

            for (int i = 0; i < row_count; i++) {
                float norm = exs.row(i).norm();
                for (int j = 0; j < 3; j++) {
                    exs(i, j) = exs(i, j) / norm;
                }
            }

            MatrixX3f eys = MatrixX3f::Zero(row_count, 3);
            for (int i = 0; i < row_count; i++) {
                Vector3f vector1 = ezs.row(i);
                Vector3f vector2 = exs.row(i);
                eys.row(i) = vector1.cross(vector2);
            }

            MatrixX3f out_ps = MatrixX3f::Zero(row_count, 3);
            MatrixX3f out_shs = eys;

            for (int i = 1; i < vecs.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    out_ps(i - 1, j) = vecs[i][j];
                }
            }

            MatrixX3f out_svs = MatrixX3f::Zero(row_count, 3);
            for (int i = 0; i < row_count; i++) {
                Vector3f vector1 = out_shs.row(i);
                Vector3f vector2 = out_ps.row(i);
                out_svs.row(i) = vector1.cross(vector2);
            }

            //      std::cerr << out_ps << std::endl << std::endl;
            //      std::cerr << out_svs << std::endl << std::endl;
            //      std::cerr << out_shs << std::endl << std::endl;

            std::vector<MatrixX3f> tensor_matrix_vector = {out_ps, out_svs, out_shs};
            Tensor<float, 3> out_triplets(row_count, tensor_matrix_vector.size(), 3);

            for (int i = 0; i < row_count; i++) {
                for (int j = 0; j < tensor_matrix_vector.size(); j++) {
                    for (int k = 0; k < 3; k++) {
                        out_triplets(i, k, j) = tensor_matrix_vector[j](i, k);
                    }
                }
            }

//            std::cerr << out_triplets << std::endl << std::endl;

            MatrixX3f wave_ids = MatrixX3f::Zero(trajectory.size() - 1, 3);
            for (int k = 0; k < trajectory.size() - 1;
                 k++) { // TODO: get WaveType from layer and choice
                //          WaveType type = velocity_model->getLayer(i).
                WaveType type = WaveType::PWave;
                if (WaveType::PWave == type) {
                    wave_ids(k, 0) = 1;
                    wave_ids(k, 1) = 0;
                    wave_ids(k, 2) = 0;
                } else {
                    wave_ids(k, 0) = 0;
                    wave_ids(k, 1) = 1;
                    wave_ids(k, 2) = 1;
                }
            }

            for (int k = 0; k < inc_slows.rows(); k++) {
                Vector3f slow = inc_slows.row(k);
                Vector3f res = rt_coeffs_iso(slow, polariz0, ezs.row(k), eys.row(k), 1, props[k],
                              props[k + 1]);
                Matrix3Xf matrix_from_tensor = Matrix3f::Zero(3, 3);
                for (int j = 0; j < 3; j++) {
                    for (int g = 0; g < 3; g++) {
                        matrix_from_tensor(j, g) = out_triplets(k, j, g) * wave_ids.row(k)(g) * sqrt((polariz0.transpose() * polariz0).sum());
                    }
                }
//                std::cerr << matrix_from_tensor  << std::endl << std::endl;
//                std::cerr << res << std::endl << std::endl;
//                std::cerr << wave_ids.row(k).transpose() << std::endl << std::endl;
//                std::cerr << sqrt((polariz0.transpose() * polariz0).sum()) << std::endl << std::endl;
                for (int j = 0; j < 3; j++) {
                    float s = res.dot(matrix_from_tensor.row(j));
//                    std::cerr << s << std::endl;
                    polariz0[j] = s;
                }
//                std::cerr << std::endl;
            }
        }


        polariz0 = sou_factor * polariz0 * (source.getMagnitude() / 100.0f);
        return polariz0;

        //////////// End EXS Block
    }

    void Ray::createDefaultRayCode(WaveType type) {
        const int wave_type = type == WaveType::PWave ? 0 : 1;
        const int layers_count = velocity_model->getLayersCount();

        ray_code.clear();
//        ray_code.reserve((layers_count * 2));
//
//        for (int i = 0; i < layers_count - 1; i++) {
//            ray_code.emplace_back(Code(i, Direction::DOWN, type));
//        }

        for (int i = 0; i < layers_count; i++) {
            ray_code.emplace_back(Code(i, Direction::DOWN, type));
        }

//        for (int i = layers_count - 2; i >= 1; i--) {
//            ray_code.emplace_back(Code(i, Direction::UP, type));
//        }
    }

    Vector3f Ray::rt_coeffs_iso(Vector3f inc_slow, Vector3f inc_polariz,
                                Vector3f ez, Vector3f ey, int rt_sign,
                                Layer::Properties lr_props_1,
                                Layer::Properties lr_props_2) {
        if (inc_polariz.isZero()) {
             return Vector3f::Zero();
        }
        inc_polariz = inc_polariz / (inc_polariz.transpose() * inc_polariz);

        float ez_sum = (inc_slow.transpose() * ez).sum();
        if (ez_sum != 0) {
            if (ez_sum > 0) {
                ez_sum = 1.0;
            } else {
                ez_sum = -1.0;
            }
            ez = ez_sum * ez;
        }

        float vp1 = lr_props_1.vp;
        float vs1 = lr_props_1.vs;
        float rho1 = lr_props_1.density;

        float vp2 = lr_props_2.vp;
        float vs2 = lr_props_2.vs;
        float rho2 = lr_props_2.density;

        Vector3f tang_slow = inc_slow - (inc_slow.transpose() * ez).sum() * ez;
        Vector3f refl_p_slow =
                tang_slow -
                (sqrt(1.0 / pow(vp1, 2) - (tang_slow.transpose() * tang_slow).sum()) *
                 ez);
        Vector3f refl_s_slow;
        if (vs1 != 0) {
            refl_s_slow = tang_slow - sqrt(1.0 / pow(vs1, 2) -
                                           (tang_slow.transpose() * tang_slow).sum()) *
                                      ez;
        } else {
            refl_s_slow = Vector3f::Zero();
        }

        Vector3f refl_p_polariz = refl_p_slow * vp1;
        Vector3f refl_sh_polariz = ey * (vs1 != 0);
        Vector3f refl_sv_polariz = refl_sh_polariz.cross(refl_s_slow * vs1);

        Vector3f trans_p_slow;
        Vector3f trans_s_slow;
        if (vp2 != 0) {
            trans_p_slow = tang_slow + sqrt(1.0 / pow(vp2, 2) -
                                            (tang_slow.transpose() * tang_slow).sum()) *
                                       ez;
        } else {
            trans_p_slow = Vector3f::Zero();
        }

        if (vs2 != 0) {
            trans_s_slow = tang_slow + sqrt(1.0 / pow(vs2, 2) -
                                            (tang_slow.transpose() * tang_slow).sum()) *
                                       ez;
        } else {
            trans_s_slow = Vector3f::Zero();
        }

        Vector3f trans_p_polariz = trans_p_slow * vp2;
        Vector3f trans_sh_polariz = ey * (vs2 != 0);
        Vector3f trans_sv_polariz = trans_sh_polariz.cross(trans_s_slow * vs2);

        // Stiffness tensors:
        MatrixXf c_ij1 = CIJMatrix::iso_c_ij(vp1, vs1, rho1);
        MatrixXf c_ij2 = CIJMatrix::iso_c_ij(vp2, vs2, rho2);

        Tensor<float, 4> c_ijkl1 = CIJMatrix::c_ijkl(c_ij1);
        Tensor<float, 4> c_ijkl2 = CIJMatrix::c_ijkl(c_ij2);

        Vector3f refl_p_factors = Vector3f::Zero();
        Vector3f refl_sv_factors = Vector3f::Zero();
        Vector3f refl_sh_factors = Vector3f::Zero();

        Vector3f trans_p_factors = Vector3f::Zero();
        Vector3f trans_sv_factors = Vector3f::Zero();
        Vector3f trans_sh_factors = Vector3f::Zero();

        // Elements of the future matrix:
        Vector3f inc_factors = Vector3f::Zero();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int p = 0; p < 3; p++) {
                    for (int q = 0; q < 3; q++) {
                        inc_factors[i] = inc_factors[i] + c_ijkl1(i, j, p, q) * ez[j] *
                                                          inc_slow[p] * inc_polariz[q];

                        refl_p_factors[i] =
                                (refl_p_factors[i] + c_ijkl1(i, j, p, q) * ez[j] *
                                                     refl_p_slow[p] * refl_p_polariz[q]);
                        refl_sv_factors[i] =
                                (refl_sv_factors[i] + c_ijkl1(i, j, p, q) * ez[j] *
                                                      refl_s_slow[p] * refl_sv_polariz[q]);
                        refl_sh_factors[i] = (refl_sh_factors[i] +
                                              c_ijkl1(i, j, p, q) * ez[j] * refl_s_slow[p] *
                                              refl_sh_polariz[q]); // ????

                        trans_p_factors[i] =
                                (trans_p_factors[i] + c_ijkl2(i, j, p, q) * ez[j] *
                                                      trans_p_slow[p] * trans_p_polariz[q]);
                        trans_sv_factors[i] =
                                (trans_sv_factors[i] + c_ijkl2(i, j, p, q) * ez[j] *
                                                       trans_s_slow[p] * trans_sv_polariz[q]);
                        trans_sh_factors[i] =
                                (trans_sh_factors[i] + c_ijkl2(i, j, p, q) * ez[j] *
                                                       trans_s_slow[p] * trans_sh_polariz[q]);
                    }
                }
            }
        }

        MatrixXf matrix_upper_none_transpose = MatrixXf::Zero(6, 3);
        matrix_upper_none_transpose.row(0) = refl_p_polariz;
        matrix_upper_none_transpose.row(1) = refl_sv_polariz;
        matrix_upper_none_transpose.row(2) = refl_sh_polariz;
        matrix_upper_none_transpose.row(3) = (-1) * trans_p_polariz;
        matrix_upper_none_transpose.row(4) = (-1) * trans_sv_polariz;
        matrix_upper_none_transpose.row(5) = (-1) * trans_sh_polariz;

        MatrixXf matrix_upper = matrix_upper_none_transpose.transpose();

        MatrixXf matrix_lower_none_transpose = MatrixXf::Zero(6, 3);
        matrix_lower_none_transpose.row(0) = refl_p_factors;
        matrix_lower_none_transpose.row(1) = refl_sv_factors;
        matrix_lower_none_transpose.row(2) = refl_sh_factors;
        matrix_lower_none_transpose.row(3) = (-1) * trans_p_factors;
        matrix_lower_none_transpose.row(4) = (-1) * trans_sv_factors;
        matrix_lower_none_transpose.row(5) = (-1) * trans_sh_factors;

        MatrixXf matrix_lower = matrix_lower_none_transpose.transpose();
        MatrixXf matrix = MatrixXf::Zero(6, 6);
        matrix.block(0, 0, 3, 6) = matrix_upper;
        matrix.block(3, 0, 3, 6) = matrix_lower;

//        std::cerr << matrix << std::endl << std::endl;

        VectorXf right_part = VectorXf(6);
        right_part.head(3) = (-1) * inc_polariz;
        right_part.tail(3) = (-1) * inc_factors;

        std::vector<int> rows;
        if (vp1 != 0 && vs1 != 0 && vp2 != 0 && vs2 != 0) {
            rows.push_back(0);
            rows.push_back(1);
            rows.push_back(2);
            rows.push_back(3);
            rows.push_back(4);
            rows.push_back(5);
        } else if (vs1 != 0 && vp2 != 0 && vs2 == 0) {
            rows.push_back(2);
            rows.push_back(3);
            rows.push_back(4);
            rows.push_back(5);
        } else if (vs1 == 0 && vp2 != 0 && vs2 == 0) {
            rows.push_back(2);
            rows.push_back(5);
        } else if (vs1 != 0 && vp2 == 0 && vs2 == 0) {
            rows.push_back(3);
            rows.push_back(4);
            rows.push_back(5);
        } else {
            rows.push_back(5);
        }

        // Determine which columns to preserve
        std::vector<bool> cols = {vp1 != 0, vs1 != 0, vs1 != 0,
                                  vp2 != 0, vs2 != 0, vs2 != 0 };

        int count_of_cols = 0;
        for (auto && col : cols) {
            if (col) {
                count_of_cols += 1;
            }
        }

        MatrixXf left_part = MatrixXf(rows.size(), count_of_cols);
        for (int i = 0; i < count_of_cols; i++) {
            if (cols[i]) {
                left_part.col(i) = matrix.col(i);
            }
        }

        VectorXf new_right_part = VectorXf(rows.size());

        for (int i = 0; i < rows.size(); i++) {
            if (cols[i]) {
                new_right_part(i) = right_part(i);
            }
        }

        VectorXf coeffs = VectorXf::Zero(rows.size());

//        std::cerr << left_part << std::endl << std::endl;
//        std::cerr << right_part << std::endl << std::endl;

        coeffs = left_part.colPivHouseholderQr().solve(right_part);
        auto left_inverse = left_part.inverse();
        auto res = left_inverse * right_part;

//        std::cerr << coeffs << std::endl << std::endl;
//        std::cerr << res << std::endl << std::endl;

        Vector3f res_coeff = Vector3f::Zero();

        if (rt_sign == 1) {
            return res_coeff = coeffs.tail(3);
        } else {
            return res_coeff = coeffs.head(3);
        }
    }

    Vector3f Ray::computeAmplitude() {
//        Vector3f polariz = rayPolarization();
        auto props = raySpreading();
        return Vector3f::Zero();
//        return polariz * props.discont_factor / props.spread * exp( -1.0 / 2 * M_PI * props.kmah_indx);
    }

    Ray::SpreadingProps Ray::raySpreading() {
        Vector3f vec0 = {trajectory[1][0] - trajectory[0][0],
                        trajectory[1][1] - trajectory[0][1],
                        trajectory[1][2] - trajectory[0][2]
        };

        vec0 = vec0 / (sqrt(pow(vec0(0), 2) + pow(vec0(1), 2) + pow(vec0(2), 2)) + 1e-15);
        float vel0 = velocity_model->getLayer(0)->Vp;
        float theta = atan2(sqrt(pow(vec0(0), 2) + pow(vec0(1), 2)), vec0[2]);
        float phi = atan2(vec0[1], vec0[0]);

//        std::cerr << theta << " " << std::endl << phi << std::endl;
        Matrix3f qq0 = Matrix3f();
        qq0 << 0, 0, sin(theta) * cos(phi),
                0, 0, sin(theta) * sin(phi),
                0, 0, cos(theta);

        Matrix3f pp0 = Matrix3f();
        pp0 << cos(theta) * cos(phi) / vel0, (-1.0f) * sin(phi) / vel0, 0,
                cos(theta) * sin(phi) / vel0, cos(phi) / vel0, 0,
                (-1.0f) * sin(theta) / vel0, 0, 0;

//        std::cerr << pp0 << std::endl;
        std::vector<MatrixXf> qq_pps;
        int kmah_index = 0;
        float discont_factor = 1;

        if (1 < velocity_model->getLayersCount()) {
            auto vecs = getVectors();

//            std::cerr << "vecs" << std::endl;
//            for (int i = 0; i < vecs.size(); i++) {
////               std::cerr << vecs[i] << std::endl << std::endl;
//            }
            auto vels = getVels(velocity_model->getLayersCount());
            auto dists = getDistance();
//            std::cerr << "dist" << std::endl;
//            for (int i = 0; i < dists.size(); i++) {
//                std::cerr << dists[i] << std::endl << std::endl;
//            }

            MatrixX3f points = MatrixX3f(trajectory.size() - 1, 3);
            for (int i = 1; i < trajectory.size(); i++) {
                points.row(i - 1) = Vector3f(trajectory[i][0], trajectory[i][1], trajectory[i][2]);
            }
//            std::cerr << "points" << std::endl << points << std::endl << std::endl;

            MatrixX3f grads_f = MatrixX3f(vecs.size() - 1, 3);
            std::vector<Matrix2f> hessians;
            std::vector<Matrix3f> hessians_f;
            std::vector<Vector3f> grads_vectors;
//
//            Matrix2f h1;
//            Matrix2f h2;
//            Matrix2f h3;
//            Matrix2f h4;
//            h1 << 0, 0, 0, 0;
//            h2 << -9.7086542e-06, -9.7086542e-06, -9.7086542e-06, -9.7086542e-06;
//            h3 << 0, 0, 0, 0;
//            h4 << 0, 0, 0, 0;
//
//            hessians.push_back(h1); // TODO: delete it
//            hessians.push_back(h2);
//            hessians.push_back(h3);
//            hessians.push_back(h4);
//
//
//            Vector3f grad1;
//            Vector3f grad2;
//            Vector3f grad3;
//            Vector3f grad4;
//
//            grad1 << (-1.0f) * 0, (-1.0f) * 0, 1.0f;
//            grad2 << (-1.0f) * 0.03080193, (-1.0f) * 0.03080193, 1.0f;
//            grad3 << (-1.0f) * 0.01743114, (-1.0f) * 0.00091353, 1.0f;
//            grad4 << (-1.0f) * 0, (-1.0f) * 0, 1.0f;
//
//            grads_vectors.push_back(grad1); // TODO: delete it
//            grads_vectors.push_back(grad2);
//            grads_vectors.push_back(grad3);
//            grads_vectors.push_back(grad4);

//            for (int i = 0; i < hessians.size(); i++) { // TODO: delete cycle
//                Matrix3f hessian_f;
//                hessian_f << hessians[i](0, 0), hessians[i](0, 1), 0.0f,
//                        hessians[i](1, 0), hessians[i](1, 1), 0.0f,
//                        0.0f, 0.0f, 0.0f;
//                std::cerr << "hessian_f: " << std::endl << hessian_f << std::endl << std::endl;
//                hessians_f.push_back(hessian_f);
//                VectorXf grad_vector = VectorXf(3);
//                grads_f.row(i) = grads_vectors[i];
//            }

            for (int i = 1; i < trajectory.size() - 1; i++) {
                std::array<float, 2> grad =
                        velocity_model->getLayer(ray_code[i].layerNumber)->getTop()->getGradientInPoint(points(i - 1, 0), points(i - 1, 1));
                VectorXf grad_vector = VectorXf(3);
                grad_vector << (-1.0f) * grad[0], (-1.0f) * grad[1], 1.0f;
                grads_f.row(i - 1) = grad_vector;
                std::cerr << "grad: " << std::endl << grad_vector << std::endl << std::endl;
                Matrix2f hessian = velocity_model->getLayer(ray_code[i].layerNumber)->getTop()->getHessian(points(i - 1, 0),
                                                                                     points(i - 1, 1));
                hessians.push_back(hessian);
                Matrix3f hessian_f;
                hessian_f << hessian(0, 0), hessian(0, 1), 0.0f,
                        hessian(1, 0), hessian(1, 1), 0.0f,
                        0.0f, 0.0f, 0.0f;
//                std::cerr << "hessian_f: " << std::endl << hessian_f << std::endl << std::endl;
                hessians_f.push_back(hessian_f);
            }
//            std::cerr << "grads_f" << std::endl << grads_f << std::endl << std::endl;

//            std::cerr << "grads_f: " << std::endl;
//            std::cerr << grads_f << std::endl << std::endl;

//            std::cerr << "hessian_f: " << std::endl;
//            for (int i = 0; i < hessians_f.size(); i++) {
//                std::cerr << hessians_f[i] << std::endl << std::endl;
//            }

            MatrixX3f inc_slows = MatrixX3f::Zero(vecs.size() - 1, 3);
            MatrixX3f out_slows = MatrixX3f::Zero(vecs.size() - 1, 3);

            for (int i = 0; i < vecs.size() - 1; i++) {
                inc_slows.row(i) = vecs[i] / vels[i];
//                std::cerr << vecs[i] << std::endl << std::endl;
//                std::cerr << vels[i] << std::endl << std::endl;
            }

//            std::cerr << "inc_slows" << std::endl;
//            std::cerr << inc_slows << std::endl << std::endl;

            for (int i = 1; i < vecs.size(); i++) {
//                std::cerr << vecs[i] << " " << vels[i] << std::endl;
                out_slows.row(i - 1) = vecs[i] / vels[i];
            }

//            std::cerr << "out_slows" << std::endl;
//            std::cerr << out_slows << std::endl << std::endl;

            VectorXf a1 = VectorXf::Zero(vecs.size() - 1);
//            std::cerr << inc_slows << std::endl << std::endl;
//            std::cerr << out_slows << std::endl << std::endl;

            auto tmp_matrix = inc_slows - out_slows;
//            std::cerr << tmp_matrix << std::endl << std::endl;
            for (int i = 0; i < vecs.size() - 1; i++) {
                float norm = 1.0f / grads_f.row(i).norm();
                for (int j = 0; j < 3; j++) {
                    a1(i) += tmp_matrix(i, j) * grads_f(i, j);
                }
                a1(i) = a1(i) * norm * norm;
            }
//            std::cerr << "a1" << std::endl;
//            std::cerr << a1 << std::endl << std::endl;

            std::vector<Matrix3f> inc_phis;
            inc_phis.reserve(vecs.size() - 1);

            MatrixX3f tmp_tmp_matrix = MatrixX3f::Zero(vecs.size() - 1, 3);
            for (int i = 0; i < vecs.size() - 1; i++) {
                for (int j = 0; j < 3; j++) {
                    tmp_tmp_matrix(i, j) = inc_slows(i, j) * vels[i];
                }
            }
//            std::cerr << "tmp " << std::endl;
//            std::cerr << tmp_tmp_matrix << std::endl << std::endl;
//            std::cerr << "grads_f" << std::endl;
//            std::cerr << grads_f << std::endl << std::endl;

            for (int i = 0; i < vecs.size() - 1; i++) {
                Matrix3f matrix = Matrix3f::Zero();
                for (int j = 0; j < 3; j++) {
                    matrix.row(j) = tmp_tmp_matrix(i, j) * grads_f.row(i);
//                    std::cerr << matrix.row(j) << " = " <<  tmp_tmp_matrix(i, j) << " * " << grads_f.row(i) << std::endl;
                }
//                std::cerr << matrix << std::endl << std::endl;
                inc_phis.push_back(matrix);
            }

            std::vector<Matrix3f> out_phis;
            for (int i = 1; i < vecs.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    tmp_tmp_matrix(i - 1, j) = out_slows(i - 1, j) * vels[i];
                }
            }

            for (int i = 1; i < vecs.size(); i++) {
                Matrix3f matrix = Matrix3f::Zero();
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 3; k++) {
                        matrix.row(j) = tmp_tmp_matrix(i - 1, j) * grads_f.row(i - 1);
                    }
                }
                out_phis.push_back(matrix);
            }

//            // TODO: delete it
//            std::cerr << "inc_phis & out_phis:" << std::endl;
//            for (int i = 0; i < inc_phis.size(); i++) {
//                std::cerr << inc_phis[i] << std::endl << std::endl;
//                std::cerr << out_phis[i] << std::endl << std::endl << std::endl;
//            }

            std::vector<Matrix3f> bd_props11_additional;
            VectorXf inc_trace_vector = VectorXf::Zero(inc_phis.size());

            for (int i = 0; i < inc_phis.size(); i++) {
                inc_trace_vector(i) = 1.0f / inc_phis[i].diagonal().sum();
            }

            for (int i = 0; i < vecs.size() - 1; i++) {
                Matrix3f additional_difference = inc_phis[i] - out_phis[i];
//                std::cerr << additional_difference << std::endl << std::endl;
                Matrix3f res_matrix = Matrix3f::Zero();
                for (int j = 0; j < 3; j++) {
                    res_matrix = Matrix3f::Identity() - additional_difference * inc_trace_vector(i);
                }
                bd_props11_additional.push_back(res_matrix);
            }

            std::vector<Matrix3f> bd_props22_additional;
            VectorXf out_trace_vector = VectorXf::Zero(out_phis.size());

            for (int i = 0; i < out_phis.size(); i++) {
                out_trace_vector(i) = 1.0f / out_phis[i].diagonal().sum();
            }

            for (int i = 0; i < vecs.size() - 1; i++) {
                Matrix3f additional_difference = out_phis[i] - inc_phis[i];
//                std::cerr << additional_difference << std::endl << std::endl;
                Matrix3f res_matrix = Matrix3f::Zero();
                for (int j = 0; j < 3; j++) {
                    res_matrix = (Matrix3f::Identity() - additional_difference * out_trace_vector(i)).transpose();
                }
                bd_props22_additional.push_back(res_matrix);
            }

            std::vector<Matrix3f> bd_props21;

            std::vector<Matrix3f> tmp_bd_props21_out;
            std::vector<Matrix3f> tmp_bd_props21_inc;

            for (int i = 0; i < inc_phis.size(); i++) {
                Matrix3f out;
                Matrix3f inc;
                for (int j = 0; j < 3; j++) {
                    out = Matrix3f::Identity() - out_phis[i] * out_trace_vector(i);
                    inc = Matrix3f::Identity() - inc_phis[i] * inc_trace_vector(i);
                }
                tmp_bd_props21_out.push_back(out);
                tmp_bd_props21_inc.push_back(inc);
            }

            for (int i = 0; i < a1.size(); i++) {
                Matrix3f res_matrix;
                res_matrix = -1.0f * (a1(i) * tmp_bd_props21_out[i] * hessians_f[i] * tmp_bd_props21_inc[i].transpose());
                bd_props21.push_back(res_matrix);
            }

            std::vector<MatrixXf> bd_props;
            for (int i = 0; i < a1.size(); i++) {
                MatrixXf matrix = MatrixXf::Zero(6, 6);
//                std::cerr << bd_props11_additional[i] << std::endl << std::endl;
                matrix.block(0, 0, 3, 3) = bd_props11_additional[i];
                matrix.block(0, 3, 3, 3) = Matrix3f::Zero();
                matrix.block(3, 0, 3, 3) = bd_props21[i];
                matrix.block(3,3,3,3) = bd_props22_additional[i];
                bd_props.push_back(matrix);
//                std::cerr << std::endl << matrix << std::endl;
            }

            MatrixXf qq_pps1 = MatrixXf::Zero(6,6);

            qq_pps1.block(0, 0, 3, 3) = qq0;
            qq_pps1.block(3,0,3,3) = pp0;
            qq_pps1.block(0,3,3,3) = qq0 + vels[0] * dists[0] * pp0;
            qq_pps1.block(3,3,3,3) = pp0;
            qq_pps.push_back(qq_pps1);
//            std::cerr << "qq_pps1" << std::endl;
//            std::cerr << qq_pps1 << std::endl << std::endl;

            for (int i = 1; i < vels.size(); i++) {
                float vel = vels[i];
                float dist = dists[i];
                MatrixXf bd_prop = bd_props[i - 1];

//                std::cerr << bd_prop << std::endl << std::endl;
//                std::cerr << qq_pps[qq_pps.size() - 1].block(0,3, 6, 3) << std::endl << std::endl;

                MatrixX3f qq_pp = MatrixX3f::Zero(6, 3);
                qq_pp = bd_prop * qq_pps[qq_pps.size() - 1].block(0,3, 6, 3);
//                std::cerr << qq_pp << std::endl << std::endl;
                MatrixXf new_qq_pps = MatrixXf::Zero(6,6);
                new_qq_pps.block(0,0,6,3) =  qq_pp;
                new_qq_pps.block(0, 3, 3, 3) = qq_pp.block(0, 0, 3, 3)
                        + vel * dist  * qq_pp.block(3, 0, 3, 3);
                new_qq_pps.block(3, 3, 3, 3) = qq_pp.block(3, 0, 3 ,3);
                qq_pps.push_back(new_qq_pps);

                Matrix3f qq1 = qq_pps[qq_pps.size() - 1].block(0,0,3,3);
                Matrix3f qq2 = qq_pps[qq_pps.size() - 1].block(3,0,3,3);

//                std::cerr << qq_pps[qq_pps.size() - 1] << std::endl;
//                std::cerr << std::endl << std::endl;
//                std::cerr << qq1 << std::endl << std::endl;
//                std::cerr << qq2 << std::endl;

                if (qq1.determinant() * qq2.determinant() < 0) {
                    kmah_index += 1;
                } else {
                    float value = 0;
                    Matrix3f inverse_qq2 = qq2.inverse();
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            value += qq1(i, j) * inverse_qq2(i, j);
                        }
                    }
                    if (value * qq1.determinant() * qq2.determinant() < 0) {
                        kmah_index += 2;
                    }
                }

//                std::cerr << "new_qq_pps " << std::endl;
//                std::cerr << new_qq_pps << std::endl << std::endl;
            }

//            std::cerr << qq_pps[qq_pps.size() - 1] << std::endl << std::endl;
            Matrix3f qq = qq_pps[qq_pps.size() - 1].block(0,3, 3, 3);

            std::vector<float> disconts;
            for (int i = 0; i < out_phis.size(); i++) {
                discont_factor *= abs(out_phis[i].trace() / inc_phis[i].trace());
            }
            discont_factor = sqrt(discont_factor);
        } else {
            auto dists = getDistance();
            auto vels = getVels(velocity_model->getLayersCount() - 1);

            float dist0 = dists[0];
            float vels0 = vels[0];

            MatrixXf qq_pps1 = MatrixXf::Zero(6,6);

            qq_pps1.block(0, 0, 3, 3) = qq0;
            qq_pps1.block(3,0,3,3) = pp0;
            qq_pps1.block(0,3,3,3) = qq0 + vels0 * dist0 * pp0;
            qq_pps1.block(3,3,3,3) = pp0;
            qq_pps.push_back(qq_pps1);
        }

        Matrix3f qq = qq_pps[qq_pps.size() - 1].block(0,3, 3, 3);
        float ray_jac = qq.determinant();
        float spread = sqrt(abs(ray_jac));

        std::cerr << "spread:"  << spread << std::endl;

        return Ray::SpreadingProps(discont_factor, kmah_index, spread);
    }

    std::vector<float> Ray::getDistance() {
        std::vector<float> dists;
        dists.reserve(trajectory.size() - 1);
        for (int i = 0; i < trajectory.size() - 1; i++) {
            dists.emplace_back(sqrt(pow(trajectory[i+1][0] - trajectory[i][0], 2) +
                    pow(trajectory[i+1][1] - trajectory[i][1], 2) +
                    pow(trajectory[i+1][2] - trajectory[i][2], 2)));

        }
        return dists;
    }

} // namespace ray_tracing
