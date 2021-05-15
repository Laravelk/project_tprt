#pragma once

#include <segyio/segy.h>
#include <array>
#include <string>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <fstream>
#include <utility>

class ModelTranformation {
public:

    ModelTranformation(std::vector<std::string> &layer_files_paths, std::string sgy_file_path) {
        this->layer_files_paths = layer_files_paths;
        this->sgy_file_path = std::move(sgy_file_path);
    }

    std::vector<double> layer_speed_from_sgy() {
        std::vector<_2D::BicubicInterpolator<float>> interpolators_vector;

        float first_x, first_y, max_z;
        for (int i = 0; i < layer_files_paths.size(); i++) {
            std::ifstream input_stream(layer_files_paths[i]);
            _2D::BicubicInterpolator<float> interpolator;
            std::vector<std::array<float, 3>> value_vectors;

            while (!input_stream.eof()) {
                float x, y, z;
                input_stream >> x >> y >> z;
                std::array<float, 3> array = { x, y, z };
                value_vectors.push_back(array);
            }

            _2D::BicubicInterpolator<float>::VectorType
            xx(value_vectors.size()),
            yy(value_vectors.size()),
            zz(value_vectors.size());

            for (int j = 0; j < value_vectors.size(); i++) {
                xx(j) = value_vectors[j][0];
                yy(j) = value_vectors[j][1];
                zz(j) = value_vectors[j][2];
            }

            first_x = xx(0);
            first_y = yy(0);

            interpolator.setData(xx, yy, zz);
            interpolators_vector.push_back(interpolator);
        }

        float z = 0;
        for (auto & interpolator : interpolators_vector) {
            float new_z = interpolator(first_x, first_y);
            if (new_z > z) {
                z = new_z;
            } else {
                throw std::runtime_error("ModelTransformation::layer_speed_sgy(Incorrect layers)");
            }
        }

        segy_file *fp = segy_open(sgy_file_path.c_str(), "rb");

        if (!fp) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not open sgy file)");
        }

        char binheader[SEGY_BINARY_HEADER_SIZE];

        int error = segy_binheader(fp, binheader);
        if (SEGY_OK != error) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not create sgy binheader)");
        }

        float _sam_intr{0};
        int _sam_num{0};
        int _format{0};
        long _trace0{0};
        int _trace_bsize{0};
        int _trace_num{0};

        float fallback = -1.0;
        error = segy_sample_interval(fp, fallback, &_sam_intr);
        if (SEGY_OK != error) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not get sgy sample interval");
        }

        _sam_num = segy_samples(binheader);
        if (0 >= _sam_num) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(sgy samples returned negative value)");
        }

        _format = segy_format(binheader);
        _trace0 = segy_trace0(binheader);
        _trace_bsize = segy_trsize(_format, _sam_num);

        if (0 >= _trace_bsize) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(sgy trsize is negative)");
        }

        error = segy_traces(fp, &_trace_num, _trace0, _trace_bsize);
        if (SEGY_OK != error) {
            throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not get sgy traces");
        }

        int _alreadyRead = 0;
        int err = 0;

        uint32_t buffer_size = static_cast<uint32_t>(_trace_bsize) /
                                           sizeof(float);

        std::vector<int> count_of_elements_in_layer;
        std::vector<float> sum_of_samples_in_layer;

        for (int i = 0; i < interpolators_vector.size(); i++) {
            count_of_elements_in_layer.push_back(0);
            sum_of_samples_in_layer.push_back(0);
        }

        char traceh[SEGY_TRACE_HEADER_SIZE];

        for (int i = 0; i < _trace_num; ++i) {
            std::cerr << i << std::endl;
            if (_trace_num == _alreadyRead) {
                throw std::runtime_error("ModelTransformation::layer_speed_sgy(No more traces in the sgy-file)");
            }

            err = segy_traceheader(fp, _alreadyRead, traceh, _trace0, _trace_bsize);
            if (SEGY_OK != err) {
                std::cerr << err << std::endl;
                throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not get sgy traceheader in loop)");
            }

            float *buffer = new float[buffer_size];
            err = segy_readtrace(fp, _alreadyRead, buffer, _trace0, _trace_bsize);
            if (SEGY_OK != err) {
                throw std::runtime_error("ModelTransformation::layer_speed_sgy(Can not read a trace)");
            }

            int p_wave_arrival_trace;
            err = segy_get_field(traceh, SEGY_TR_CDP_X, &p_wave_arrival_trace);

            int s_wave_arrival_trace;
            err = segy_get_field(traceh, SEGY_TR_CDP_Y, &s_wave_arrival_trace);

            segy_to_native(_format, _sam_num, buffer);

            err = segy_samples(binheader);

            std::vector<int> horizon_indexs;
            for (auto & interpolate : interpolators_vector) {
                horizon_indexs.push_back(interpolate((float) s_wave_arrival_trace, (float) p_wave_arrival_trace) / _sam_intr * 1000);
            }

            int first_index = 0;
            for (int j = 0; j < interpolators_vector.size(); j++) {
                int second_index = horizon_indexs[j];
                float sum = 0;

                for (int k = first_index; k < second_index; k++) {
                    sum += buffer[k];
                }

                sum_of_samples_in_layer[j] += sum;
                count_of_elements_in_layer[j] += second_index - first_index;
                first_index = second_index;
            }

            delete[] buffer;

            ++_alreadyRead;
        }

        std::vector<double> speeds;

        for (int i = 0; i < interpolators_vector.size(); i++) {
            double some = sum_of_samples_in_layer[i] / count_of_elements_in_layer[i];
            speeds.push_back(some);
        }

        return speeds;
    }

private:
    std::vector<std::string> layer_files_paths;
    std::string sgy_file_path;
};