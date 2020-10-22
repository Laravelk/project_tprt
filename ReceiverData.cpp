#include "ReceiverData.hpp"

#include <iostream>

namespace processing {
ReceiverData::ReceiverData(unsigned int samples_number,
                           unsigned int components_number, const float *values)
    : samples_number(samples_number), components_number(components_number) {
  data = vnl_matrix<float>(values, components_number, samples_number);
}

void ReceiverData::rotate(const vnl_matrix<float> &matrix) {
  data.inplace_transpose();
  data *= matrix;
  data.inplace_transpose();
}

void ReceiverData::detrend() {
  for (unsigned int i = 0; i < components_number; i++) {
    data.set_row(i, data.get_row(i) - data.get_row(i).mean());
  }
}

void ReceiverData::changePolarity() { data *= -1; }

unsigned int ReceiverData::getSamplesNumber() const { return samples_number; }

unsigned int ReceiverData::getComponentsNumber() const {
  return components_number;
}

void ReceiverData::setTrace(unsigned i, vnl_vector<float> trace) {
  data.set_row(i, trace);
}

vnl_vector<float> ReceiverData::getTrace(unsigned i) const {
  return data.get_row(i);
}

} // namespace processing
