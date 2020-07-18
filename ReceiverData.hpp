#ifndef PROCESSING_RECEIVERDATA_HPP
#define PROCESSING_RECEIVERDATA_HPP


#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

namespace processing {
    class ReceiverData {

        unsigned int samples_number;
        unsigned int components_number;

    public:
        vnl_matrix<float> data;

        ReceiverData(unsigned int samples_number, unsigned int components_number, const float *values);

        void rotate(const vnl_matrix<float> & matrix);

        void detrend();

        void changePolarity();

        unsigned int getComponentsNumber() const;

        vnl_vector<float> getTrace(unsigned i) const;

        void setTrace(unsigned i, vnl_vector<float> trace);

        unsigned int getSamplesNumber() const;
    };
}

#endif //PROCESSING_RECEIVERDATA_HPP
