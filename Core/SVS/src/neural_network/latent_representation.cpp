#include <stdlib.h>
#include "latent_representation.h"


void latent_representation::copy_from(latent_representation* other) {
    delete _mu;
    delete _sigma;

    _mu = other->_mu;
    _sigma = other->_sigma;
}

std::vector<double>* latent_representation::sample(std::vector<double>* sample) {
    for (int i = 0; i < _mu->size(); i++) {
        sample->push_back(_mu->at(i) + (_sigma->at(i) * ((double)rand() / RAND_MAX)));
    }
    return sample;
}
