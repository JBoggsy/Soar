#include <random>
#include <math.h>
#include <stdio.h>
#include "latent_representation.h"


void latent_representation::copy_from(latent_representation* other) {
    if (_mu != NULL) delete _mu;
    if (_sigma != NULL) delete _sigma;

    _mu = new std::vector<double>(*other->_mu);
    _sigma = new std::vector<double>(*other->_sigma);
}

std::vector<double>* latent_representation::sample(std::vector<double>* sample) {
    std::random_device rd;
    std::mt19937 rand_engine(rd());
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int i = 0; i < _mu->size(); i++) {
        double stddev = exp(0.5 * _sigma->at(i));
        double dim_sample = _mu->at(i) + (stddev * distribution(rand_engine));
        sample->push_back(dim_sample);
    }
    return sample;
}

void latent_representation::print_latent() {
    printf("Latent:\n");
    for (int i = 0; i < _mu->size(); i++) {
        printf("  %f (%f)\n", _mu->at(i), _sigma->at(i));
    }
}
