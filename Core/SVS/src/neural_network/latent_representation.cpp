#include <random>
#include <math.h>
#include <stdio.h>
#include "latent_representation.h"


latent_representation::latent_representation() {
    _mu = NULL;
    _sigma = NULL;

    _mu_vec = *(new cvec());
    _sigma_vec = *(new cvec());
}

latent_representation::latent_representation(std::vector<double>* mu, std::vector<double>* sigma) {
    _mu = new std::vector<double>(*mu);
    _sigma = new std::vector<double>(*sigma);

    _mu_vec = *(new cvec(mu->size()));
    _sigma_vec = *(new cvec(sigma->size()));

    for (int i = 0; i < mu->size(); i++) {
        _mu_vec[i] = mu->at(i);
        _sigma_vec[i] = sigma->at(i);
    }
}

void latent_representation::copy_from(latent_representation* other) {
    if (_mu != NULL) delete _mu;
    if (_sigma != NULL) delete _sigma;

    _mu = new std::vector<double>(*other->_mu);
    _sigma = new std::vector<double>(*other->_sigma);

    _mu_vec.resize(_mu->size());
    _sigma_vec.resize(_sigma->size());

    for (int i = 0; i < _mu->size(); i++) {
        _mu_vec[i] = _mu->at(i);
        _sigma_vec[i] = _sigma->at(i);
    }
}

void latent_representation::set_mu(std::vector<double>* mu) {
    if (_mu != NULL) delete _mu;
    _mu = new std::vector<double>(*mu);

    _mu_vec.resize(_mu->size());
    for (int i = 0; i < _mu->size(); i++) {
        _mu_vec[i] = _mu->at(i);
    }
}

void latent_representation::set_sigma(std::vector<double>* sigma) {
    if (_sigma != NULL) delete _sigma;
    _sigma = new std::vector<double>(*sigma);

    _sigma_vec.resize(_sigma->size());
    for (int i = 0; i < _sigma->size(); i++) {
        _sigma_vec[i] = _sigma->at(i);
    }
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

double latent_representation::wasser_distance(latent_representation* other) {
    cvec mean_diff = _mu_vec - other->_mu_vec;
    double mean_diff_term = pow(mean_diff.norm(), 2);

    cvec var_1 = 0.5 * _sigma_vec;
    var_1 = var_1.array().exp();
    cvec var_2 = 0.5 * other->_sigma_vec;
    var_2 = var_2.array().exp();
    cvec root_var_prod = var_1.cwiseProduct(var_2);
    root_var_prod = root_var_prod.array().sqrt();
    cvec var_term = var_1 + var_2 - (2 * root_var_prod);
    double var_term_sum = var_term.sum();

    return sqrt(mean_diff_term + var_term_sum);
}

void latent_representation::print_latent() {
    printf("Latent:\n");
    for (int i = 0; i < _mu->size(); i++) {
        printf("  %f (%f)\n", _mu->at(i), _sigma->at(i));
    }
}
