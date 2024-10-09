#pragma once
// c++ includes
#include <vector>
// SVS includes
#include "mat.h"

/**
 * @brief Represents a latent representation of an image.
 *
 * This class is used to store the latent representation of an image as a pair of
 * vectors, one for the mean and one for the standard deviation. It also provides
 * a method for sampling from the distribution defined by these two vectors.
 *
 * @note Sampling should be done only as-needed; the latent representation is
 * explicitly meant to be a distribution, not a single value.
 */
class latent_representation
{
private:
    std::vector<double>* _mu;
    std::vector<double>* _sigma;
    cvec _mu_vec;
    cvec _sigma_vec;

public:
    /**
     * @brief Construct a new latent representation object.
     *
     * @param mu The mean vector of the latent representation.
     * @param sigma The standard deviation vector of the latent representation.
     *
     * @note The `latent_representation` object takes ownership of the `mu` and
     * `sigma` vectors, so the caller should not delete them.
     */
    latent_representation();
    latent_representation(std::vector<double>* mu, std::vector<double>* sigma);
    ~latent_representation() {
        delete _mu;
        delete _sigma;
    }

    void copy_from(latent_representation* other);

    size_t get_size() const { return _mu->size(); }
    std::vector<double>* get_mu() { return _mu; }
    std::vector<double>* get_sigma() { return _sigma; }
    void set_mu(std::vector<double>* mu);
    void set_sigma(std::vector<double>* sigma);

    double wasser_distance(latent_representation* other);
    std::vector<double>* sample(std::vector<double>* sample);

    void print_latent();

};
