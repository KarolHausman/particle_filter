#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <Eigen/Core>


class Particle
{
public:
    Particle();
    virtual ~Particle();
    double weight;
    bool operator < (const Particle& p) const { return this->weight < p.weight;}
    Eigen::VectorXd state;
};

#endif // PARTICLE_H
