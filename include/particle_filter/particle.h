#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

class Particle
{
public:
    Particle();
    virtual ~Particle();
    double weight;
    bool operator < (const Particle& p) const { return this->weight < p.weight;}
    std::vector <double> state;
};

#endif // PARTICLE_H
