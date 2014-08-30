#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <Eigen/Core>
#include "particle_filter/articulation_model.h"


template <class StateType> class Particle
{
public:
    Particle();
    Particle(const Particle<ArticulationModelPtr>& p2);
    Particle<ArticulationModelPtr>& operator = (const Particle<ArticulationModelPtr>& p2);
    virtual ~Particle();

    bool operator < (const Particle& p) const
    {
      return this->weight < p.weight;
    }
    friend std::ostream& operator << (std::ostream& stream, const Particle& p)
    {
      stream << "weight= " << p.weight << "\n" << "weight_to_print_only= " << p.weight_to_print_only << "\n" << "state= \n" << p.state << "\n";
      return stream;
    }

    double weight;
    double expected_weight;
    double weight_to_print_only;
    StateType state;
protected:
};

#endif // PARTICLE_H
