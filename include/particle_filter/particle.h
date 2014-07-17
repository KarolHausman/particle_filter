#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <Eigen/Core>


template <class StateType> class Particle
{
public:
    Particle();
    virtual ~Particle();
    double weight;
    StateType state;
    bool operator < (const Particle& p) const { return this->weight < p.weight;}
    friend std::ostream& operator << (std::ostream& stream, const Particle& p)
    {
      stream << "weight= " << p.weight << "\n" << "state= \n" << p.state << "\n";
      return stream;
    }
};

#endif // PARTICLE_H
