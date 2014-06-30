

#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"

int main(int argc, char **argv)
{
  Particle p;
  std::vector <Particle> v_p;
  v_p.push_back(p);
  ParticleFilter pf(v_p);

}
