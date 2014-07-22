#ifndef FREE_MODEL_H
#define FREE_MODEL_H

#include "particle_filter/articulation_model.h"

class FreeModel : public ArticulationModel
{
public:
  FreeModel();
  virtual ~FreeModel();
  virtual ArticulationModelPtr getCopy()
  {
   return static_cast<ArticulationModelPtr>(new FreeModel(*this));
  }

protected:
};

#endif //FREE_MODEL_H
