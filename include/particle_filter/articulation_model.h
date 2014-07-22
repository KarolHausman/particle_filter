#ifndef ARTICULATION_MODEL_H
#define ARTICULATION_MODEL_H

#include <iostream>
#include "boost/shared_ptr.hpp"

class ArticulationModel;
typedef boost::shared_ptr<ArticulationModel> ArticulationModelPtr;
enum Model { RIGID, FREE, PRISMATIC, ROTATIONAL, MODELS_NUMBER };

class ArticulationModel
{
public:
  ArticulationModel();
  virtual ~ArticulationModel();
  virtual ArticulationModelPtr getCopy()
  {
     return ArticulationModelPtr(new ArticulationModel(*this));
  }

  Model model;
protected:
};



#endif // ARTICULATION_MODEL_H
