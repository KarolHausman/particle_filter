#ifndef ARTICULATION_MODEL_H
#define ARTICULATION_MODEL_H

#include <iostream>
#include "boost/shared_ptr.hpp"


enum Model{ RIGID, FREE, PRISMATIC, ROTATIONAL, MODELS_NUMBER};
class ArticulationModel
{
public:
  ArticulationModel();
  virtual ~ArticulationModel();
  friend std::ostream& operator << (std::ostream& stream, const ArticulationModel& am)
  {
    stream << "model= " << am.model << "\n";
    return stream;
  }

protected:
  Model model;
};

typedef boost::shared_ptr<ArticulationModel> ArticulationModelPtr;

#endif // ARTICULATION_MODEL_H
