#ifndef ARTICULATION_MODEL_H
#define ARTICULATION_MODEL_H

#include <iostream>
#include "boost/shared_ptr.hpp"

class ArticulationModel;
typedef boost::shared_ptr<ArticulationModel> ArticulationModelPtr;
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
  friend std::ostream& operator << (std::ostream& stream, const ArticulationModelPtr& amptr);

  Model model;
protected:
};



#endif // ARTICULATION_MODEL_H
