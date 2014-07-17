#ifndef ARTICULATION_MODEL_H
#define ARTICULATION_MODEL_H

enum Model{ RIGID, FREE, PRISMATIC, ROTATIONAL};
class ArticulationModel
{
public:
  ArticulationModel();
  virtual ~ArticulationModel();

protected:
  Model model;
};


#endif // ARTICULATION_MODEL_H
