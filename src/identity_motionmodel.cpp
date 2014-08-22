#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/free_model.h"

template <class StateType> IdentityMotionModel<StateType>::IdentityMotionModel()
{
}

template <class StateType> IdentityMotionModel<StateType>::~IdentityMotionModel()
{
}

template <class StateType> StateType IdentityMotionModel<StateType>::move(const StateType &state, const Eigen::VectorXd &controls,
                          const Eigen::VectorXd &noise) const
{
  return state + noise;
}

template <> ArticulationModelPtr IdentityMotionModel<ArticulationModelPtr>::move(const ArticulationModelPtr &state, const Eigen::VectorXd &controls,
                                                                                 const Eigen::VectorXd &noise) const

{
  ArticulationModelPtr state_result;

  switch (state->model)
  {
    case (RIGID):
      {
        state_result.reset(new RigidModel);
        state_result = state->getCopy();

        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (state_result);
        rigid->pos_x += noise(0);
        rigid->pos_y += noise(1);
        rigid->pos_z += noise(2);
        rigid->roll += noise(3);
        rigid->pitch += noise(4);
        rigid->yaw += noise(5);
        state_result = static_cast<ArticulationModelPtr> (rigid);
        break;
      }
    case (PRISMATIC):
      {
        state_result.reset(new PrismaticModel);
        state_result = state->getCopy();

        boost::shared_ptr<PrismaticModel> prismatic = boost::dynamic_pointer_cast< PrismaticModel > (state_result);
        prismatic->pos_x += noise(0);
        prismatic->pos_y += noise(1);
        prismatic->pos_z += noise(2);
        prismatic->roll += noise(3);
        prismatic->pitch += noise(4);
        prismatic->yaw += noise(5);
        prismatic->axis_x += noise(6);
        prismatic->axis_y += noise(7);
        prismatic->axis_z += noise(8);
        state_result = static_cast<ArticulationModelPtr> (prismatic);
        break;
      }
    case (ROTATIONAL):
      {
        state_result.reset(new RotationalModel);
        state_result = state->getCopy();

        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (state_result);
        rotational->rot_center_x += noise(0);
        rotational->rot_center_y += noise(1);
        rotational->rot_center_z += noise(2);
        rotational->roll += noise(3);
        rotational->pitch += noise(4);
        rotational->yaw += noise(5);
        rotational->radius += noise(6);
        rotational->axis_roll += noise(7);
        rotational->axis_pitch += noise(8);
        rotational->axis_yaw += noise(9);
        state_result = static_cast<ArticulationModelPtr> (rotational);
        break;
      }
    case (FREE):
      {
        state_result.reset(new FreeModel);
        state_result = state->getCopy();
      }
  }
  state_result->writeParamsToModel();
  return state_result;
}

template class IdentityMotionModel<Eigen::VectorXd>;
template class IdentityMotionModel<ArticulationModelPtr>;

