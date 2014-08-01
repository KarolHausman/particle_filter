#include "particle_filter/articulation_model.h"
#include <limits>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_blas.h>

ArticulationModel::ArticulationModel()
{
//  TODO: not needed yet, setId(-1);
  sigma_position = 0.005;
  sigma_orientation = 360 * M_PI/180.0;

  avg_error_position = 0;
  avg_error_orientation = 0;
  bic = 0;

  last_error_jacobian = 0;
  evaluated = false;
  supress_similar = true;
  outlier_ratio = 0.5;
  sac_iterations = 300;
//  optimizer_iterations = 10;
  optimizer_iterations = 0;

  prior_outlier_ratio = log(0.01) / (- 0.05); // prior over outlier ratio

  complexity = 0;
  jacobian = Eigen::MatrixXd();
  hessian = Eigen::MatrixXd();
}

ArticulationModel::~ArticulationModel()
{}

void ArticulationModel::setTrack(const articulation_model_msgs::TrackMsg& track)
{
  this->model_msg.track = track;
}

void ArticulationModel::addTrack(const articulation_model_msgs::TrackMsg& track)
{
  this->model_msg.track.pose.insert(this->model_msg.track.pose.end(), track.pose.begin(), track.pose.end());
  prepareChannels();
}



void ArticulationModel::readParamsFromModel()
{
  getParam("sigma_position",sigma_position);
  getParam("sigma_orientation",sigma_orientation);
  getParam("supress_similar",supress_similar);
  getParam("avg_error_position",avg_error_position);
  getParam("avg_error_orientation",avg_error_orientation);
  getParam("bic",bic);
  getParam("last_error_jacobian",last_error_jacobian);
  getParam("evaluated",evaluated);
  //	getParam("complexity",complexity);//read-only
  getParam("jacobian",jacobian);
  getParam("hessian",hessian);
  getParam("loglikelihood",loglikelihood);
  getParam("outlier_ratio",outlier_ratio);
  getParam("sac_iterations",sac_iterations);
  getParam("prior_outlier_ratio",prior_outlier_ratio);
}

void ArticulationModel::writeParamsToModel()
{
  setParam("sigma_position",sigma_position,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("sigma_orientation",sigma_orientation,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("supress_similar",supress_similar,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("avg_error_position",avg_error_position,articulation_model_msgs::ParamMsg::EVAL);
  setParam("avg_error_orientation",avg_error_orientation,articulation_model_msgs::ParamMsg::EVAL);
  setParam("loglikelihood",loglikelihood,articulation_model_msgs::ParamMsg::EVAL);
  setParam("bic",bic,articulation_model_msgs::ParamMsg::EVAL);
  setParam("last_error_jacobian",last_error_jacobian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("evaluated",evaluated,articulation_model_msgs::ParamMsg::EVAL);
  setParam("complexity",complexity,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("jacobian",jacobian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("hessian",hessian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("dofs",getDOFs(),articulation_model_msgs::ParamMsg::EVAL);
  setParam("samples",getSamples(),articulation_model_msgs::ParamMsg::EVAL);
  setParam("outlier_ratio",outlier_ratio,articulation_model_msgs::ParamMsg::EVAL);
  setParam("sac_iterations",sac_iterations,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("prior_outlier_ratio",prior_outlier_ratio,articulation_model_msgs::ParamMsg::PRIOR);
}

double ArticulationModel::getParam(std::string name)
{
  for(size_t i = 0; i < model_msg.params.size(); i++)
  {
    if(model_msg.params[i].name == name)
    {
      return(model_msg.params[i].value);
    }
  }
//  std::cerr << "WARNING (in ArticulationModel::getParam): undefined parameter '"<<name<<"'" << std::endl;
  return 0.00;
}

void ArticulationModel::getParam(std::string name,double& value)
{
  if(hasParam(name))
    value = getParam(name);
}

void ArticulationModel::getParam(std::string name,Eigen::VectorXd &vec)
{
  for(int i=0;i<vec.rows();i++)
    vec(i) = getParam( str(boost::format( name+"[%1%]") %i ) );
}

void ArticulationModel::getParam(std::string name,tf::Vector3 &vec)
{
  vec.setX( getParam( name+".x" ) );
  vec.setY( getParam( name+".y" ) );
  vec.setZ( getParam( name+".z" ) );
}

void ArticulationModel::getParam(std::string name,tf::Quaternion &quat)
{
  quat.setX( getParam( name+".x" ) );
  quat.setY( getParam( name+".y" ) );
  quat.setZ( getParam( name+".z" ) );
  quat.setW( getParam( name+".w" ) );
}

void ArticulationModel::getParam(std::string name,tf::Transform &t)
{
  tf::Quaternion q;
  tf::Vector3 v;
  getParam(name+".orientation",q);
  getParam(name+".position",v);
  t = tf::Transform(q,v);
}

void ArticulationModel::getParam(std::string name,Eigen::MatrixXd &mat)
{
  for(int r=0;r<mat.rows();r++)
    for(int c=0;c<mat.cols();c++)
      mat(r,c) = getParam( str(boost::format( name+"[%1%][%2%]") %r % c) );
}

void ArticulationModel::setParam(std::string name,double value,int type)
{
  for(size_t i=0;i<model_msg.params.size();i++)
  {
    if(model_msg.params[i].name == name)
    {
      model_msg.params[i].value = value;
      return;
    }
  }
  articulation_model_msgs::ParamMsg data;
  data.name = name;
  data.value = value;
  data.type = type;
  model_msg.params.push_back(data);
}

void ArticulationModel::setParam(std::string name,const Eigen::VectorXd &vec,int type)
{
  for(int i=0;i<vec.rows();i++)
    setParam( boost::str(boost::format( name+"[%1%]") %i ),vec(i),type );
}

void ArticulationModel::setParam(std::string name,const tf::Vector3 &vec,int type)
{
  setParam( name+".x",vec.x(),type );
  setParam( name+".y",vec.y(),type );
  setParam( name+".z",vec.z(),type );
}

void ArticulationModel::setParam(std::string name,const tf::Quaternion &quat,int type)
{
  setParam( name+".x",quat.x(),type );
  setParam( name+".y",quat.y(),type );
  setParam( name+".z",quat.z(),type );
  setParam( name+".w",quat.w(),type );
}

void ArticulationModel::setParam(std::string name,const tf::Transform &t,int type)
{
  setParam(name+".position",t.getOrigin(),type);
  setParam(name+".orientation",t.getRotation(),type);
}

void ArticulationModel::setParam(std::string name,const Eigen::MatrixXd &mat,int type)
{
  for(int r=0;r<mat.rows();r++)
    for(int c=0;c<mat.cols();c++)
      setParam( str(boost::format( name+"[%1%][%2%]") %r%c ),mat(r,c),type );
}

bool ArticulationModel::hasParam(std::string name)
{
  for(size_t i=0;i<model_msg.params.size();i++)
  {
    if(model_msg.params[i].name == name)
      return(true);
  }
  return false;
}

void ArticulationModel::setModel(const articulation_model_msgs::ModelMsg& model_m)
{
  this->model_msg = model_m;
  readParamsFromModel();
  prepareChannels();
}

articulation_model_msgs::ModelMsg ArticulationModel::getModel()
{
  writeParamsToModel();
  return this->model_msg;
}


bool ArticulationModel::sampleConsensus()
{
  // sample consensus
  writeParamsToModel();
  std::vector<articulation_model_msgs::ParamMsg> bestParams = model_msg.params;
  double bestLikelihood = -std::numeric_limits<double>::max();
  bool goodGuess = false;
  for(size_t it=0; it < sac_iterations; it++)
  {
    if(!guessParameters()) continue;
    goodGuess = true;
    double likelihood = getLogLikelihood(true);
//    cout <<"RANSAC iteration="<<it<<", likelihood="<<likelihood<<endl;
    if(bestLikelihood < likelihood)
    {
      writeParamsToModel();
      bestParams = model_msg.params;
      bestLikelihood = likelihood;
    }
  }
//	cout <<"RANSAC best likelihood="<<bestLikelihood<<endl;
  model_msg.params = bestParams;
  readParamsFromModel();

  return goodGuess;
}

bool ArticulationModel::guessParameters()
{
  // sample a minimum number of samples to compute parameters
  return false;
}

size_t ArticulationModel::getSamples()
{
  return model_msg.track.pose.size();
}

V_Configuration ArticulationModel::predictConfiguration(geometry_msgs::Pose pose)
{
  V_Configuration q;
  if(getDOFs()>0) q.resize(getDOFs());
  return q;
}

geometry_msgs::Pose ArticulationModel::predictPose(V_Configuration q)
{
  geometry_msgs::Pose p;
  p.orientation.w = 1;
  return p;
}

double ArticulationModel::getInlierLogLikelihood( size_t index )
{
  geometry_msgs::Pose &pose_obs = model_msg.track.pose[index];
  V_Configuration q_estimated = predictConfiguration(pose_obs);
  for(size_t i = 0; i < (size_t)q_estimated.rows(); i++)
  {
    model_msg.track.channels[channelConfiguration[i]].values[index] =  q_estimated[i];
  }

  geometry_msgs::Pose pose_estimated = predictPose(q_estimated);
  model_msg.track.pose_projected[index] = pose_estimated;

  tf::Transform pose_obs_tf, pose_estimated_tf;
  tf::poseMsgToTF(pose_obs, pose_obs_tf);
  tf::poseMsgToTF(pose_estimated, pose_estimated_tf);

  tf::Transform diff = pose_obs_tf.inverseTimes(pose_estimated_tf);//poseToTransform(pose_obs).inverseTimes( poseToTransform(pose_estimated) );
  double err_position = diff.getOrigin().length();
  double err_orientation = fabs(diff.getRotation().getAngle());

  double loglikelihood =
                  - log(2*M_PI * sigma_position*sigma_orientation)
                  - 0.5*(
                                  ( (err_position*err_position) / (sigma_position*sigma_position) ) +
                                  ( (err_orientation*err_orientation) / (sigma_orientation*sigma_orientation)) );	// 2-dim multivariate normal distribution
  return loglikelihood;
}

double ArticulationModel::getOutlierLogLikelihood()
{
  // outside the 95% ellipsoid
  double chi2inv = 1.96;
  //	chi2inv = 6.63;	//99% ellipsoid

  double err_position = chi2inv * sigma_position;
  double err_orientation = chi2inv * sigma_orientation;

  double loglikelihood =
                  - log(2*M_PI * sigma_position*sigma_orientation)
                  - 0.5*(
                                  ( (err_position*err_position) / (sigma_position*sigma_position) ) +
                                  ( (err_orientation*err_orientation) / (sigma_orientation*sigma_orientation)) );	// 2-dim multivariate normal distribution

  return( loglikelihood );
}

double ArticulationModel::getLogLikelihoodForPoseIndex(size_t index) {
//  double gamma_mixing = 0.1;
//  return (1-gamma_mixing)*getInlierLogLikelihood(pose_obs) + gamma_mixing * getOutlierLogLikelihood();
    double inlierLikelihood = getInlierLogLikelihood(index);
    double outlierLikelihood = getOutlierLogLikelihood();

//  mle-sac
    model_msg.track.channels[channelInlierLogLikelihood].values[index] = inlierLikelihood;
    double pi = (1-outlier_ratio) * exp( inlierLikelihood );
    double po = outlier_ratio * exp( outlierLikelihood );
    model_msg.track.channels[channelOutlier].values[index] =  po / (pi+po);
    double p = (log(exp(inlierLikelihood) + exp(outlierLikelihood)));
    model_msg.track.channels[channelLogLikelihood].values[index] =  p;
    return p;

//  // ransac
//  return ((1-gamma_mixing)*inlierLikelihood + (gamma_mixing)*outlierLikelihood);

//  // m-sac
//  return (max(inlierLikelihood,outlierLikelihood));
}


double ArticulationModel::getLogLikelihood(bool estimate_outlier_ratio)
{
  model_msg.track.pose_projected.resize(model_msg.track.pose.size());


  model_msg.track.channels[channelInlierLogLikelihood].values.resize(model_msg.track.pose.size());
  model_msg.track.channels[channelOutlier].values.resize(model_msg.track.pose.size());
  model_msg.track.channels[channelLogLikelihood].values.resize(model_msg.track.pose.size());

  for(size_t i=0; i<(size_t)getDOFs(); i++)
  {
    model_msg.track.channels[channelConfiguration[i]].values.resize(model_msg.track.pose.size());
  }

  double sum_likelihood = 0;
  size_t n = getSamples();
  if(estimate_outlier_ratio)
  {
    outlier_ratio =0.5;
    for(size_t i=0;i<n;i++)
    {
      model_msg.track.channels[channelOutlier].values[i] = 0.5;	// initial estimate of gamma
    }

    int iter = 0;
    double diff = 0;
    do
    {
      sum_likelihood = 0;

      for(size_t i=0; i<n; i++)
      {
        sum_likelihood += getLogLikelihoodForPoseIndex(i);
      }

      double outlier_ratio_new = 0;
      for(size_t i=0;i<n;i++) {
              outlier_ratio_new += model_msg.track.channels[channelOutlier].values[i]/n;
      }
      diff = fabs(outlier_ratio - outlier_ratio_new);
      iter++;
      outlier_ratio = outlier_ratio_new;
  //  cout <<"EM iter="<<iter<<" outlier_ratio="<<outlier_ratio<<endl;
    } while( diff > 0.01 && iter < 10 );
  } else
  {
    for(size_t i = 0; i < n; i++)
    {
      sum_likelihood += getLogLikelihoodForPoseIndex(i);
    }
  }
  sum_likelihood += - prior_outlier_ratio * outlier_ratio * n;
  return sum_likelihood;
}

void ArticulationModel::prepareChannels()
{
  channelOutlier = openChannel(model_msg.track, "outlier");
  channelLogLikelihood = openChannel(model_msg.track, "loglikelihood");
  channelInlierLogLikelihood = openChannel(model_msg.track, "loglikelihood_if_inlier");
  channelConfiguration.resize(getDOFs(),0);
  for(size_t i = 0; i < getDOFs(); i++)
  {
    channelConfiguration[i] = openChannel(model_msg.track, str(boost::format("q%d")%i));
  }

}

size_t ArticulationModel::getDOFs()
{
  return 0;
}

int ArticulationModel::openChannel(articulation_model_msgs::TrackMsg &track, std::string name, bool autocreate)
{
  // find channel
  size_t i = 0;
  for(; i < track.channels.size(); i++)
  {
    if(track.channels[i].name == name)
      break;
  }
  // found?
  if( i == track.channels.size() )
  {
    if(!autocreate)
      return -1;
    // create, if not found
    sensor_msgs::ChannelFloat32 ch;
    ch.name = name;
    track.channels.push_back(ch);
  }
  // ensure that channel has right number of elements
  track.channels[i].values.resize( track.pose.size() );
  // return channel number
  return i;
}




double my_f (const gsl_vector *v, void *params)
{
  ArticulationModel *p = (ArticulationModel *)params;

  std::vector<double> delta( v->size );
  for(size_t i=0;i<v->size;i++)
  {
    delta[i] = gsl_vector_get (v, i);
//  cout <<"delta["<<i<<"]="<<delta[i]<<" ";
  }

  p->model_msg.params = p->params_initial;
  p->readParamsFromModel();
  p->updateParameters(delta);
  p->writeParamsToModel();
  double likelihood = p->getLogLikelihood(true);
//  cout <<"likelihood="<<likelihood<<endl;
  return -likelihood;
}

/* The gradient of f, df = (df/dx, df/dy). */
void my_df (const gsl_vector *v, void *params,gsl_vector *df)
{
  double likelihood = my_f(v,params);

  gsl_vector * v_delta =gsl_vector_alloc (v->size);

  double DELTA = 1e-3;
  for(size_t i=0;i<v->size;i++)
  {
    v_delta = gsl_vector_alloc (v->size);
    gsl_vector_memcpy(v_delta,v);
    gsl_vector_set(v_delta, i,gsl_vector_get(v, i)+DELTA);

    double likelihood_delta = my_f(v_delta,params);
    gsl_vector_set(df, i,(likelihood_delta-likelihood)/DELTA);
  }
  gsl_vector_free (v_delta);
}

/* Compute both f and df together. */
void my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
  *f = my_f(x, params);
  my_df(x, params, df);
}


bool ArticulationModel::optimizeParameters()
{
  // using gsl minimizer
  // dofs to optimize: complexity
  // stores current parameter vector as params_initial
  writeParamsToModel();
  params_initial = model_msg.params;
  // calls updateParameters(..) to set new params vector

  size_t iter = 0;
  int status;

  const gsl_multimin_fdfminimizer_type *T;
  gsl_multimin_fdfminimizer *s;

  gsl_vector *x;
  gsl_multimin_function_fdf my_func;

  my_func.n = (int)complexity;
  my_func.f = my_f;
  my_func.df = my_df;
  my_func.fdf = my_fdf;
  my_func.params = this;

  x = gsl_vector_alloc ((int)complexity);
  gsl_vector_set_zero (x);
//    double likelihood_initial= -my_f(x,this);

  T = gsl_multimin_fdfminimizer_vector_bfgs2;
  s = gsl_multimin_fdfminimizer_alloc (T, (int)complexity);

  gsl_multimin_fdfminimizer_set (s, &my_func, x, 0.01, 0.1);

//    cout <<"optimizing "<<complexity<<" DOFs"<<endl;

//    for(size_t i=0;i<model.params.size();i++) {
//    	if(model.params[i].type != ParamMsg::PARAM) continue;
//    	cout <<" param "<<model.params[i].name<<"="<<model.params[i].value<<endl;
//    }

  if(optimizer_iterations > 0) do
    {
      iter++;
//        cout <<"iter "<<iter<<".."<<endl;
      status = gsl_multimin_fdfminimizer_iterate (s);

      if (status)
        break;

      status = gsl_multimin_test_gradient (s->gradient, 1e-1);

//        if (status == GSL_SUCCESS)
//          printf ("Minimum found at:\n");

//        printf ("i=%5d ", (int)iter);
//        for(size_t i=0;i<s->x->size;i++) {
//            printf ("%5f ", gsl_vector_get(s->x,i));
//        }
//        printf("\n");
    } while (status == GSL_CONTINUE && iter < optimizer_iterations);

//    double likelihood_final = -my_f(s->x,this);
//    cout <<"likelihood_initial="<<likelihood_initial<<" ";
//    cout <<"likelihood_final="<<likelihood_final<<" ";
//    cout <<"after "<<iter<<" iterations";
//    cout << endl;

  gsl_multimin_fdfminimizer_free (s);
  gsl_vector_free (x);

  if(!fitMinMaxConfigurations())
    return false;

//	cout <<"type="<<getModelName()<<endl;
//    for(size_t i=0;i<model.params.size();i++) {
//    	if(model.params[i].type != ParamMsg::PARAM) continue;
//    	cout <<" param "<<model.params[i].name<<"="<<model.params[i].value<<endl;
//    }

  return true;
}

bool ArticulationModel::fitMinMaxConfigurations()
{
  setParam("q_min",getMinConfigurationObserved(),articulation_model_msgs::ParamMsg::PARAM);
  setParam("q_max",getMaxConfigurationObserved(),articulation_model_msgs::ParamMsg::PARAM);
  return true;
}

void ArticulationModel::updateParameters(std::vector<double> delta)
{
}

bool ArticulationModel::normalizeParameters()
{
  return true;
}

bool ArticulationModel::fitModel() {
  if(!sampleConsensus())
  {
    std::cout << "sampleConsesus failed" << std::endl;
          return false;
  }
  if(!optimizeParameters()) {
    std::cout << "optimizeParameters failed "<< std::endl;
          return false;
  }
  if(!normalizeParameters()) {
          return false;
  }
    return true;
}

V_Configuration ArticulationModel::getMinConfigurationObserved()
{
  if(getDOFs()==0) return V_Configuration();
  V_Configuration q_min(getDOFs());
  for(size_t j=0;j<getDOFs();j++)
  {
    q_min[j] = std::numeric_limits<float>::max();
  }

  for(size_t i=0;i<model_msg.track.pose.size();i++)
  {
    V_Configuration q = getConfiguration(i);
    for(size_t j=0; j<getDOFs(); j++)
    {
      q_min[j] = std::min(q_min[j], q[j]);
    }
  }

  return q_min;
}

V_Configuration ArticulationModel::getMaxConfigurationObserved()
{
  if(getDOFs()==0) return V_Configuration();
  V_Configuration q_max(getDOFs());
  for(size_t j=0;j<getDOFs();j++)
  {
    q_max[j] = -std::numeric_limits<float>::max();
  }

  for(size_t i=0;i<model_msg.track.pose.size();i++)
  {
    V_Configuration q = getConfiguration(i);
    for(size_t j=0;j<getDOFs();j++)
    {
      q_max[j] = std::max( q_max[j], q[j] );
    }
  }

  return q_max;
}

V_Configuration ArticulationModel::getConfiguration(size_t index)
{
  std::map<int,int> channel;
  // get channel ids and prepare channels
  for(size_t ch=0; ch<getDOFs(); ch++)
  {
    std::stringstream s;
    s << "q" << ch;
    channel[ch] = openChannel( model_msg.track, s.str() );
  }

  V_Configuration q;
  if (getDOFs()) q.resize(getDOFs());
  for(int j=0;j<q.rows();j++)
  {
    q[j] = model_msg.track.channels[ channel[j] ].values[index];
  }
  return(q);
}


bool ArticulationModel::check_values(const tf::Vector3 &vec) {
  return(check_values(vec.x()) && check_values(vec.y()) && check_values(vec.z()) );
}
bool ArticulationModel::check_values(const tf::Quaternion &vec) {
  return(check_values(vec.x()) && check_values(vec.y()) && check_values(vec.z()) && check_values(vec.w()));
}
bool ArticulationModel::check_values(double v) {
  return(!isnan(v) && !isinf(v));
}
bool ArticulationModel::check_values(float v) {
  return(!isnan(v) && !isinf(v));
}

// -- evaluate model
bool ArticulationModel::evaluateModel()
{
  // need at least one data point
  if(model_msg.track.pose.size() == 0)
          return false;

  std::cerr << "size of points: " << model_msg.track.pose.size() << std::endl;

  // let getLogLikelihood() do the projection
  loglikelihood = getLogLikelihood(false);

  // evaluate some extra statistics

  // get params
  size_t n = model_msg.track.pose.size();

  // compute pose difference and likelihoods
  double sum_position2 = 0;
  double sum_orientation2 = 0;

  double sum_position = 0;
  double sum_orientation = 0;
//  double sum_loglikelihood = 0;

  for(size_t i=0;i<n;i++)
  {
    tf::Transform p1, p2;
    tf::poseMsgToTF(model_msg.track.pose[i], p1);
    tf::poseMsgToTF(model_msg.track.pose_projected[i], p2);

    tf::Transform diff = p1.inverseTimes(p2);
    double err_position2 = diff.getOrigin().length2();
    double err_orientation2 = (diff.getRotation().getAngle())*(diff.getRotation().getAngle());
//    sum_loglikelihood += model.track.channels[channelInlierLogLikelihood].values[i];

    sum_position2 += err_position2;
    sum_orientation2 += err_orientation2;
    sum_position += sqrt(err_position2);
    sum_orientation += sqrt(err_orientation2);
  }

  // store into cache
  avg_error_position = sum_position/n;
  avg_error_orientation = sum_orientation/n;
  loglikelihood += - ((double)n)*getDOFs()*log(n);

  bic =
        -2*(loglikelihood )
        +complexity
        * log( n );

  last_error_jacobian = evalLatestJacobian();

  evaluated = true;

  if(model_msg.track.pose.size()>=2)
  {
    Eigen::VectorXd q1 = predictConfiguration( model_msg.track.pose.front() );
    Eigen::VectorXd q2 = predictConfiguration( model_msg.track.pose.back() );

    jacobian = predictJacobian( q2 );
    hessian = predictHessian( q2 );
    if(getDOFs() >= 1 )
    {
      Eigen::VectorXd p(3);
      for(size_t i=0;i<model_msg.track.pose.size();i++)
      {
        p += pointToEigen(model_msg.track.pose[i].position) - pointToEigen(model_msg.track.pose.front().position);
      }
      if(p.dot(jacobian.col(0))<0)	// make Jacobian point into current direction (of dof 1)
        jacobian *= -1;
        hessian *= -1;
    }
}

  writeParamsToModel();
  return true;
}

double ArticulationModel::evalLatestJacobian()
{
  if(model_msg.track.pose.size()<2)
    return 0.00;

  // observed direction of motion
  Eigen::VectorXd p1 = pointToEigen( model_msg.track.pose[model_msg.track.pose.size()-1].position );
  Eigen::VectorXd p2 = pointToEigen( model_msg.track.pose[model_msg.track.pose.size()-2].position );
  Eigen::VectorXd pD_obs = p1-p2;
  if(pD_obs.norm() == 0)
          return 0.00;
  pD_obs.normalize();
//	cout << "p1="<<p1<<endl;
//	cout << "p2="<<p2<<endl;
//	cout << "pD_obs="<<pD_obs<<endl;
  if(getDOFs()==0)
    return 2*M_PI;

  // predicted direction of motion
  Eigen::VectorXd q1 = predictConfiguration( model_msg.track.pose[model_msg.track.pose.size()-1] );
  Eigen::VectorXd q2 = predictConfiguration( model_msg.track.pose[model_msg.track.pose.size()-2] );
//	cout << "q1="<<q1<<endl;
//	cout << "q2="<<q2<<endl;
  Eigen::MatrixXd J2 = predictJacobian( q2 );
//	cout << "J2="<<J2<<endl;
  Eigen::VectorXd qD = (q1 - q2);
//	cout << "qD="<<qD<<endl;
  Eigen::VectorXd pD_pred = J2 * qD; // reduce Jacobian
//	cout << "pD_pred="<<pD_pred<<endl;
  if(pD_pred.norm() == 0)
    return 0.00;
  pD_pred.normalize();
//	cout << "qD_pred.normalized="<<pD_pred<<endl;
//
//	cout << "angle_diff="<<acos( pD_obs.dot( pD_pred ) )<<endl;
//
//	// return angle between predicted motion and observed motion
  return acos( pD_obs.dot( pD_pred ) );
}

Eigen::VectorXd ArticulationModel::pointToEigen(const geometry_msgs::Point& p)
{
  Eigen::VectorXd vec(3);
  vec << p.x , p.y , p.z;
  return vec;
}

M_CartesianJacobian ArticulationModel::predictJacobian(V_Configuration vq,double delta)
{
  M_CartesianJacobian J;
  J.resize(3,getDOFs());
  Eigen::VectorXd p = pointToEigen(predictPose(vq).position);
  for(size_t i=0;i<getDOFs();i++)
  {
    V_Configuration q = vq;
    q(i) += delta;
    J.col(i) = (pointToEigen( predictPose(q).position ) - p)/delta;
  }
  return J;
}

M_CartesianJacobian ArticulationModel::predictHessian(V_Configuration q,double delta)
{
  M_CartesianJacobian H;
  H.resize(3*getDOFs(),getDOFs());
//	cout <<"dofs="<<getDOFs()<<" q.size"<<vq.size()<<endl;
  for(size_t i=0;i<getDOFs();i++)
  {
    V_Configuration qd = q;
    q(i) += delta;
    M_CartesianJacobian H_part;

    M_CartesianJacobian J = predictJacobian(q);
    M_CartesianJacobian Jd = predictJacobian(qd);

//		cout << J(0,0) << " "<< J(1,0) << " "<< J(2,0) << endl;
//		cout << "H_part "<<Jd(0,0) << " "<< Jd(1,0) << " "<< Jd(2,0) << endl;

    H_part = (Jd - J)/delta;
//		cout << "H_part "<<H_part(0,0) << " "<< H_part(1,0) << " "<< H_part(2,0) << endl;
    for(size_t r=0;r<3;r++)
    {
      for(size_t c=0;c<getDOFs();c++)
      {
        H(r+3*i,c) = H_part(r,c);
      }
    }
  }
  return H;
}

