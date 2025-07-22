#include "GaussMarkovProcess.hh"

//! [registerSampleSystem]

namespace lotusim::gazebo {
/////////////////////////////////////////////////
GaussMarkovProcess::GaussMarkovProcess()
{
  this->mean = 0;
  this->min = -1;
  this->max = 1;
  this->mu = 0;
  this->noiseAmp = 0.5;
  this->Reset();
  std::srand(std::time(NULL));
}

/////////////////////////////////////////////////
void GaussMarkovProcess::Reset()
{
  this->var = this->mean;
}

/////////////////////////////////////////////////
bool GaussMarkovProcess::SetMean(double _mean)
{
  if (this->min > _mean || this->max < _mean)
    return false;

  this->mean = _mean;
  this->Reset();
  return true;
}

/////////////////////////////////////////////////
bool GaussMarkovProcess::SetModel(double _mean, double _min, double _max,
    double _mu, double _noise)
{
  if (_min >= _max)
    return false;
  if (_min > _mean || _max < _mean)
    return false;
  if (_noise < 0)
    return false;
  if (_mu < 0 || _mu > 1)
    return false;
  this->mean = _mean;
  this->min = _min;
  this->max = _max;
  this->mu = _mu;
  this->noiseAmp = _noise;

  this->Reset();
  return true;
}

/////////////////////////////////////////////////
double GaussMarkovProcess::Update(double _time)
{
  double step = _time - this->lastUpdate;
  
  gzmsg << "\tTime = " << _time << std::endl
  << "\tLast update = " << lastUpdate << std::endl
  << "\tstep = " << step << std::endl;
  double random =  static_cast<double>(static_cast<double>(rand()) / RAND_MAX)
    - 0.5;
  this->var = (1 - step * this->mu) * this->var + this->noiseAmp * random;
 
  gzmsg << "\tvar= " << var << std::endl;

  if (this->var >= this->max)
    this->var = this->max;
  if (this->var <= this->min)
    this->var = this->min;

  this->lastUpdate = _time;

  return this->var;
}

/////////////////////////////////////////////////
void GaussMarkovProcess::Print()
{
  gzmsg << "\tMean = " << this->mean << std::endl
    << "\tMin. Limit = " << this->min << std::endl
    << "\tMax. Limit = " << this->max << std::endl
    << "\tMu = " << this->mu << std::endl
    << "\tNoise Amp. = " << this->noiseAmp << std::endl;
    
}
}  // namespace lotusim::gazebo