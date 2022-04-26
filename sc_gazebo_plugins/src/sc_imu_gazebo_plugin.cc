#include <sc_gazebo_plugins/sc_imu_gazebo_plugin.hh>

namespace gazebo
{
  IMUROSPlugin::IMUROSPlugin() : ROSBaseModelPlugin()
  { }

  IMUROSPlugin::~IMUROSPlugin()
  { }

void IMUROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  //Only need to load settings specific to this sensor
  GetSDFParam<double>(_sdf, "gyroscope_noise_density",
                      this->imuParameters.gyroscopeNoiseDensity,
                      this->imuParameters.gyroscopeNoiseDensity);
  GetSDFParam<double>(_sdf, "gyroscope_bias_random_walk",
                      this->imuParameters.gyroscopeRandomWalk,
                      this->imuParameters.gyroscopeRandomWalk);
  GetSDFParam<double>(_sdf, "gyroscope_bias_correlation_time",
                      this->imuParameters.gyroscopeBiasCorrelationTime,
                      this->imuParameters.gyroscopeBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.gyroscopeBiasCorrelationTime > 0.0,
    "Gyroscope bias correlation time must be greater than zero");
  GetSDFParam<double>(_sdf, "gyroscope_turn_on_bias_sigma",
                      this->imuParameters.gyroscopeTurnOnBiasSigma,
                      this->imuParameters.gyroscopeTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "accelerometer_noise_density",
                      this->imuParameters.accelerometerNoiseDensity,
                      this->imuParameters.accelerometerNoiseDensity);
  GetSDFParam<double>(_sdf, "accelerometer_random_walk",
                      this->imuParameters.accelerometerRandomWalk,
                      this->imuParameters.accelerometerRandomWalk);
  GetSDFParam<double>(_sdf, "accelerometer_bias_correlation_time",
                      this->imuParameters.accelerometerBiasCorrelationTime,
                      this->imuParameters.accelerometerBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.accelerometerBiasCorrelationTime > 0.0,
    "Accelerometer bias correlation time must be greater than zero");
  GetSDFParam<double>(_sdf, "accelerometer_turn_on_bias_sigma",
                      this->imuParameters.accelerometerTurnOnBiasSigma,
                      this->imuParameters.accelerometerTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "orientation_noise",
                      this->imuParameters.orientationNoise,
                      this->imuParameters.orientationNoise);
  
  this->imuROSMessage.header.frame_id = this->link->GetName();
  // Fill IMU message.
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as
  // a continuous-time density (two-sided spectrum); not the true covariance
  // of the measurements.
  // Angular velocity measurement covariance.
  this->AddNoiseModel("gyro_noise_density", this->imuParameters.gyroscopeNoiseDensity);
  double gyroVar = this->imuParameters.gyroscopeNoiseDensity *
    this->imuParameters.gyroscopeNoiseDensity;
  this->imuROSMessage.angular_velocity_covariance[0] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[4] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[8] = gyroVar;
  
  // Linear acceleration measurement covariance.
  this->AddNoiseModel("acc_noise_density", this->imuParameters.accelerometerNoiseDensity);
  double accelVar = this->imuParameters.accelerometerNoiseDensity *
    this->imuParameters.accelerometerNoiseDensity;
  this->imuROSMessage.linear_acceleration_covariance[0] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[4] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[8] = accelVar;

  // Orientation estimate covariance
  this->AddNoiseModel("orientation_noise_density", this->imuParameters.orientationNoise);
  double orientationVar = this->imuParameters.orientationNoise *
    this->imuParameters.orientationNoise;
  this->imuROSMessage.orientation_covariance[0] = orientationVar;
  this->imuROSMessage.orientation_covariance[4] = orientationVar;
  this->imuROSMessage.orientation_covariance[8] = orientationVar;

  // Store the acc. gravity vector
#if GAZEBO_MAJOR_VERSION >= 8
  this->gravityWorld = this->world->Gravity();
#else
  this->gravityWorld = this->world->GetPhysicsEngine()->GetGravity().Ign();
#endif

  double sigmaBonG = this->imuParameters.gyroscopeTurnOnBiasSigma;
  double sigmaBonA = this->imuParameters.accelerometerTurnOnBiasSigma;

  this->AddNoiseModel("gyro_turn_on_bias", sigmaBonG);
  this->AddNoiseModel("acc_turn_on_bias", sigmaBonA);

  // FIXME Add the noise amplitude input for gyroscope
  this->gyroscopeTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp));
  // FIXME Add the noise amplitude input for accelerometer
  this->accelerometerTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp));

  // TODO(nikolicj) incorporate steady-state covariance of bias process
  this->gyroscopeBias = ignition::math::Vector3d::Zero;
  this->accelerometerBias = ignition::math::Vector3d::Zero;

  this->rosSensorOutputPub =
    this->rosNode->advertise<sensor_msgs::Imu>(this->sensorOutputTopic, 1);
  
  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Imu>(
        this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool IMUROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

    // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time curTime = this->world->SimTime();
#else
    common::Time curTime = this->world->GetSimTime();
#endif

  double dt = curTime.Double() - this->lastMeasurementTime.Double();

  ignition::math::Pose3d worldLinkPose;
  ignition::math::Vector3d refLinVel;
  ignition::math::Vector3d bodyAngVel;
  ignition::math::Vector3d bodyLinAcc, worldLinAcc;
  ignition::math::Vector3d refGravityWorld;

  // Read sensor link's current pose and velocity
#if GAZEBO_MAJOR_VERSION >= 8
  bodyAngVel = this->link->RelativeAngularVel();
  bodyLinAcc = this->link->RelativeLinearAccel();
  worldLinkPose = this->link->WorldPose();
#else
  bodyAngVel = this->link->GetRelativeAngularVel().Ign();
  bodyLinAcc = this->link->GetRelativeLinearAccel().Ign();
  worldLinkPose = this->link->GetWorldPose().Ign();
#endif

  this->UpdateReferenceFramePose();
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->referenceFrame = this->referenceLink->WorldPose();
#else
    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }

  // Transform pose and velocity vectors to be represented wrt the
  // reference link provided
  worldLinkPose.Pos() = worldLinkPose.Pos() - this->referenceFrame.Pos();
  worldLinkPose.Pos() = this->referenceFrame.Rot().RotateVectorReverse(
    worldLinkPose.Pos());
  worldLinkPose.Rot() *= this->referenceFrame.Rot().Inverse();

  ignition::math::Vector3d gravityBody =
    worldLinkPose.Rot().RotateVectorReverse(this->gravityWorld);

  if (this->enableLocalNEDFrame)
  {
    bodyAngVel = this->localNEDFrame.Rot().RotateVector(bodyAngVel);
    bodyLinAcc = this->localNEDFrame.Rot().RotateVector(bodyLinAcc);
    gravityBody = this->localNEDFrame.Rot().RotateVector(gravityBody);
  }

  // Compute the simulated measurements wrt the default world ENU frame
  this->measLinearAcc = bodyLinAcc - gravityBody;
  this->measAngularVel = bodyAngVel;
  this->measOrientation = worldLinkPose.Rot();
