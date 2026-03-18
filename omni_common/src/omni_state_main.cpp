//TODO: Change way angular an linear velocities are taken from the device. The API gives ways of retrieving them. Don't know why they were not used.

#include "../../omni_common/include/omni_common/omni_state.hpp"

int calibrationStyle;

void logMapSO3(hduMatrix rot, hduMatrix rot_prev, hduVector3Dd& body_angular_vel, const double dt)
{
  Eigen::Matrix3d Rk = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Rkm1 = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Rk(i, j) = rot[i][j];
      Rkm1(i, j) = rot_prev[i][j];
    }
  }

  Eigen::Matrix3d Delta_R = Rk * Rkm1.transpose();
  Eigen::AngleAxisd angle_axis;
  angle_axis.fromRotationMatrix(Delta_R);
  double gamma = angle_axis.angle();

  Eigen::Matrix3d omega_skew = Eigen::Matrix3d::Zero();
  if (std::abs(gamma) > 1e-5)
  {
    omega_skew = gamma / (2 * std::sin(gamma)) * (Delta_R - Delta_R.transpose()) / dt;
  }
  // No minus sign here needed
  body_angular_vel[0] = omega_skew(1, 2);
  body_angular_vel[1] = -omega_skew(0, 2);
  body_angular_vel[2] = omega_skew(0, 1);
}

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  static hduMatrix rot_prev(1.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0,
                            0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0);

  static hduVector3Dd filtered_body_ang_vel(0.0, 0.0, 0.0);
  static bool initialized = false;
  if (!initialized) {
    rot_prev.getRotationMatrix(rot_prev);
    initialized = true;
  }
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      std::cout << "Updating Calibration.." << std::endl;
      hdUpdateCalibration(calibrationStyle);
  }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  hduVector3Dd body_ang_vel;
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  // Position
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  hduMatrix current_rot(rotation_offset * rotation);
  omni_state->rot = hduQuaternion(current_rot);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  // Compute Body Angular Velocity through the SO(3) Logarithmic map
  logMapSO3(current_rot, rot_prev, body_ang_vel, omni_state->dt);
  filtered_body_ang_vel = 0.99 * filtered_body_ang_vel + 0.01 * body_ang_vel;
  omni_state->body_angular_velocity[0] = filtered_body_ang_vel[0];
  omni_state->body_angular_velocity[1] = filtered_body_ang_vel[1];
  omni_state->body_angular_velocity[2] = filtered_body_ang_vel[2];
  //~ // Set forces if locked
  //~ if (omni_state->lock == true) {
    //~ omni_state->force = 0.04 * omni_state->units_ratio * (omni_state->lock_pos - omni_state->position)
        //~ - 0.001 * omni_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);
  
  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];

  rot_prev = current_rot;
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    std::cout << "HD_CALIBRATION_ENCODER_RESET.." << std::endl;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    std::cout << "HD_CALIBRATION_INKWELL.." << std::endl;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    std::cout << "HD_CALIBRATION_AUTO.." << std::endl;
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      std::cout << "Calibrating.. (put stylus in well)" << std::endl;
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    std::cout << "Calibration complete." << std::endl;
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      std::cout << "Calibration needs manual input.." << std::endl;
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      std::cout << "Calibration updated successfully" << std::endl;
      hdUpdateCalibration(calibrationStyle);
    }
    else
      std::cerr << "Unknown calibration status.." << std::endl;
  }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Init the Geomagic Device
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize device");
        rclcpp::shutdown();
        return -1;
    }

    hdEnable(HD_FORCE_OUTPUT);

    // Start the scheduler
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        hdDisableDevice(hHD);
        rclcpp::shutdown();
        return -1;
    }

    // Automatic Calibration
    HHD_Auto_Calibration();

    // Calibration Result 
    HDint calibStatus = hdCheckCalibration();
    if (calibStatus == HD_CALIBRATION_OK)
        RCLCPP_INFO(rclcpp::get_logger("main"), "Calibration OK");
    else if (calibStatus == HD_CALIBRATION_NEEDS_UPDATE)
        RCLCPP_INFO(rclcpp::get_logger("main"), "Calibration NEEDS UPDATE");
    else if (calibStatus == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
        RCLCPP_INFO(rclcpp::get_logger("main"), "Calibration NEEDS MANUAL INPUT");
    else
    RCLCPP_INFO(rclcpp::get_logger("main"), "Calibration status unknown: %d", calibStatus);

    // Ros Node
    OmniState state;
    auto geo_ros_node = std::make_shared<geoRos>("touch");
    geo_ros_node->init(&state);
    hdScheduleAsynchronous(
        omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(geo_ros_node);
    executor.spin();

    // On shutdown
    hdStopScheduler();
    hdDisableDevice(hHD);
    rclcpp::shutdown();
    return 0;
}