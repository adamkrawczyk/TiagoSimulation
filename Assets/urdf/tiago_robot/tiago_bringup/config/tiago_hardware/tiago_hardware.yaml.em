actuators:
  raw_data:
    position_port: act_position
    effort_port: act_current
    mode_port: act_control_mode
    position_cmd_port: ref_position
    velocity_cmd_port: ref_velocity
    effort_cmd_port: ref_current
    mode_cmd_port: ref_control_mode
    current_limit_cmd_port: ref_limit_current
    absolute_encoder_position_port: act_abs_position

joint_mode_black_list: ['head_controller', 'torso_controller']

@[if ft_sensor == "schunk-ft"]@
force_torque:
  wrist_ft:
    frame: wrist_ft_link
    transformation:
      force: [-y,x,z]
      torque: [-y,x,z]
    raw_data:
      force_port: force_wrist
      torque_port: torque_wrist
@[end if]@

e_stop:
  raw_data:
    e_stop_port: emergency_stop_state

ros_control_component:
  ignore_read_errors: false
