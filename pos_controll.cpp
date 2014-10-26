
float g = 9.8f;

int move_desire_pos();			// desired_vel_to_pos(_dt_xy);
int pos_to_rate();				// pos_to_rate_xy(use_desired_velocity,_dt_xy);
int rate_to_accel();			// rate_to_accel_xy(_dt_xy);
int accel_to_lean_angles();

int lean_angles_to_accel_NED()
{
	// lean angle to accel
	float accel_forward = g * (-sin_pitch/cos_pitch);		// lean forward = negetive pitch angle
	float accel_right = g * (sin_roll/cos_roll);

	// rotate accel from forward-right to north-east axis
	float accel_north = cos_yaw * accel_forward - sin_yaw * accel_right;
	float accel_east = sin_yaw * accel_forward + cos_yaw * accel_right;
}

int accel_to_lean_angles()
{
	// rotate from north-east to forward-right axis
	float accel_forward = cos_yaw * accel_north + sin_yaw * accel_east;
	float accel_right = -sin_yaw * accel_north + cos_yaw * accel_east;

	// TODO: check and handle accel limitation

	// accel to lean angle
	float pitch = atan2(-accel_forward, g);
	float roll = atan(accel_right*cos(pitch), g);
}