#include "main.h"

#include <cmath>

// V5 Inertial Sensor: 3-axis gyro (deg/s) + 3-axis accelerometer (g)
constexpr std::uint8_t IMU_PORT = 10;
pros::Imu imu(IMU_PORT);

// V5 Rotation Sensor on a drive axle / tracking wheel: angle + speed of the shaft
constexpr std::int8_t ROTATION_PORT = 11;
pros::Rotation rotation(ROTATION_PORT);

// Thresholds for deciding whether the robot is moving
constexpr double GYRO_MOVING_DPS = 5.0;     // any axis rotating faster than this
constexpr double ACCEL_MOVING_G = 0.05;     // total accel differing from 1 g (gravity) by more than this
constexpr double WHEEL_MOVING_DPS = 10.0;   // axle spinning faster than this
constexpr int STILL_CYCLES_REQUIRED = 10;   // consecutive quiet readings before we call it still

/**
 * Returns true if the gyro, accelerometer or rotation sensor shows the
 * robot is moving.
 *
 * The rotation sensor catches slow, smooth driving that barely shows up on
 * the accelerometer, while the IMU catches turning and bumps.
 *
 * Accel magnitude is compared against 1 g so gravity is ignored no matter
 * how the sensor is mounted. A short debounce stops the result flickering
 * between readings.
 */
bool is_moving() {
	static int still_cycles = 0;

	pros::imu_gyro_s_t gyro = imu.get_gyro_rate();
	pros::imu_accel_s_t accel = imu.get_accel();

	bool rotating = std::fabs(gyro.x) > GYRO_MOVING_DPS || std::fabs(gyro.y) > GYRO_MOVING_DPS ||
	                std::fabs(gyro.z) > GYRO_MOVING_DPS;
	double accel_mag = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
	bool accelerating = std::fabs(accel_mag - 1.0) > ACCEL_MOVING_G;
	double wheel_dps = rotation.get_velocity() / 100.0;  // centidegrees/s -> deg/s
	bool wheel_spinning = std::fabs(wheel_dps) > WHEEL_MOVING_DPS;

	if (rotating || accelerating || wheel_spinning) {
		still_cycles = 0;
		return true;
	}
	if (still_cycles < STILL_CYCLES_REQUIRED) still_cycles++;
	return still_cycles < STILL_CYCLES_REQUIRED;
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// Calibrate the IMU (~2 s). The robot must stay still during this.
	pros::lcd::set_text(3, "Calibrating IMU...");
	imu.reset(true);
	pros::lcd::clear_line(3);

	rotation.reset_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage

		// IMU readouts + movement detection
		pros::imu_gyro_s_t gyro = imu.get_gyro_rate();
		pros::imu_accel_s_t accel = imu.get_accel();
		pros::lcd::print(3, "Heading: %.1f deg", imu.get_heading());
		pros::lcd::print(4, "Gyro  x:%.1f y:%.1f z:%.1f", gyro.x, gyro.y, gyro.z);
		pros::lcd::print(5, "Accel x:%.2f y:%.2f z:%.2f", accel.x, accel.y, accel.z);
		pros::lcd::print(6, "Wheel: %.1f deg  %.1f deg/s", rotation.get_position() / 100.0,
		                 rotation.get_velocity() / 100.0);
		pros::lcd::print(7, is_moving() ? "MOVING" : "STILL");

		pros::delay(20);                               // Run for 20 ms then update
	}
}