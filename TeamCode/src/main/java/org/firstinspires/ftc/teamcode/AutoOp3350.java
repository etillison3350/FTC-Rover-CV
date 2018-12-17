package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.reflect.Field;

@Autonomous(name="3350", group="~~")
public class AutoOp3350 extends LinearOpMode {

	private static final String LEFT_MOTOR_NAME = "leftMotor";
	private static final String RIGHT_MOTOR_NAME = "rightMotor";
	private static final String LIFT_MOTOR_NAME = "lift";
	private static final String SLURP_MOTOR_NAME = "slurp";

	// Whether or not the direction of the motors in inverted (motor will be reversed if set to 'true')
	private static final boolean LEFT_INVERTED = false;
	private static final boolean RIGHT_INVERTED = false;
	private static final boolean LIFT_INVERTED = false; // 'Forward' is up
	private static final boolean SLURP_INVERTED = false; // Forward is take in

	private static final String LICENSE_KEY = ""; // TODO Add license key

	private DcMotor leftWheels, rightWheels, lift, slurp;

	private BNO055IMU imu;

	private SamplingOrderDetector detector;

	private long startTime;

	public void runOpMode() {
		///////////////////////// HARDWARE DEFINITIONS ////////////////////////
		leftWheels = hardwareMap.dcMotor.get(LEFT_MOTOR_NAME);
		rightWheels = hardwareMap.dcMotor.get(RIGHT_MOTOR_NAME);
		lift = hardwareMap.dcMotor.get(LIFT_MOTOR_NAME);
		slurp = hardwareMap.dcMotor.get(SLURP_MOTOR_NAME);

		if (LEFT_INVERTED) leftWheels.setDirection(DcMotor.Direction.REVERSE);
		if (RIGHT_INVERTED) rightWheels.setDirection(DcMotor.Direction.REVERSE);
		if (LIFT_INVERTED) lift.setDirection(DcMotor.Direction.REVERSE);
		if (SLURP_INVERTED) slurp.setDirection(DcMotor.Direction.REVERSE);

		// IMU
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);

		dogeCVSetup();

		/////////////////// PRE-START CONTROLLER PARAMETERS ///////////////////

		while (!isStarted()) {

		}

		/////////////////////////////// OP-MODE ///////////////////////////////


	}

	protected void resetTime() {
		startTime = System.currentTimeMillis();
	}

	/**
	 * Set up DogeCV. This setup code is adapted from GoldAlignExample available with DogeCV.
	 */
	private void dogeCVSetup() {
		// Set up detector
		detector = new SamplingOrderDetector(); // Create detector
		detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
		detector.useDefaults(); // Set detector to use default settings

		detector.enable();
	}

	/**
	 * @return The current heading of the robot, as measured by the imu, in degrees.
	 */
	protected double getImuHeading() {
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
	}

	/**
	 * Rotates the robot by the specified angle, relative to its angle at the time this method is called.
	 * This method blocks until the rotation is complete.
	 * @param power the power at which to run the motors
	 * @param degrees the angle to rotate, in degrees. Negative angles will rotate clockwise
	 */
	protected void rotate(double power, double degrees) {
		rotate(power, degrees, false);
	}

	/**
	 * Rotates the robot by the specified angle, relative to its angle at the time this method is called.
	 * This method blocks until the rotation is complete.
	 * @param power the power at which to run the motors
	 * @param degrees the angle to rotate, in degrees. Negative angles will rotate in the opposite direction of that specified.
	 * @param clockwise whether to rotate clockwise or counterclockwise
	 */
	protected void rotate(double power, double degrees, boolean clockwise) {
		double currentHeading = getImuHeading();

		double targetHeading = currentHeading + degrees;

		
	}

	// A handler wrapper for buttons on the controller, to trigger an action when the button is first pressed, but not afterwards.
	protected static class MomentaryButton {
		private final Field button;
		private final Gamepad gamepad;

		/**
		 * Whether or not the button being tracked has a boolean value. If it is not a boolean value
		 * (for example, the right trigger), it will be converted to a boolean value, where values
		 * greater than 0.5 are 'true' and those less are 'false'.
		 */
		private boolean isBoolean = true;

		/**
		 * Whether or not wasPressed() has been called since the button was pressed.
		 */
		private boolean pressProcessed = false;

		/**
		 * Creates a new MomentaryButton with the given button name and gamepad.
		 * @param gamepad - the gamepad (usually gamepad1 or gamepad2) to track
		 * @param buttonName - the name of the field (case sensitive) in Gamepad to track
		 */
		protected MomentaryButton(Gamepad gamepad, String buttonName) {
			try {
				button = Gamepad.class.getField(buttonName);
			} catch (NoSuchFieldException e) {
				throw new IllegalArgumentException("No button could be found with the name " + buttonName);
			}

			if (gamepad == null) throw new IllegalArgumentException("The given gamepad cannot be null");

			this.gamepad = gamepad;

			try {
				button.getBoolean(gamepad);
			} catch (IllegalAccessException | NullPointerException | ExceptionInInitializerError e) {
				throw new RuntimeException("Unexpected error occurred while creating Momentary Button, with button name \"" + buttonName + "\"");
			} catch (IllegalArgumentException e) {
				isBoolean = false;
			}
		}

		protected boolean wasPressed() {
			if (pressProcessed) {
				if (!buttonDown()) { // When the button is released, reset the flag
					pressProcessed = false;
				}
			} else {
				if (buttonDown()) { // When the button is pressed, set the flag and return that the button has been pressed.
					pressProcessed = true;
					return true;
				}
			}

			return false;
		}

		/**
		 * @return whether or not the button tracked by this MomentaryButton is pressed.
		 */
		private boolean buttonDown() {
			try {
				return isBoolean ? button.getBoolean(gamepad) : button.getFloat(gamepad) > 0.5F;
			} catch (IllegalAccessException e) {
				return false;
			}
		}
	}

}
