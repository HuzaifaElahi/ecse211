package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements UltrasonicController {
	private LocalizationType type;
	private Odometer odo;
	private Navigation nav;
	static final int MOTOR_SPEED = 80;
	private static final int D_THRESHHOLD = 30;
	private static final int NOISE_MARGIN = 5;
	private static final int FILTER_OUT = 15;
	private static final int ODO_CORRECTION = 0;
	private static double ALPHA = 0;
	private static double BETA = 0;
	private static double ANGLE_CORRECTION = 0;
	private static double FINAL_ANGLE = 0;
	private int filterControl;

	public int distance;


	public static enum LocalizationType {
		FALLING_EDGE
	}

	public UltrasonicLocalizer(LocalizationType edge, Odometer odometer, Navigation nav) 
			throws OdometerExceptions {
		this.type = edge;
		this.odo = Odometer.getOdometer(Lab5.leftMotor, Lab5.rightMotor, Lab5.TRACK, Lab5.WHEEL_RAD);
		Lab5.leftMotor.setSpeed(MOTOR_SPEED);
		Lab5.rightMotor.setSpeed(MOTOR_SPEED);
		this.nav = nav;
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// Print values

		if(Lab5.isUSLocalizing) {
			Lab5.lcd.clear();
			Lab5.lcd.drawString("Distance: " + distance, 0, 1);
			Lab5.lcd.drawString("Alpha: " + ALPHA, 0, 2);
			Lab5.lcd.drawString("Beta: " + BETA, 0, 3);
			Lab5.lcd.drawString("Final: " + FINAL_ANGLE, 0, 4);
		} else if(Lab5.isLightLocalizing) {
			Lab5.lcd.clear();
			Lab5.lcd.drawString("passedLines: " + LightLocalizer.passedLine, 0, 1);
			Lab5.lcd.drawString("color: "+ LightLocalizer.newColor, 0, 2);
			Lab5.lcd.drawString("x: "+ LightLocalizer.result[0], 0, 3);
			Lab5.lcd.drawString("y: "+ LightLocalizer.result[1], 0, 4);
			Lab5.lcd.drawString("theta: "+ LightLocalizer.result[2], 0, 5);
		}
		else if(Lab5.isLightLocalizingTurn) {
			Lab5.lcd.clear();
			Lab5.lcd.drawString("passedLines: " + LightLocalizer.passedLine, 0, 1);
			Lab5.lcd.drawString("points size: " + LightLocalizer.points.size(), 0, 2);
			Lab5.lcd.drawString("x: " + odo.getXYT()[0], 0, 3);
			Lab5.lcd.drawString("y: " + odo.getXYT()[1], 0, 4);
			Lab5.lcd.drawString("theta: " + odo.getXYT()[2], 0, 5);
		}
		else if(Lab5.isGoingToLL) {
			Lab5.lcd.clear();
			Lab5.lcd.drawString("x: " + odo.getXYT()[0]/Lab5.SQUARE_SIZE, 0, 1);
			Lab5.lcd.drawString("y: " + odo.getXYT()[1]/Lab5.SQUARE_SIZE, 0, 3);
			Lab5.lcd.drawString("theta: " + odo.getXYT()[2], 0, 5);
		}

	}

	/**
	 * Performs falling edge localization
	 * @throws OdometerExceptions 
	 */
	void fallingEdge() throws OdometerExceptions{

		//Instantiate odometer storage and set theta of odometer to 0
		double[] odometer = {0,0,0};
		boolean isAboveThresh = false;
		Odometer.getOdometer().setTheta(0);

		// Checks orientation or sets orientation to perform localization
		if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
			isAboveThresh = true;
		} else {
			findWallAbove();
			isAboveThresh = true;
		}

		// Find first falling edge
		while (true) {

			// Move forward and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			// If is falling and you are above the threshold
			// then store theta as alpha and stop turning
			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				ALPHA = odometer[2];
				isAboveThresh = false;
				break;
			}
		}

		// Find second falling edge
		while (true) {

			// Go backwards and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.backward();
			Navigation.rightMotor.forward();

			// Set above thresh to true if you are above the threshold 
			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				isAboveThresh = true;
			}

			// If is falling and you are above the threshold
			// then store 180-theta as beta and stop turning
			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				BETA = odometer[2];
				break;
			}
		}

		//TODO SORT THIS STUFF OUT
		// Alpha and Beta algorithms
		if (ALPHA < BETA) {
			ANGLE_CORRECTION = 40 - ((ALPHA + BETA) / 2); 
		} else {
			ANGLE_CORRECTION = 220 - ((ALPHA + BETA) / 2);
		} 

		// Set theta to 0 to apply correction
		// from current reference angle
		//Odometer.getOdometer().setTheta(0);
		FINAL_ANGLE = 180-(ANGLE_CORRECTION+odometer[2]+ODO_CORRECTION);
		Navigation.turnTo(FINAL_ANGLE);
	}

	
	/**
	 * Sets orientation of robot so it can perform the localization with falling edges 
	 * Makes sure that you are above detectable threshold (i.e facing far from wall)
	 * before you read for falling edge
	 */
	void findWallAbove() {
		while (true) {
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
		}
	}

	
	/**
	 * Checks if fallingEdge, i.e distance
	 * drops below the threshold
	 * @return boolean: if fallingEdge or not
	 */
	boolean isFalling() {
		if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public int readUSDistance() {
		return distance;
	}

}