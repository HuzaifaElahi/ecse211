package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread implements Runnable {

	// Instantiate Color sensor and other variables
	public static SensorModes myColorLeft = new EV3ColorSensor(Lab5.portColorLeft);
	public static SampleProvider myColorSampleLeft = myColorLeft.getMode("Red");
	static float[] sampleColorLeft = new float[myColorLeft.sampleSize()];
	public static float colorLeft[];
	static double newColorLeft;
	static double oldSampleLeft;


	public static SensorModes myColorRight = new EV3ColorSensor(Lab5.portColorRight);
	public static SampleProvider myColorSampleRight = myColorRight.getMode("Red");
	static float[] sampleColorRight = new float[myColorRight.sampleSize()];
	public static float colorRight[];
	static double newColorRight;
	static double oldSampleRight;


	private Odometer odo;
	private Navigation nav;
	public static double []result = new double[3];
	public static final double SENSOR_OFFSET = 5;
	static ArrayList<Double> points = new ArrayList<Double>();
	double[] oldResult = new double [3];
	static int passedLine;
	private static final double D = 13;
	static double xOffset = 0;
	static double yOffset = 0;
	double dy;
	double dx;

	public LightLocalizer(Odometer odometer, Navigation nav) throws OdometerExceptions {
		this.odo = Odometer.getOdometer(Lab5.leftMotor, Lab5.rightMotor, Lab5.TRACK, Lab5.WHEEL_RAD);
		odo.setTheta(0);
		result = odo.getXYT();
		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		this.nav = nav;
		colorLeft = new float[myColorSampleLeft.sampleSize()];
		colorRight = new float[myColorSampleRight.sampleSize()];
	}

	@Override
	public void run() {

		// Set theta to 0 following ultrasonic localization
		odo.setTheta(0);

		// Move to the origin
		try {
			findStraightLine();
		} catch (OdometerExceptions e1) {
		}
		// Retrieve angle pts at each line intersection
		findRightLine();

	} 


	/**
	 * Obtains the theta angles at each line intersection
	 */
	private void findRightLine() {
		int foundLeft = 0;
		int foundRight = 0;
		Navigation.leftMotor.setAcceleration(1000);
		Navigation.rightMotor.setAcceleration(1000);
		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);	
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);

		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();

		while(true) {
			//color sensor and scaling
			myColorSampleLeft.fetchSample(colorLeft, 0);
			myColorSampleRight.fetchSample(colorRight, 0);
			newColorLeft = colorLeft[0];
			newColorRight = colorRight[0];
			result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				Navigation.leftMotor.stop(true);
				foundLeft++;
			}
			if((newColorRight) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				Navigation.rightMotor.stop(true);
				foundRight++;
			}
			oldSampleLeft = newColorLeft;
			oldSampleRight = newColorRight;
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		
		//Turn to 0 degrees at 1,1
		odo.setTheta(90);
		Navigation.leftMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), true);
		Navigation.rightMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), false);
		Navigation.leftMotor.rotate(-Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), true);
		Navigation.rightMotor.rotate(+Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), false);
		odo.setY(Lab5.SQUARE_SIZE);
		odo.setX(Lab5.SQUARE_SIZE);
		odo.setTheta(0);
	}

	private void findStraightLine() throws OdometerExceptions {
		int foundLeft = 0;
		int foundRight = 0;

		Navigation.leftMotor.setAcceleration(1000);
		Navigation.rightMotor.setAcceleration(1000);

		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);

		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();

		while(true) {
			//color sensor and scaling
			myColorSampleLeft.fetchSample(colorLeft, 0);
			myColorSampleRight.fetchSample(colorRight, 0);
			newColorLeft = colorLeft[0];
			newColorRight = colorRight[0];
			result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				Navigation.leftMotor.stop(true);
				foundLeft++;
			}
			if((newColorRight) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				Navigation.rightMotor.stop(true);
				foundRight++;
			}
			oldSampleLeft = newColorLeft;
			oldSampleRight = newColorRight;
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		// Turn 90 at 1,0
		odo.setTheta(0);
		Navigation.leftMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), true);
		Navigation.rightMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), false);
		Navigation.leftMotor.rotate(Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), true);
		Navigation.rightMotor.rotate(-Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), false);
		odo.setY(Lab5.SQUARE_SIZE);
		odo.setX(15);
		odo.setTheta(90);
	}
}

