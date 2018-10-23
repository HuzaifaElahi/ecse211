package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread implements Runnable {

	// Instantiate Color sensor and other variables
	public static SensorModes myColor = new EV3ColorSensor(Lab5.portColor);
	public static SampleProvider myColorSample = myColor.getMode("Red");
	static float[] sampleColor = new float[myColor.sampleSize()];
	private Odometer odo;
	private Navigation nav;
	static double []result = new double[3];
	private float color[];
	private static final double SQUARE_SIZE = 30.48;
	private static final int SENSOR_OFFSET = 13;
	static ArrayList<Double> points = new ArrayList<Double>();
	private float[] csData;
	double[] oldResult = new double [3];
	double oldSample;
	static int passedLine;
	static double newColor;
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
		color = new float[myColorSample.sampleSize()];
		this.csData = color;
	}

	@Override
	public void run() {

		// Set theta to 0 following ultrasonic localization
		odo.setTheta(0);

		// Move to the origin
		try {
			goToOrigin();
		} catch (OdometerExceptions e1) {
		}
		// Retrieve angle pts at each line intersection
		getLocalizationPts();
		Lab5.isLightLocalizing = false;

		// Localization is performed 
		try {
			performLocalization();
		} catch (OdometerExceptions e) {
		}

	} 

	/**
	 * Uses the array of theta values to calculate the accurate 0 heading
	 * @throws OdometerExceptions
	 */
	private void performLocalization() throws OdometerExceptions {

		Lab5.isLightLocalizingTurn = true;

		// Retreive theta values from each line detection
		double yPos = points.get(0);
		double xPos = points.get(1);
		double yNeg = points.get(2);
		double xNeg = points.get(3);

		double xOffset = -D * Math.cos((yNeg - yPos) / 2);
		double yOffset = -D * Math.abs(Math.cos((xNeg - xPos) / 2));

		// correct the odometer
		odo.setX(xOffset);
		odo.setY(yOffset);

		// Align the robot to 0degrees
		Navigation.turnWithTheta(0);
//		Navigation.travelTo(0, 0);
		Lab5.isLightLocalizingTurn = false;


	}

	/**
	 * Obtains the theta angles at each line intersection
	 */
	private void getLocalizationPts() {
		long correctionStart, correctionEnd;
		double currentOdo = odo.getXYT()[2];
		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.leftMotor.backward();
		Navigation.rightMotor.forward();

		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			correctionStart = System.currentTimeMillis();
			// Store current robot position and current theta
			result = odo.getXYT();
			//If line detected (intensity less than 0.35), only count once by keeping track of last value
			if((newColor) < 0.35 && oldSample > 0.35) {
				//Error handling 
				if(result != null) {
					//Beep to notify, update counter and find and set correct X and Y using old reference pts
					passedLine++;
					points.add(result[2]);
					Sound.beep();
				}
			}
			//Store color sample
			oldSample = newColor;

			if(passedLine > 0) {
				if(result[2] > currentOdo - 5 && result[2] < currentOdo + 5) {
					break;
				}
			}
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < 10) {
				try {
					Thread.sleep(10 - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
		Navigation.leftMotor.stop(true);
		Navigation.rightMotor.stop(false);
	}

	private void goToOrigin() throws OdometerExceptions {
		
		Navigation.leftMotor.setAcceleration(1000);
		Navigation.rightMotor.setAcceleration(1000);

		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);	
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);

		// turn 45 degrees clockwise and move to origin
		Navigation.leftMotor.rotate(Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45.0), true);
		Navigation.rightMotor.rotate(-Navigation.convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45.0), false);

		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();
		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
				Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
				Navigation.leftMotor.rotate(-Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), true);
				Navigation.rightMotor.rotate(-Navigation.convertDistance(Lab5.WHEEL_RAD, SENSOR_OFFSET), false);
				odo.setX(0);
				odo.setY(0);
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
			oldSample = newColor;
		}
	}
}

