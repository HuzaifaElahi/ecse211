package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.UltrasonicLocalizer.LocalizationType;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Class with main method to start the UI
 * @author Huzaifa, Jake
 *
 */

// TODO
// - make faster start to loading sensors
//


public class Lab5 {

	// Instantiate relevant variables 
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.20;
	public static final double SQUARE_SIZE = 30.48;
	public static final double TRACK = 10.4;
	public static boolean isUSLocalizing = false;
	public static boolean isLightLocalizing = false;
	public static boolean isLightLocalizingTurn = false;
	public static boolean isGoingToLL = false;

	static Odometer odometer = null;

	public static final int LLx = 2;
	public static final int LLy = 2;
	public static final int URx = 5;
	public static final int URy = 5;
	public static final int SC = 0;
	public static final int TR = 1;


	//Motors and sensor initialization
	static final Port usPort = LocalEV3.get().getPort("S1");
	static final Port portColorLeft = LocalEV3.get().getPort("S2");
	static final Port portColorRight = LocalEV3.get().getPort("S3");
	static final Port portRing = LocalEV3.get().getPort("S4");


	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));


	public static void main(String[] args) throws OdometerExceptions {
		
		int buttonChoice = 0;

		do {
			lcd.clear();   		// clear the display

			lcd.drawString("<      >", 0, 0);
			lcd.drawString("Falling ", 0, 1);
			lcd.drawString(" Edge   ", 0, 2);
			lcd.drawString("        ", 0, 3);
			lcd.drawString("<      >", 0, 4);

			buttonChoice = Button.waitForAnyPress();      // Record choice (left or right press)

			// Until button pressed
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT); 

		Button.waitForAnyPress();
		lcd.clear();
		
		// Set odometer and start thread
		try {
			odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
		}
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Navigation nav = new Navigation(leftMotor, rightMotor, odometer);
		try {
			Display odometryDisplay = new Display(lcd);
		} catch (Exception e1) {
			e1.printStackTrace();
		}

		RingDetection ringDetector = new RingDetection();
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(LocalizationType.FALLING_EDGE, odometer, nav);
		SensorModes usSensor = new EV3UltrasonicSensor(Lab5.usPort);                 // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");                    // usDistance provides samples 
		float[] usData = new float[usDistance.sampleSize()];                         // usData is the buffer for data
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer); // Instantiate poller
		usPoller.start();
		
//		// Ultrasonic localize
//		isUSLocalizing = true;
//		usLocalizer.fallingEdge();
//		isUSLocalizing = false;

//		// Light localize
//		isLightLocalizing = true;
		LightLocalizer lightLocalizer  = new LightLocalizer(odometer, nav);    
//		lightLocalizer.start();
//		try {
//			lightLocalizer.join();
//		} catch (InterruptedException e) {
//
//		}
		buttonChoice = 0;
		odometer.setXYT(SQUARE_SIZE,SQUARE_SIZE , 0);

		int colorCode = 0;
		do {
			float[] rgbValues = new float[3];
			RingDetection.colorSample.fetchSample(rgbValues, 0);
			colorCode = RingDetection.colorDetection(rgbValues);
			Lab5.lcd.drawString("R: " + 100 * rgbValues[0], 0, 1);
			Lab5.lcd.drawString("G: " + 100 * rgbValues[1], 0, 2);
			Lab5.lcd.drawString("B: " + 100 * rgbValues[2], 0, 3);
			Lab5.lcd.drawString("Color: " + colorCode, 0, 4); // display the Color detected
	} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT); 
		
	//	Search search = new Search(leftMotor, rightMotor, usPoller);
	//	search.start();
		
		Button.waitForAnyPress();



		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	

	}

}