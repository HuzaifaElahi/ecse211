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
public class Lab5 {

	// Instantiate relevant variables 
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.20;
	public static final double SQUARE_SIZE = 30.48;
	public static final double TRACK = 14.0;
	public static boolean isUSLocalizing = false;
	public static boolean isLightLocalizing = false;
	public static boolean isLightLocalizingTurn = false;
	public static boolean isGoingToLL = false;

	static Odometer odometer = null;

	public static final int LLx = 2;
	public static final int LLy = 2;
	public static final int URx = 7;
	public static final int URy = 7;
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
		
		int buttonChoice;

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

		
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(LocalizationType.FALLING_EDGE, odometer, nav);
		SensorModes usSensor = new EV3UltrasonicSensor(Lab5.usPort);                 // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");                    // usDistance provides samples 
		float[] usData = new float[usDistance.sampleSize()];                         // usData is the buffer for data
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer); // Instantiate poller
		usPoller.start();
		
		// Ultrasonic localize
		isUSLocalizing = true;
		usLocalizer.fallingEdge();
		isUSLocalizing = false;

		// Light localize
		isLightLocalizing = true;
		LightLocalizer lightLocalizer  = new LightLocalizer(odometer, nav);    
		lightLocalizer.start();
		try {
			lightLocalizer.join();
		} catch (InterruptedException e) {

		}
		
		odometer.setXYT(SQUARE_SIZE,SQUARE_SIZE , 0);

	//	isGoingToLL = true;
		Navigation.travelTo(1, LLy);
		Navigation.travelTo(LLx, LLy);
		
		Search search = new Search(leftMotor, rightMotor, usPoller);
		search.start();
		
		


		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	

	}

}