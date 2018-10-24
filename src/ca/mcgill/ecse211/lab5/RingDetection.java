package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class RingDetection  {
	private static final long CORRECTION_PERIOD = 10;
	private static float[] blueRGB = { 0.030f, 0.105f, 0.120f };
	private static float[] greenRGB = { 0.043f, 0.095f, 0.020f };
	private static float[] yellowRGB = { 0.188f, 0.132f, 0.032f };
	private static float[] orangeRGB = { 0.103f, 0.030f, 0.028f };
	public static final String COLOR_TO_DETECT = "Yellow";

	public static Port portRing = LocalEV3.get().getPort("S4");
	public static SensorModes colorSensor = new EV3ColorSensor(portRing);
	public static SampleProvider colorSample = colorSensor.getMode("RGB");
	float[] colorData;

	public RingDetection() {
		colorData = new float[colorSensor.sampleSize()];
		float[] rgbValues = new float[3];
		colorSample.fetchSample(rgbValues, 0);
	}

	public static int detectRing() {
		long sampleStart, sampleEnd;
		boolean ringDetected = false;
		int colorCode = 0;
		double distance = 30;
		double[] oldOdometer = {0,0,0}, newOdometer  = {0,0,0};

		try {
			oldOdometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e1) {
			e1.printStackTrace();
		}
		float[] rgbValues = new float[3];
		Navigation.moveStraight(Lab5.leftMotor, Lab5.rightMotor, 20, 150, true, true);
		
		while (true) {
			sampleStart = System.currentTimeMillis();
			colorSample.fetchSample(rgbValues, 0);
			colorCode = colorDetection(rgbValues);
			if (colorCode != 0) {
				Lab5.leftMotor.stop(true);
				Lab5.rightMotor.stop();
				try {
					newOdometer = Odometer.getOdometer().getXYT();
				} catch (OdometerExceptions e1) {
					e1.printStackTrace();
				}
				
				distance = Math.hypot(Math.abs(oldOdometer[0] - newOdometer[0]), Math.abs(oldOdometer[1] - newOdometer[1]));
				colorCode = takeMoreReadings();
				if(colorCode == Lab5.TR) {
					Sound.beep();
					Sound.beep();
					Lab5.leftMotor.stop(true);
					Lab5.rightMotor.stop();
					ringDetected = true;


					Lab5.lcd.drawString("R: " + 100 * rgbValues[0], 0, 1);
					Lab5.lcd.drawString("G: " + 100 * rgbValues[1], 0, 2);
					Lab5.lcd.drawString("B: " + 100 * rgbValues[2], 0, 3);
					Lab5.lcd.drawString("Color: " + colorDetection(rgbValues), 0, 4); // display the Color detected
					break;
				}
				if (!ringDetected) {
					Sound.beep();
					ringDetected = true;
					Lab5.lcd.drawString("R: " + 100 * rgbValues[0], 0, 1);
					Lab5.lcd.drawString("G: " + 100 * rgbValues[1], 0, 2);
					Lab5.lcd.drawString("B: " + 100 * rgbValues[2], 0, 3);
					Lab5.lcd.drawString("Color: " + colorDetection(rgbValues), 0, 4); // display the Color detected
					break;
				} else {
					break;
				}

			}
			if (colorCode == 0) {
				ringDetected = false;
			}

			Lab5.lcd.drawString("R: " + 100 * rgbValues[0], 0, 1);
			Lab5.lcd.drawString("G: " + 100 * rgbValues[1], 0, 2);
			Lab5.lcd.drawString("B: " + 100 * rgbValues[2], 0, 3);
			Lab5.lcd.drawString("Color: " + colorDetection(rgbValues), 0, 4); // display the Color detected

			// this ensure the odometry correction occurs only once every period
			sampleEnd = System.currentTimeMillis();
			if (sampleEnd - sampleStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (sampleEnd - sampleStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
			
			if(!Lab5.leftMotor.isMoving()) {
				break;
			}
		}
		
		Navigation.moveStraight(Lab5.leftMotor, Lab5.rightMotor, distance+3, 120, false, false);
		return colorCode;
	}


	public static int takeMoreReadings() {
		int[] list = new int[300];
		float[] rgbValues = new float[3];
		Navigation.moveStraight(Lab5.leftMotor, Lab5.rightMotor, 1.5, 40, true, false);
		for(int i = 0; i < 300; i++) {
			colorSample.fetchSample(rgbValues, 0);
			list[i] = colorDetection(rgbValues);
		}
		return findMode(list);
		
	}
	private static int findMode(int[] samples) {
		int [] data = new int[5];
		for(int j = 0; j < samples.length; j++) {
			data[samples[j]]++;
		}
		int mode = 0, temp = data[0];
		for(int i = 1; i < 5; i++){
			if(data[i] > temp){
				mode = i;
				temp = data[i];
			}
		}
		return mode;
	}


	static int colorDetection(float[] rgb) {
		float[] colorsDistances = new float[5];

		// if this distance is the minimum then, no Color/Object is detected
		colorsDistances[0] = 0.05f; // to modify

		// distance from Blue
		colorsDistances[1] = (float) distance(blueRGB, rgb);

		// distance from Green
		colorsDistances[2] = (float) distance(greenRGB, rgb);

		// distance from Yellow
		colorsDistances[3] = (float) distance(yellowRGB, rgb);

		// distance from Orange
		colorsDistances[4] = (float) distance(orangeRGB, rgb);

		int minimum = getMinIndex(colorsDistances);
		// lcd.drawString("BDist: " + 100 * colorsDistances[1], 0, 2);
		// lcd.drawString("GDist: " + 100 * colorsDistances[2], 0, 3);
		// lcd.drawString("YDist: " + 100 *colorsDistances[3], 0, 4);
		// lcd.drawString("ODist: " + 100 * colorsDistances[4], 0, 5);
		Lab5.lcd.drawString("min d: " + colorsDistances[minimum], 0, 6);
		return minimum;
	}

	private static double distance(float[] rgbMean, float[] rgbValues) {
		return Math.sqrt(Math.pow(rgbMean[0] - rgbValues[0], 2) + Math.pow(rgbMean[1] - rgbValues[1], 2)
				+ Math.pow(rgbMean[2] - rgbValues[2], 2));
	}

	private static int getMinIndex(float[] inputArray) {
		double minValue = inputArray[0];
		int minIndex = 0;
		for (int i = 0; i < inputArray.length; i++) {
			if (inputArray[i] < minValue) {
				minValue = inputArray[i];
				minIndex = i;
			}
		}
		return minIndex;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}