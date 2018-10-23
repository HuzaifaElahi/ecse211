package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class RingDetection {
	public static SensorModes myColor = new EV3ColorSensor(Lab5.portColor);
	public static SampleProvider myColorSample = myColor.getMode("RGB");
	static float[] sampleColor = new float[myColor.sampleSize()];

	public static int detectColor(float[] rgb) {
		// yellow
		if(
				rgb[0] >= 6 && 
				rgb[0] <= 9 &&

				rgb[1] >= 4 && 
				rgb[1] <= 7 &&

				rgb[2] >= 0 && 
				rgb[2] <= 2
				)
			return 3;

		// blue
		if(
				rgb[0] >= 0 && 
				rgb[0] <= 3 &&

				rgb[1] >= 3 && 
				rgb[1] <= 8 &&

				rgb[2] >= 1 && 
				rgb[2] <= 5
				)
			return 1;

		// orange
		if(
				rgb[0] >= 2 && 
				rgb[0] <= 6 &&

				rgb[1] >= 0 && 
				rgb[1] <= 3 &&

				rgb[2] >= 0 && 
				rgb[2] <= 1
				)
			return 4;

		// green
		if(
				rgb[0] >= 0 && 
				rgb[0] <= 4 &&

				rgb[1] >= 2 && 
				rgb[1] <= 8 &&

				rgb[2] >= 0 && 
				rgb[2] <= 2
				)
			return 2;

		return 0;
	}
}