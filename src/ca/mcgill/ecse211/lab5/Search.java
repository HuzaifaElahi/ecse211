package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Search extends Thread{

	private static double waypoints[][];
	
	private static final int RING_INBOUND = 6;

	private Navigation navigator;
	private Odometer odo;
	private UltrasonicPoller usPoller;
  	private EV3LargeRegulatedMotor leftMotor;
  	private EV3LargeRegulatedMotor rightMotor;
	

	/**
	 * Constructor
	 */
	public Search(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) 
			throws OdometerExceptions {
		this.usPoller = usPoller;
		this.odo = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	
	@SuppressWarnings("unused")
	public void run() {		
		
		waypoints = createWayPoints(Lab5.LLx, Lab5.LLy, Lab5.URx, Lab5.URy);	// create waypoints to travel to
		
		double[] nextXY = new double[2];
		double d;
		
		//this is for first destination
		nextXY = waypoints[0];
		Navigation.travelTo(nextXY[0], nextXY[1]); 
		Display.displayColorResult(checkForRing());
		
		//now move to rest of waypoints
		for (int i = 1; i < waypoints.length; i++) {
			nextXY = waypoints[i];
			Navigation.travelTo(nextXY[0], nextXY[1]); //go to next destination
			//turn left check for ring
			Navigation.turnRobot(leftMotor, rightMotor, 45, 60, false, false);

			Display.displayColorResult(checkForRing());
			//turn right and check for ring
			Navigation.turnRobot(leftMotor, rightMotor, 90, 60, true, false);

			Display.displayColorResult(checkForRing());
			Navigation.turnRobot(leftMotor, rightMotor, 45, 60, false, false);

		}
	}
	
	/** 
	 * this method checks for a ring in front of the robot
	 * assumes in right direction
	 */
	//can also incorporate readings from light sensor
	public int checkForRing() {
		int d = (int)(UltrasonicPoller.usData[0]*100.0);
		if (d < RING_INBOUND) { //ring for sure ahead no need to check take reading
			return detectRingColor();
		} else if (d > 13) {
			if (doubleCheck()) {
				return detectRingColor();
			}
		}
		return 0;
	}
	
	public boolean doubleCheck() {
		boolean ring = false;
		Navigation.moveStraight(leftMotor, rightMotor, 1, 20, true, false);
		if ((int)(UltrasonicPoller.usData[0]*100.0) < 10) ring = true;
		Navigation.moveStraight(leftMotor, rightMotor, 1, 20, false, false);
		return ring;
	}
	
	
	private int detectRingColor() {
		Navigation.turnRobot(leftMotor, rightMotor, 20, 50, true, true);
		ArrayList<Integer> samples = new ArrayList<Integer>();
		float[] rgb = new float[]{0, 0, 0};
		int colorCode = 0;
		while(leftMotor.isMoving()) {
			try {
				Thread.sleep(200);
			} catch (Exception e) {
			} // Poor man's timed sampling
//			RingDetection.myColorSample.fetchSample(rgb, 0);
//			colorCode = RingDetection.detectColor(rgb); //gets sensor data and passes to colordetector class
			samples.add(colorCode);
		}
		Navigation.turnRobot(leftMotor, rightMotor, 20, 50, false, true);
		colorCode = findMode(samples);
		return colorCode;
	}

	
	private int findMode(ArrayList<Integer> samples) {
		int [] data = new int[4];
		for(Integer sample : samples)
			data[sample-1]++;
		int mode = 0, temp = data[0];
		for(int i = 1; i < 4; i++){
			if(data[i] > temp){
				mode = i;
				temp = data[i];
			}
		}
		return mode + 1;
	}
	
	public static double[][] createWayPoints(int lx, int ly, int ux, int uy) {
		int numbOfRows = uy - ly + 1, laps = 0;
		if ((numbOfRows%2)==0) laps = numbOfRows/2;
		else laps = numbOfRows/2 +1;
		int numberOfWaypoints = (ux - lx) * laps + 1;
		double[][] waypoints = new double[numberOfWaypoints][2];
		waypoints[0][0] = (lx * Lab5.SQUARE_SIZE)/Lab5.SQUARE_SIZE;
		waypoints[0][1] = (ly * Lab5.SQUARE_SIZE + (Lab5.SQUARE_SIZE/2))/Lab5.SQUARE_SIZE;
		
		boolean east = true;
		int j = 1;
		for(int y = ly; y < uy + 1; y += 2) {
			if (east) {
				for(int x = lx + 1; x < ux + 1; x++, j++) {
					waypoints[j][0] = (x * Lab5.SQUARE_SIZE - 10)/Lab5.SQUARE_SIZE;
					waypoints[j][1] = (y * Lab5.SQUARE_SIZE + (Lab5.SQUARE_SIZE/2))/Lab5.SQUARE_SIZE ;
				}
				east = false;
			} else {
				for(int x = ux - 1; x > lx - 1; x--, j++) {
					waypoints[j][0] = (x * Lab5.SQUARE_SIZE + 10)/Lab5.SQUARE_SIZE;
					waypoints[j][1] = (y * Lab5.SQUARE_SIZE + (Lab5.SQUARE_SIZE/2))/Lab5.SQUARE_SIZE;
				}
				east = true;
			}
		}
		return waypoints;
	}
	
}
