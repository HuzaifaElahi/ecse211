package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Search extends Thread{

	private static double waypoints[][];
	private static final int RING_INBOUND = 25;
	private static final int ANGLE_ERROR = 5;
	private static final int ROTATE_SPEED = 40;
	private static final int FORWARD_SPEED = 120;
	
	
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
	
	public void run() {	
		
		//declare variables to be used
		double[] nextXY = new double[2];
		int[] result = {255,0};
		boolean ringAhead = false;
		int d=255;
		boolean detectedLastRound = false;
		
		// go to start of region
		Navigation.travelToNoCorrection(Lab5.LLx, Lab5.LLy);
		
		// create waypoints to travel to
		waypoints = createWayPoints(Lab5.LLx, Lab5.LLy, Lab5.URx, Lab5.URy);	
		
		
		//move to rest of waypoints, start at 1 cause 0 is start of SR
		for (int i = 1; i < waypoints.length; i++) {

			//move to next point, wait till finish
			nextXY = waypoints[i];

			
			Navigation.travelToNoCorrection(nextXY[0], nextXY[1]); 
			while(leftMotor.isMoving()) {
				d = (int)(UltrasonicPoller.usData[0]*100.0);
				if (d < RING_INBOUND) {
					ringAhead = true;
					break;
				}
				
			}
			while(leftMotor.isMoving());
			
			//check to see if there is a ring
			if(!ringAhead) {
				result = checkForRing();
			} else {
				result[1] = RingDetection.detectRing();
				result[0] = d;
			}

			//if its target ring go home
			if(result[1] == Lab5.TR) {
				goHome();
				break;
			}
			
			//if there is ring avoid
			if(result[1] != 0 && result[0] != 255) {
				avoidRing();

			}
		}
	}
	
	
	
	
	


	//assumes that the robot starts off with sensors on either side of the line 
	//it is traveling on i.e. its heading is around 90 or 270 degrees
	//ends robot on horizontal line
	private void avoidRing() {
		double theta = -1;
		boolean east = true;
		try {
			theta = Odometer.getOdometer().getXYT()[2];
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
		

		
	}


	private void correctAngle(boolean headingEast,boolean headingWest, boolean headingNorth, boolean headingSouth) {
		
		int foundLeft = 0; int foundRight = 0, correctAngle = 0;
		while(true) {
			//color sensor and scaling
			LightLocalizer.myColorSampleLeft.fetchSample(LightLocalizer.colorLeft, 0);
			LightLocalizer.myColorSampleRight.fetchSample(LightLocalizer.colorRight, 0);
			LightLocalizer.newColorLeft = LightLocalizer.colorLeft[0];
			LightLocalizer.newColorRight = LightLocalizer.colorRight[0];

			// If line detected for left sensor (intensity less than 0.4), only count once by keeping track of last value
			if((LightLocalizer.newColorLeft) < 0.4 && LightLocalizer.oldSampleLeft > 0.4 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((LightLocalizer.newColorRight) < 0.4 && LightLocalizer.oldSampleRight > 0.4 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}
			// Store last color samples
			LightLocalizer.oldSampleLeft = LightLocalizer.newColorLeft;
			LightLocalizer.oldSampleRight = LightLocalizer.newColorRight;
			
			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		if(headingNorth) {
			correctAngle = 0;
		} else if(headingSouth) {
			correctAngle = 180;
		}
		
		//correct odometer theta to correct one
		//if traveling east correct to 90, else west 270
		if(headingEast)
			correctAngle = 90;
		else if(headingWest)
			correctAngle = 270;
		try {
			Odometer.getOdometer().setTheta(correctAngle);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}


	private void goHome() {
		Navigation.travelToNoCorrection(Lab5.URx, Lab5.URy);
		
	}


	public int[] checkForRing() {
		int d = (int)(UltrasonicPoller.usData[0]*100.0);
		int[] result = {255, 0};
		Lab5.lcd.drawString("D: " + d, 0, 1);
		if (doubleCheck()) {
			d = (int)(UltrasonicPoller.usData[0]*100.0);
			result[0] = d;
			result[1] = RingDetection.detectRing(); 
			return result;
		}
		return result;
	}
	
	//the robot is before grid intersection
	//it might be off by an angle
	//scan + - ANGLE_ERROR degrees to account for this error
	public boolean doubleCheck() {
		boolean ringDetected = false;
		Navigation.moveStraight(leftMotor, rightMotor, 2, FORWARD_SPEED, true, false);
		if ((int)(UltrasonicPoller.usData[0]*100.0) < 10) {
			Navigation.moveStraight(leftMotor, rightMotor, 2, FORWARD_SPEED, false, false);
			return true;
		}
		Navigation.turnRobot(leftMotor, rightMotor, ANGLE_ERROR, ROTATE_SPEED, true, false);
		if ((int)(UltrasonicPoller.usData[0]*100.0) < 10) {
			Navigation.turnRobot(leftMotor, rightMotor, (-1*ANGLE_ERROR), ROTATE_SPEED, false, false);
			Navigation.moveStraight(leftMotor, rightMotor, 2, FORWARD_SPEED, false, false);
			return true;
		}
		Navigation.turnRobot(leftMotor, rightMotor, (-1*2*ANGLE_ERROR), ROTATE_SPEED, false, false);
		if ((int)(UltrasonicPoller.usData[0]*100.0) < 10) ringDetected = true;
		Navigation.turnRobot(leftMotor, rightMotor, ANGLE_ERROR, ROTATE_SPEED, true, false);
		if ((int)(UltrasonicPoller.usData[0]*100.0) < 10) ringDetected = true;
		Navigation.moveStraight(leftMotor, rightMotor, 2, FORWARD_SPEED, false, false);
		return ringDetected;
	}
	

	

	
	public static double[][] createWayPoints(int lx, int ly, int ux, int uy) {
		int numberOfWaypoints = (ux - lx + 1) * (uy - ly + 1);
		double[][] waypoints = new double[numberOfWaypoints][2];
		boolean east = true;
		int j = 0;
		for(int y = ly; y < uy + 1; y++) {
			if (east) {
				for(int x = lx; x < ux + 1; x++, j++) {
					waypoints[j][0] = x - 0.60;
					waypoints[j][1] = y;
				}
				east = false;
			} else {
				for(int x = ux; x > lx - 1; x--, j++) {
					waypoints[j][0] = x + 0.60;
					waypoints[j][1] = y;
				}
				east = true;
			}
		}
		return waypoints;
	}
	
}
