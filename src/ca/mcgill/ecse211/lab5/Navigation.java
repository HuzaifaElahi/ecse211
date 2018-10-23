package ca.mcgill.ecse211.lab5;
import ca.mcgill.ecse211.lab5.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * This is the Navigation class which extends Thread and implements
 * Runnable and Ultrasonic Controller. It uses the Ultrasonic sensor
 * to handle cases where an object is detected in the path of the robot 
 * and otherwise travels to the selected waypoints that are to it by
 * Controller in the NavWithObstacle constructor
 * 
 * @author Huzaifa, Jake
 * 
 */
public class Navigation {



	// Parameters: Can adjust these for desired performance
	private static final int MOTOR_HIGH = 80;     // Speed of the faster rotating wheel (deg/seec)
	private static final int ROTATE_SPEED = 50;   // Speed upon rotation
	private final static double ODOMETER_ADJUSTMENT = 0.5;    // Adjusts the inaccuracy of the odometer
	//Motors initialized
	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	// Variables for odometer
	Odometer odometer = null;
	private static double prevAngle = 0;
	static boolean navigating = false;
	/**
	 * Contructor, takes in and sets path passed by user
	 * selection in Controller class
	 * 
	 * @param finalPath
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		Navigation.leftMotor = leftMotor;
		Navigation.rightMotor = rightMotor;
		try {
			this.odometer = Odometer.getOdometer(Navigation.leftMotor, Navigation.rightMotor, Lab5.TRACK, Lab5.WHEEL_RAD);
		} catch (OdometerExceptions e) {
			System.out.println("no odo");
		}
	}
	/**
	 * This method makes robot move in the direction of the
	 * waypoint whose coordinates are passed as arguments
	 * 
	 * @param x
	 * @param y
	 * @return void
	 */ 
	public static void travelTo(double x, double y) {
		// Define variables
		double odometer[] = { 0, 0, 0 }, absAngle = 0, dist = 0, deltaX = 0, deltaY = 0;

		// Set navigating to true
		navigating = true;

		// Get odometer readings
		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (Exception e) {
			// Do nothing lol
			e.printStackTrace();
		}

		x = x*Lab5.SQUARE_SIZE;
		y = y*Lab5.SQUARE_SIZE;

		// Set odometer reading angle as prev angle as well
		prevAngle = odometer[2];

		// Get displacement to travel on X and Y axis
		deltaX = x - odometer[0];
		deltaY = y - odometer[1];

		// Displacement to point (hypothenuse)
		dist = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		// Get absolute angle the robot must be facing
		absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		// If the value of absolute angle is negative, loop it back
		if (absAngle < 0)
			absAngle = 360 - Math.abs(absAngle);

		// Make robot turn to the absolute angle
		turnTo(absAngle);

		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		
		leftMotor.setSpeed(MOTOR_HIGH);
		rightMotor.setSpeed(MOTOR_HIGH);
		
		LightLocalizer.oldSampleLeft = 0;
		LightLocalizer.oldSampleRight = 0;
		leftMotor.forward();
		rightMotor.forward();
		
		int foundLeft = 0; int foundRight = 0;
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
		
		// Move forward beyond line by length of offset so turning point is directly above line
		leftMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, LightLocalizer.SENSOR_OFFSET), true);
		rightMotor.rotate(Navigation.convertDistance(Lab5.WHEEL_RAD, LightLocalizer.SENSOR_OFFSET), false);

	}

	/**
	 * This method causes robot to travel a distance based on the
	 * hypotenus value obtained from performing pythagoras
	 * theorem on x and y
	 * @param x
	 * @param y
	 * @throws OdometerExceptions
	 */
	static void travelToHypot(double x, double y) throws OdometerExceptions {
		double calcTheta = 0, distance = 0, deltaX = 0, deltaY = 0;

		double[] odometer = Odometer.getOdometer().getXYT();

		double odoAngle = odometer[2];

		deltaX = x*Lab5.SQUARE_SIZE- odometer[0];;
		deltaY = y*Lab5.SQUARE_SIZE - odometer[1];

		distance = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));
		calcTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));

		//if result is negative subtract it from 360 to get the positive
		if (calcTheta < 0)
			calcTheta = 360 - Math.abs(calcTheta);

		// turn to the found angle
		turnTo(calcTheta);


		// go
		leftMotor.setSpeed(Navigation.MOTOR_HIGH);
		rightMotor.setSpeed(Navigation.MOTOR_HIGH);
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);


	}


	/**
	 * method uses gyro to ensure that we turn the specified corrected amount
	 * @param theta
	 */
	public static void correctedTurnTo(double theta) {
		boolean turnLeft = false;
		double[] odometer = null;
		double delTheta;
		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e1) {
		}
		// If deltaAngle is negative, loop it back
		if (theta < 0) {
			theta = 360 - Math.abs(theta);
		}
		// Check if we want to move left or right
		if (theta > 180) {
			turnLeft = true;
			theta = 360 - Math.abs(theta);
		} else {
			turnLeft = false;
		}
		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);


		while(true) {
			try {
				odometer = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e) {
			}
			if(((odometer[2] < theta+2.5) && (odometer[2] > theta-2.5))) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
			// Turn motors according to which direction we want to turn in
			if (turnLeft) {
				leftMotor.backward();
				rightMotor.forward();
			} else {
				leftMotor.forward();
				rightMotor.backward();
			}
		}
	}
	/**
	 * This method causes the robot to turn (on point) to the relative heading theta
	 * (turn by)
	 * @param theta
	 * @return void
	 */ 

	/**
	 * This method causes the robot to turn (on point) to the relative heading theta
	 * (turn by)
	 * @param theta
	 * @return void
	 */ 
	public static void turnTo(double theta) {
		boolean turnLeft = false;
		double deltaAngle = 0;
		// Get change in angle we want
		deltaAngle = theta - prevAngle;

		// If deltaAngle is negative, loop it back
		if (deltaAngle < 0) {
			deltaAngle = 360 - Math.abs(deltaAngle);
		}

		// Check if we want to move left or right
		if (deltaAngle > 180) {
			turnLeft = true;
			deltaAngle = 360 - Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Turn motors according to which direction we want to turn in
		if (turnLeft) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), false);
		}

	}

	/**
	 * This method uses the current angle as a reference
	 * to turn to the angle required
	 * @param theta: angle to turn by
	 * @throws OdometerExceptions
	 */
	public static void turnWithTheta(double theta) throws OdometerExceptions {

		boolean turnLeft = false;
		double deltaAngle = 0;
		// Get change in angle we want
		prevAngle = Odometer.getOdometer().getXYT()[2];
		deltaAngle = theta - prevAngle;

		// If deltaAngle is negative, loop it back
		if (deltaAngle < 0) {
			deltaAngle = 360 - Math.abs(deltaAngle);
		}

		// Check if we want to move left or right
		if (deltaAngle > 180) {
			turnLeft = true;
			deltaAngle = 360 - Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Turn motors according to which direction we want to turn in
		if (turnLeft) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaAngle), false);
		}

	}

	/**
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param distance : distance to travel
	 * @param forwards : if true then it goes forward direction
	 * @param continueRunning : if true then program does not wait for wheels to stop, false program waits  
	 */
	public static void moveStraight(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double distance, int speed, boolean forwards, boolean continueRunning) {
		int i = 1;
		if (!forwards) i = -1;
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, i * distance), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, i *distance), continueRunning);
	}


	/**
	 * This method turns the robot to the right or left depending on direction boolean and turns the robot by the specified
	 * degrees amount.
	 * @param left : motor
	 * @param right : motor
	 * @param degrees : degrees to turn by
	 * @param direction : true means turn right, left otherwise
	 */
	public static void turnRobot(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right, 
			int degrees, int speed, boolean direction, boolean continueRunning) {
		int i = 1;
		if (!direction)
			i = -1;
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.rotate(i * convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, degrees), true);
		rightMotor.rotate(i * -convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, degrees), continueRunning);
	}

	/**
	 * This method returns the static boolean, navigating
	 * 
	 * @return boolean
	 */
	public static boolean isNavigating() {
		return navigating;
	}

	/**
	 * This method is a helper that allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return int
	 */
	static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle and it's calculated distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return int
	 */
	static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}