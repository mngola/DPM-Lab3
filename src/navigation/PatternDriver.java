package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PatternDriver extends Thread {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private boolean isNavigating = false;
	private final double WHEEL_RADIUS;
	private final double TRACK;
	private double odTheta;
	private double odX;
	private double odY;
	private final int motorStraight;

	public PatternDriver(Odometer od, EV3LargeRegulatedMotor lm, EV3LargeRegulatedMotor rm, double wr, double t, int ms)
	{
		leftMotor = lm;
		rightMotor = rm;
		odometer = od;
		WHEEL_RADIUS = wr;
		TRACK = t;
		motorStraight = ms;
	}

	public void run()
	{
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(100);
		}
			travelTo(60.0, 30.0);
			travelTo(30.0, 30.0);
			travelTo(30.0, 60.0);
			travelTo(60.0, 0.0);
	}

	public void travelTo(double x, double y)
	{
		isNavigating = true;
		
		//Get Odometer readings
		odTheta = odometer.getTheta();
		odX = odometer.getX();
		odY = odometer.getY();

		//Compute the distance from the current poin to where you want to go
		double dX = x - odX;
		double dY = y - odY;

		//Use the distances to comput the angle and turn to that angle 
		double destTheta = Math.atan2(dY,dX);
		turnTo((Math.PI/2.0) - destTheta);

		//Compute the travel distance 
		double dist = Math.sqrt(dX * dX + dY * dY);

		leftMotor.setSpeed(motorStraight+100);
		rightMotor.setSpeed(motorStraight+100);

		leftMotor.rotate(convertDistance(WHEEL_RADIUS, dist), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, dist), false);

		isNavigating = false;
	}

	public void turnTo(double destTheta)
	{
		//Compute the amount you need to turn
		//Find the shortest angle for that angle
		double dTheta = destTheta - odTheta;
		if (dTheta < -Math.PI) {
			dTheta += 2*Math.PI;
		} else if (dTheta > Math.PI) {
			dTheta -= 2*Math.PI;
		}

		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);

		//rotate to the angle
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, dTheta), false);
	}

	public boolean isNavigating()
	{
		return this.isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, width * angle / 2.0);
	}
}
