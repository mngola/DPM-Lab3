package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class TriangleDriver extends Thread {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private boolean isNavigating = true;
	private final double WHEEL_RADIUS;
	private final double TRACK;
	private double odTheta;
	private double odX;
	private double odY;
	private final int motorStraight;

	public TriangleDriver(Odometer od, EV3LargeRegulatedMotor lm, EV3LargeRegulatedMotor rm, double wr, double t, int ms)
	{
		leftMotor = lm;
		rightMotor = rm;
		odometer = od;
		WHEEL_RADIUS = wr;
		TRACK = t+0.2;
		motorStraight = ms;
	}

	public void run()
	{
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(100);
		}

		while(!Thread.interrupted() && isNavigating)
		{
			travelTo(0.0, 60.0);
			travelTo(60.0, 0.0);
		}
	}

	public void travelTo(double x, double y)
	{
		isNavigating = true;
		
		odTheta = odometer.getTheta();
		odX = odometer.getX();
		odY = odometer.getY();

		double dX = x - odX;
		double dY = y - odY;

		double destTheta = Math.atan2(dY,dX);
		turnTo((Math.PI/2.0) - destTheta);

		double dist = Math.sqrt(dX * dX + dY * dY);

		leftMotor.setSpeed(motorStraight+100);
		rightMotor.setSpeed(motorStraight+100);

		leftMotor.rotate(convertDistance(WHEEL_RADIUS, dist), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, dist), false);
		
		isNavigating = false;
	}

	public void turnTo(double destTheta)
	{
		double dTheta = destTheta - odTheta;
		if (dTheta < -Math.PI) {
			dTheta += 2*Math.PI;
		} else if (dTheta > Math.PI) {
			dTheta -= 2*Math.PI;
		}

		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);

		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, dTheta), false);
	}

	public boolean isNavigating()
	{
		return isNavigating;
	}
	
	public void setNavigating(boolean state)
	{
		isNavigating = state;
	}

	public void runWallFollower(double x, double y)
	  {
	    turnTo(odometer.getTheta() + 50.0D);
	    
	    double startX = odometer.getX();
	    double startY = odometer.getY();
	    
	    double lineB = x - startX;
	    double lineA = y - startY;
	    double lineC = lineB * startY - lineA * startX;
	    
	    double distFromStart = 0.0D;
	    double distToLine = 0.0D;
	    while ((distToLine > 1.0D) || (distFromStart < 5.0D))
	    {
	      double distance = 23;
	      
	      this.leftMotor.setSpeed(this.motorStraight);
	      this.leftMotor.forward();
	      if (distance <= 23.0D)
	      {
	        this.rightMotor.setSpeed(20);
	        this.rightMotor.forward();
	      }
	      else if (distance >= 27.0D)
	      {
	        this.rightMotor.setSpeed(280);
	        this.rightMotor.forward();
	      }
	      else
	      {
	        this.rightMotor.setSpeed(this.motorStraight);
	        this.rightMotor.forward();
	      }
	      this.odTheta = this.odometer.getTheta();
	      this.odX = this.odometer.getX();
	      this.odY = this.odometer.getY();
	      
	      distFromStart = Math.sqrt((startX - this.odX) * (startX - this.odX) + (startY - this.odY) * (startY - this.odY));
	      distToLine = Math.abs(lineA * this.odX - lineB * this.odY + lineC) / Math.sqrt(lineA * lineA + lineB * lineB);
	    }
	  }
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, width * angle / 2.0);
	}
}
