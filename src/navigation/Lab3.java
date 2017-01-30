package navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3
{

	//Motors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	//Sensors
	private static final Port usPort = LocalEV3.get().getPort("S1");
	//Constants
	public static final double WHEEL_RADIUS = 2.0;
	public static final double TRACK = 10.5; //10.5 lower tilts to the left, higher to the right
	public static final int motorStraight = 150;

	public static void main(String[] args)
	{
		int buttonChoice;

		//Instaniate objects
		final TextLCD t = LocalEV3.get().getTextLCD();

		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];


		Odometer odometer = new Odometer(leftMotor, rightMotor);
		final TriangleDriver avoidObstacles = new TriangleDriver(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK, motorStraight);
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, leftMotor, rightMotor,avoidObstacles);

		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t, usPoller);
		final PatternDriver noObstacles = new PatternDriver(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK, motorStraight);

		do
		{
			t.clear();

			t.drawString("<    Left | Right  > ", 0, 0);
			t.drawString("          |          ", 0, 1);
			t.drawString(" Navigate | Navigate ", 0, 2);
			t.drawString("  Without | With     ", 0, 3);
			t.drawString(" Obstacle | Obstacle ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT 
				&& buttonChoice != Button.ID_RIGHT);


		if (buttonChoice == Button.ID_LEFT)
		{ 
			//Start the odometer, run the pattern driver, no obsticles
			odometer.start();
			odometryDisplay.start();
			noObstacles.start();
		} else {
			//Start the odometer, triangle driver, us sensor
			usPoller.start();
			odometer.start();
			odometryDisplay.start();
			avoidObstacles.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
