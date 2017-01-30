package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread{
	private SampleProvider us;
	private float[] usData;
	int distance;
	private int bandCenter=18;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Thread controller;

	public UltrasonicPoller(SampleProvider ultSens, float[] usD, EV3LargeRegulatedMotor lm, EV3LargeRegulatedMotor rm, Thread driver)
	{
		us = ultSens;
		usData = usD;
		leftMotor = lm;
		rightMotor = rm;
		controller = driver;
	}

	public void run()
	{
		while (true)
		{
			us.fetchSample(usData, 0);
			distance = ((int)(usData[0] * 100.0));
			processUSData(distance);
			try { Thread.sleep(50); } catch (Exception localException) {}
		}
	}

	private void processUSData(int distance2) {
		if(distance2 < bandCenter)
		{
			try {
				controller.wait();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			//leftMotor.stop();
			//rightMotor.stop();
		}
	}

}
