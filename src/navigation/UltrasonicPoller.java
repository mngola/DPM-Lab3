package navigation;

import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread{
	private SampleProvider us;
	private float[] usData;
	int distance;
	private TriangleDriver controller;

	public UltrasonicPoller(SampleProvider ultSens, float[] usD, TriangleDriver td)
	{
		us = ultSens;
		usData = usD;
		controller = td;
	}

	public void run()
	{
		while (true)
		{
			us.fetchSample(usData, 0);
			distance = ((int)(usData[0] * 100.0));
			controller.processUSData(distance);
			try { Thread.sleep(30); } catch (Exception localException) {}
		}
	}
}
