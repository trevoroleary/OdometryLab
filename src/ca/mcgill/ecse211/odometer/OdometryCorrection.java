/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
//import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  
  
  
  private SensorModes Sensor;
  private SampleProvider sensorSample;
  private float[] sensorData;
  private double sensorOffset = 4.5;
  private double TILE_SIZE = 30.48;
  
  private boolean firstLineY = true;
  private boolean firstLineX = true;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(Odometer odometer) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.Sensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    this.sensorSample = Sensor.getMode("ColorID");
    this.sensorData = new float[Sensor.sampleSize()];

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();
      
      /**
       * coordinates are all based on where the center of the wheel base is located
       * 
       */
      
      /*
       * Following variables used to store the sensor data
       * sensor returns data as SampleProvider
       * Object used to return data in an array 'dataSample
       * location of the odometer is also stored
       */
      Sensor.fetchSample(sensorData, 0);
      float dataSample = sensorData[0];
      double[] location = new double[3];
      location = odometer.getXYT();     
      
      /*
       * These double variables are used to determine the distance from a black line
       * later on if these variables are too far from an integer when a black line is seen
       * the robot decides that it was not actually a line
       * 15.24 is half the length of a square
       */
      double yLine = (location[1] + sensorOffset) / TILE_SIZE;
      double xLine = (location[0] + sensorOffset ) / TILE_SIZE;
      double angle = location[2] % 360;
      
      
      /*
       * The sensor is placed ahead of the wheel base
       * if the sensor detects a line the wheel base is 4.5cm behind
       * these conditions decide weather the offset should be positive or negative
       * depending on the direction the robot if facing.
       */
      if((angle < 20 || angle > 340) || (angle < 290 && angle > 250)) {
    	  sensorOffset = 4.5;
      }
      else {
    	  sensorOffset = -4.5;
      }
      
      //Correction for North & South orientations
      
      if(dataSample > 10 && ( (angle < 20 || angle > 340) || ((angle < 200 && angle > 160 )))) {
    	  if(firstLineY) {
    		  odometer.setY(-sensorOffset);
    		  firstLineY = false;
    		  Sound.beep();
    	  }
    	  else if(yLine > 0.7 && yLine < 1.3) {
    		  odometer.setY(TILE_SIZE-sensorOffset);
    		  Sound.beep();
    	  }
    	  else if( yLine > 1.7 && yLine < 2.3) {
    		  odometer.setY((TILE_SIZE*2)-sensorOffset);
    		  Sound.beep();
    	  }
      }
      
      //Correction for East & West orientations
      if(dataSample > 10 && ((angle < 290 && angle > 250) || (angle < 110 && angle > 70)))
    	  if(firstLineX) {
    		  odometer.setX(-sensorOffset);
    		  firstLineX = false;
    		  Sound.beep();
    	  }
    	  else if(xLine > 0.7 && xLine < 1.3) {
    		  odometer.setX(TILE_SIZE - sensorOffset);
    		  Sound.beep();
    	  }
    	  else if(xLine > 1.7 && xLine < 2.3) {
    		  odometer.setX((TILE_SIZE*2) - sensorOffset);
    		  Sound.beep();
    	  }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
