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
      
      // TODO Trigger correction (When do I have information to correct?)
      /**
       * This updates color sensor data into a float variable
       * as well as the most recent location data
       */
      Sensor.fetchSample(sensorData, 0);
      float dataSample = sensorData[0];
      //boolean correctLocation = false;
      double[] location = new double[3];
      location = odometer.getXYT();
      
      
      
      
      /**
       * The following conditions will only occur if the location should be corrected
       * The conditions are split up by vertical and horizontal orientations
       */
      
      
      double yLine = (location[1] + sensorOffset) / 15.24;
      double xLine = (location[0] + sensorOffset ) / 15.24;
      double angle = location[2] % 360;
      
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
    	  
    	  else if(yLine > 1.5 && yLine < 2.5) {
    		  odometer.setY(30.48-sensorOffset);
    		  Sound.beep();
    	  }
    	  
    	  else if( yLine > 3.5 && yLine < 4.5) {
    		  odometer.setY(60.96-sensorOffset);
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
    	  
    	  else if(xLine > 1.5 && xLine < 2.5) {
    		  odometer.setX(30.48 - sensorOffset);
    		  Sound.beep();
    	  }
    	  
    	  else if(xLine > 3.5 && xLine < 4.5) {
    		  odometer.setX(60.96 - sensorOffset);
    		  Sound.beep();
    	  }
      
      // TODO Update odometer with new calculated (and more accurate) vales

      //odometer.setXYT(0.3, 19.23, 5.0);

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
