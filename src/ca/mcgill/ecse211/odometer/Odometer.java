/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import java.lang.Math;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double[] position;
  
  private double thetaH;
  private double theta;
  
  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
	  
    if (odo != null) { // Return existing object
      return odo;
    } 
    else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
   
    while (true) {
      updateStart = System.currentTimeMillis();

      /*
       * This saves the previous measurements from the motors Tacho
       * allows the code to calculate changes in the rotation of the wheels per iteration
       */
      int oldLeftMotorTachoCount = leftMotorTachoCount;
      int oldRightMotorTachoCount = rightMotorTachoCount;
    
      
      
     /*
      * This grabs the new Tacho count from both left and right motors
      * stores them in a separate variable from the old tacho count
      */
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      /*
       * Change in motor degree is the difference between the new and old Tacho counts
       */
      double thetaL = leftMotorTachoCount - oldLeftMotorTachoCount;
      double thetaR = rightMotorTachoCount - oldRightMotorTachoCount;
      
      
      /*
       * the change in displacement of the right and left sides is calculated below
       */
      double d1 = (WHEEL_RAD*3.14159*thetaL)/180;
      double d2 = (WHEEL_RAD*3.14159*thetaR)/180;
      
      /*
       * theta is used to calculate the change in heading direction
       * the difference in right and left displacement divided by the difference between the right and left wheels
       */
      double d = d2 - d1;
      double theta = d/TRACK;
      
      /*
       * new heading is calculated by old one plus the change in heading
       */
      thetaH = thetaH + theta;
      
      /*
       * the change in x and why displacements are calculated  here using the change in heading
       * and the absolute location is updated just below
       * using the odo.update method
       */
      double dH = (d1 + d2)/2;
      double dx = -dH*Math.sin(thetaH);
      double dY = dH*Math.cos(thetaH);
      
      odo.update(dx, dY, theta*180/3.14159);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
