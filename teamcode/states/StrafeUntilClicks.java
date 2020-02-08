/*
Copyright 2020 FIRST Tech Challenge Team 15317
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.OurState;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class StrafeUntilClicks extends OurState {
  
    /* Declare OpMode members. */
    public Drive d = null;
    public double goal = 0;
    public double current = 0;
    private static double ACCURACY = 100;
    //imu
    private BNO055IMU imu = null;
    double globala = 0;
    private Orientation lastAngles = null;
    private double globalAngle, power = .30, correction;
    
    public StrafeUntilClicks(int g) {
      super();
      goal = g;
      
    }
    
    
    @Override
    public void init(HardwareMap hm) {
        hardwareMap = hm;
        d = new Drive(
        hardwareMap.get(DcMotor.class, "rbmotor"),
        hardwareMap.get(DcMotor.class, "rfmotor"),
        hardwareMap.get(DcMotor.class, "lfmotor"),
        hardwareMap.get(DcMotor.class, "lbmotor")
      );
      //imu
      lastAngles = new Orientation();
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
      imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        d.resetEncoderlf();
        globala = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
      // if (goal < 0) {
      //   d.setPower(0, -1, 0, 0.4);
      // } else {
      //   d.setPower(0, 1, 0, 0.4);
      // }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    
    @Override
    public void loop() {
      current = d.getClickslf();
      if(current > goal - ACCURACY && current < goal + ACCURACY) {
          d.setPower(0, 0, 0, 0);
          running = false;
      } else if (current > goal) {
        d.setPower(0, 0.5, 0, Math.abs(goal)-Math.abs(current)/Math.abs(goal) * 0.4);
      } else if (current < goal) {
        d.setPower(0, -0.5, 0, Math.abs(goal)-Math.abs(current)/Math.abs(goal) * 0.4);
      }
      
      globala = globala + getAngle();
      //correcting
      // double current = getAngle();
      double power =  -1 * (globala) / Math.abs(globala);
      if (globala > 0.25 || globala < -0.25) {
        d.setPower(d.getLy(), d.getLx(), power/2 , d.getTurbo());
      } else {
        d.setPower(d.getLy(), d.getLx(), 0, d.getTurbo());
        globala = 0;
      }
      // if (globala < -1) {
      //   d.setPower(d.getLy(), d.getLx(), power/2 , d.getTurbo());
      // }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    
    public double getAngle() {
      //this function and the note below taken from somewhere else

      // We experimentally determined the Z axis is the axis we want to use for heading angle.
      // We have to process the angle because the imu works in euler angles so the Z axis is
      // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
      // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

      Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

      if (deltaAngle < -180)
          deltaAngle += 360;
      else if (deltaAngle > 180)
          deltaAngle -= 360;

      //globalAngle += deltaAngle;

      lastAngles = angles;

      return deltaAngle;
    }
    
    @Override
    public void addToGoal(double variable) {
      goal = goal - variable; //used with seeking
    }
}
