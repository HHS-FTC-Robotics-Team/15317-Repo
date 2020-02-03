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

import org.firstinspires.ftc.teamcode.OurState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

public class DragFoundationR extends OurState {
    /* Declare OpMode members. */
    public Drive d = null;
    private BNO055IMU imu = null;
    double theta = 0;
    double dtheta = 90;
    private Orientation lastAngles = null;
    private double globalAngle, power = .30, correction;
    private Servo left = null;
    private double lmax = .21; // up
    private double lmin = .7; //down
    private double lmid = .5;
    private double servogoal = lmin;
    private Servo f = null;
    private double anglegoal = 0;
    
    public DragFoundationR(double a){
        super ();
        anglegoal = a;
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
        
        f = hardwareMap.get(Servo.class, "foundation");
        telemetry.addData("Status", "Initialized");
        d.resetEncoderlf();
        theta = getAngle();
        dtheta = anglegoal + theta;
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
      if(running) {
        f.setPosition(servogoal);
        theta = getAngle();
        d.setPower(0, 0, (dtheta - theta) / (Math.abs(dtheta - theta)) , 0.6);
        if(Math.abs(theta - dtheta) < 2) { //if diff is less than 2 degrees
          running = false;
          d.setPower(0,0,0,0);
          this.stop();
        }
      } else {
        f.setPosition(lmax);
      }
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

      globalAngle += deltaAngle;

      lastAngles = angles;

      return globalAngle;
    }
}