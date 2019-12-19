package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Collect extends LinearOpMode {

  //init the two motors and distance sensor
  private DcMotor Lmtr = null;
  private DcMotor Rmtr = null;
  private Rev2mDistanceSensor dist = null;

  public Collect (DcMotor l, DcMotor r, Rev2mDistanceSensor d) {
    Lmtr = l;
    Rmtr = r;
    //direction for one is reversed so that
    //the collectors can suck bricks in and out
    Lmtr.setDirection(DcMotor.Direction.FORWARD);
    Rmtr.setDirection(DcMotor.Direction.REVERSE);
    dist = d;
  }

  public void in() {
    Lmtr.setPower(0.8);
    Rmtr.setPower(0.8);
  }

  public void out() {
    Lmtr.setPower(-0.8);
    Rmtr.setPower(-0.8);
  }

  public void rest() {
    Lmtr.setPower(0);
    Rmtr.setPower(0);
  }

  //the sensor reads up to 2 meters.
  //in gpsbrain, once the distance sensor is under 10sm, the motors shut off.
  public double getDistance() {
    double distance = dist.getDistance(DistanceUnit.CM);
    return distance;
  }

  public String getPower() {
    double lp = Lmtr.getPower();
    double rp = Rmtr.getPower();
    String s = lp + " / " + rp;
    return s;
  }

  public void runOpMode() {

  }
}
