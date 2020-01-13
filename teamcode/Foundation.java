package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class Foundation extends LinearOpMode {

  private Servo left = null;
  private double lmax = .21; // Maximum rotational position
  private double lmin = .7; // Minimum rotational position
  private double lmid = .5;

  private String currentPos = "mid";

  public Foundation (Servo l) {
    left = l;
  }

  public void grab() {
    left.setPosition(lmin);
  }
  public void mid() {
    left.setPosition(lmid);
  }
  public void release() {
    left.setPosition(lmax);
  }
  public void alldown() {
    left.setPosition(.85);
  }

  public void nextPos() {
    if(currentPos == "down") {
      currentPos = "up";
      release();
    } else if(currentPos == "up") {
      currentPos = "mid";
      mid();
    } else if(currentPos == "mid") {
      currentPos = "down";
      grab();
    }
  }

  public void runOpMode() {

  }
}
