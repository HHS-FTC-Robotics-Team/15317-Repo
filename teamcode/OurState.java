

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class OurState extends OpMode
{
    //public HardwareMap hardwareMap = null;
    public Boolean running = true;
    public HardwareMap hardwareMap = null;
    //@Override
    public OurState() {
        //hardwareMap = hm;
    }
    
    @Override
    public void init() {
        
    }
    
    public void init(HardwareMap hm) {
        telemetry.addData("Status", "Initialized");
        hardwareMap = hm;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
    }

    @Override
    public void loop() {
        
    }

    @Override
    public void stop() {
    }
    
    public double getVariable() {
        return 0;
    }
    public void addToGoal(double variable) {
        //this page intentionally left blank.
    }

}
