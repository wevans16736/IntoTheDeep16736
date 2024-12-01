package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
public class ArmActions {
    public DcMotorEx armMotor = null;
    private Telemetry telemetry;

    //set up the slide with all the mode and hardware map
    public ArmActions(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    }
   public void teleopArmmotor(boolean gamepad2b,boolean gamepad2a){

       if (gamepad2b){
           armMotor.setPower(0.5);

       } else if (gamepad2a) {
           armMotor.setPower(-0.5);

       } else {
           armMotor.setPower(0);
       }

   }

}
