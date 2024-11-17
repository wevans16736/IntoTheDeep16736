package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmActions {
    public DcMotorEx armMotor = null;
    private Telemetry telemetry;

    //set up the slide with all the mode and hardware map
    public ArmActions(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //set the inital slide encoder count to 0 when intizilize
    public void resetSlide() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    double prevTime = System.currentTimeMillis();
    boolean override = false;
    public void setOverride(boolean input) {
        override = input;
    }

    //  controls the lifty uppy (viper slides) which is being extended and retracted
    public void teleOpArm(double power, double liftSpeedMultiplier) {
        double time = System.currentTimeMillis();
        //check if the joystick is being use and the overide is off
        if (power != 0 && !override) {
            //if line 55 is true then check if the motor is using run to encoder
            if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                //if line 57 is true, then turn the motor to run to position which keep the slide stable
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
            }
            //change the slide position by the input power times the change in time times the speed.
            //Multiplying by change in time makes sure the slide speed is more consistent
            double total = SlidePosition + power * (time - prevTime) * liftSpeedMultiplier;
            total = Range.clip(total, -1600, 1600);
            setArmPosition((int) total, 3000 * liftSpeedMultiplier);
            RobotLog.dd("LiftyUppy", "Target Position %f, time %f", SlidePosition, time);
        }

        prevTime = time;
        telemetry.addData("target position", SlidePosition);

        telemetry.addData("current position", armMotor.getCurrentPosition());
    }

    double SlidePosition = 0;

    public void setArmPosition(int position, double velocity) {
        armMotor.setTargetPosition(position);
        armMotor.setVelocity(velocity);
        SlidePosition = position;
    }
    public double getArmPosition() {
        return SlidePosition;
    }

}
