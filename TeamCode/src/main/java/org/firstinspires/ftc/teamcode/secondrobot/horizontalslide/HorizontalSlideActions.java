package org.firstinspires.ftc.teamcode.secondrobot.horizontalslide;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

@Disabled
public class HorizontalSlideActions {
    public DcMotorEx armMotor = null;
    private Telemetry telemetry;
    double armLegLength = 10.75;
    double startDistance = 5.0;

    //set up the slide with all the mode and hardware map
    public HorizontalSlideActions(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_ARM);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    double prevTime = System.currentTimeMillis();
    boolean override = false;
    public void setOverride(boolean input) {
        override = input;
    }

    public void teleOpArmMotor(double power, double liftSpeedMultiplier) {
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
            double targetDistance = SlidePosition + power * (time - prevTime) * liftSpeedMultiplier;
//            targetDistance = Range.clip(targetDistance, 0, armLegLength * 2 - startDistance);
            targetDistance = Range.clip(targetDistance, 0, 600);
            setSlideDistance((int) targetDistance, 3000 * liftSpeedMultiplier);
            RobotLog.dd("horizontal arm", "Target Position %f, time %f", SlidePosition, time);
        }

        prevTime = time;
        telemetry.addData("target position HS", SlidePosition);
        telemetry.addData("current position HS", armMotor.getCurrentPosition());
    }
    double SlidePosition = 0;
    public void setSlideDistanceMath(int distance, double velocity) {
        double targetRadians = Math.acos(distance / (2 * armLegLength));
        double targetRotations = targetRadians / (2 * Math.PI);
        double targetEncoderTics = targetRotations / 2786.2;
        armMotor.setTargetPosition((int) targetEncoderTics);
        armMotor.setVelocity(velocity);
        SlidePosition = distance;
    }
    public void setSlideDistance(int distance, double velocity) {
        armMotor.setTargetPosition(distance);
        armMotor.setVelocity(velocity);
        SlidePosition = distance;
    }

    public int getSlidePosition() {
        return armMotor.getCurrentPosition();
    }
}
