package org.firstinspires.ftc.teamcode.secondrobot.verticalslide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalSlideActions {
    public DcMotorEx verticalSlide = null;
    public DcMotorEx verticalSlide2 = null;
    public TouchSensor magnetSwitch = null;
    private Telemetry telemetry;

    //set up the slide with all the mode and hardware map
    public VerticalSlideActions(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        verticalSlide = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);

        verticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setTargetPosition(0);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide2.setTargetPosition(0);
        verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        magnetSwitch = hardwareMap.get(TouchSensor.class, ConfigConstants.VSLIDE_MAGNETIC_LIMIT_SWITCH);
    }

    public void resetSlides(boolean resetSlides) {
        if (resetSlides){
            if (magnetSwitch.isPressed()){
                verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalSlide.setPower(-0.5);
                verticalSlide2.setPower(-0.5);
            }
        }
    }

    double prevTime = System.currentTimeMillis();
    public void teleOpVerticalSlide(double power, double liftSpeedMultiplier) { //  controls the lifty uppy (viper slides) which is being extended and retracted
        double time = System.currentTimeMillis();
        if (power != 0) {
            //if line 55 is true then check if the motor is using run to encoder
            if (verticalSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

                //if line 57 is true, then turn the motor to run to position which keep the slide stable
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.0);
                verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide2.setPower(0.0);
            }

            //change the slide position by the input power times the change in time times the speed.
            //Multiplying by change in time makes sure the slide speed is more consistent
            double total = SlidePosition + power * (time - prevTime) * liftSpeedMultiplier;
            total = Range.clip(total, 0, ConfigurationSecondRobot.topBasket);
            setSlidePosition((int) total, 3000 * liftSpeedMultiplier);
            RobotLog.dd("LiftyUppy", "Target Position %f, time %f", SlidePosition, time);
        }
        prevTime = time;
        telemetry.addData("target position", SlidePosition);
        telemetry.addData("liftyPower", verticalSlide.getPower());

        telemetry.addData("Current VS1", verticalSlide.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current VS2", verticalSlide2.getCurrent(CurrentUnit.MILLIAMPS));
    }

    boolean at1 = false;
    double at1StartTime = 0;
    int preset1 = ConfigurationSecondRobot.highBar;
    int preset2 = ConfigurationSecondRobot.bottom;
    //todo is this right?
    int preset3 = ConfigurationSecondRobot.lowBar;
    int preset4 = ConfigurationSecondRobot.topBasket;
    boolean wasSet = false;
    boolean wasDown = true;
    boolean isDown = true;

    //Slide goes to preset that is set as true
    public void goToPreset(boolean bottomRung, boolean bottom, boolean topRung, boolean topBasket) {
        if (bottomRung || bottom || topRung || topBasket){
            if (verticalSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!wasSet) {
                //one shot- when button is pressed, it only sets the position once
                if (bottomRung) {
                    setSlidePosition(preset1, 1800);
                }
                if (bottom) {
                    setSlidePosition(preset2, 2500);
                } else if (topRung) {
                    setSlidePosition(preset3, 2500);
                } else if (topBasket) {
                    setSlidePosition(preset4, 2500);
                }
            }
            wasSet = true;
        } else {
            wasSet = false;
        }
    }

    public void turnOffAtBottom() {
        //If the slide is at the bottom, wait 200 millis to cancel bouncing and then turn off the motors to save power
        isDown = (verticalSlide.getCurrentPosition() < 5);
        if(isDown && !wasDown) {
            //when the slide reaches the bottom, start a timer
            at1 = true;
            at1StartTime = System.currentTimeMillis();
        }
        wasDown = isDown;
        if ((System.currentTimeMillis() > (at1StartTime + 200)) && at1 && isDown) {
            //when the timer is done, turn off the motors
            at1 = false;
            verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalSlide.setPower(0.0);
            verticalSlide2.setPower(0.0);
        }
        telemetry.addData("vert position", verticalSlide.getCurrentPosition());
        telemetry.addData("mode", verticalSlide.getMode());
    }

    double SlidePosition = 0;
    public void setSlidePosition(int position, double velocity) {
        verticalSlide.setTargetPosition(position);
        verticalSlide.setVelocity(velocity);
        verticalSlide2.setTargetPosition(position);
        verticalSlide2.setVelocity(velocity);
        SlidePosition = position;
    }
    public double getSlidePosition() {
        return SlidePosition;
    }

}
