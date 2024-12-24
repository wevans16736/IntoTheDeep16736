package org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configuration.ConfigurationFirstRobot;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class VerticalSlideActions {

    public DcMotorEx VerticalSlide1 = null;
    public DcMotorEx VerticalSlide2 = null;
    private Telemetry telemetry;

    public VerticalSlideActions(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        VerticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        VerticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
        VerticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalSlide1.setTargetPosition(0);
        VerticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        VerticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
        VerticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
        VerticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalSlide2.setTargetPosition(0);
        VerticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetSlide() {
        VerticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalSlide1.setTargetPosition(0);
        VerticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        VerticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalSlide2.setTargetPosition(0);
        VerticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    double prevTime = System.currentTimeMillis();

    public void teleOpVerticalSlide(double power, double liftSpeedMultiplier) { //  controls the lifty uppy (viper slides) which is being extended and retracted
        double time = System.currentTimeMillis();
        if (power != 0) {
            //if line 55 is true then check if the motor is using run to encoder
            if (VerticalSlide1.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//                HorizontalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                HorizontalSlide1.setPower(1.0);

                //if line 57 is true, then turn the motor to run to position which keep the slide stable
                VerticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                VerticalSlide1.setPower(1.0);
            }
//            double time = System.currentTimeMillis();

            //change the slide position by the input power times the change in time times the speed.
            //Multiplying by change in time makes sure the slide speed is more consistent
            double total = SlidePosition + power * (time - prevTime) * liftSpeedMultiplier;
            total = Range.clip(total, -830, 50);
            setSlidePosition((int) total, 3000 * liftSpeedMultiplier);
//            prevTime = time;
            RobotLog.dd("LiftyUppy", "Target Position %f, time %f", SlidePosition, time);
        }
        prevTime = time;
        telemetry.addData("target position", SlidePosition);
        telemetry.addData("liftyPower", VerticalSlide1.getPower());

        telemetry.addData("Current VS1", VerticalSlide1.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current VS2", VerticalSlide2.getCurrent(CurrentUnit.MILLIAMPS));
    }

    boolean downTo1 = false;
    boolean at1 = false;
    double at1StartTime = 0;
    int preset1 = ConfigurationFirstRobot.highBar;
    int preset2 = ConfigurationFirstRobot.bottom;
    //todo is this right?
    int preset3 = ConfigurationFirstRobot.lowBar;
    int preset4 = ConfigurationFirstRobot.topBasket;
    boolean wasSet = false;

    public void goToPreset(boolean bottomRung, boolean bottom, boolean topRung, boolean topBasket) {
        if (bottomRung || bottom || topRung || topBasket){
            if (VerticalSlide1.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                VerticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                VerticalSlide1.setPower(1.0);
            }
            if (!wasSet) {
                if (bottomRung) {
                    setSlidePosition(preset1, 1800);
                }
                if (bottom) {
                    setSlidePosition(preset2, 2500);
                    downTo1 = true;
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
        //If the slide is at the button, turn it off
        if(downTo1 && !VerticalSlide1.isMotorEnabled()) {
            downTo1 = false;
            at1 = true;
            at1StartTime = System.currentTimeMillis();
        }
        if (at1 && VerticalSlide1.isMotorEnabled()) {
            at1 = false;
        }
        if (System.currentTimeMillis() > at1StartTime + 200 && at1) {
            at1 = false;
            VerticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VerticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        telemetry.addData("vert position", VerticalSlide1.getCurrentPosition());
    }

    double SlidePosition = 0;

    public void setSlidePosition(int position, double velocity) {
        VerticalSlide1.setTargetPosition(position);
        VerticalSlide1.setVelocity(velocity);
        VerticalSlide2.setTargetPosition(-position);
        VerticalSlide2.setVelocity(-velocity);
        SlidePosition = position;
//        runVerticalSlide2.start();
    }
    public void maintainVerticalSlide2() {
        double slideAmps = VerticalSlide1.getCurrent(CurrentUnit.MILLIAMPS);
        double slidePower = slideAmps / 9200;
//        VerticalSlide2.setPower(slidePower);
    }

    Thread runVerticalSlide2 = new Thread(new Runnable() {
        @Override
        public void run() {
            double slideAmps = VerticalSlide1.getCurrent(CurrentUnit.MILLIAMPS);
            double slidePower = slideAmps / 9200;
            VerticalSlide2.setPower(slidePower);
            while (VerticalSlide1.isMotorEnabled()) {
                slideAmps = VerticalSlide1.getCurrent(CurrentUnit.MILLIAMPS);
                slidePower = slideAmps / 9200;
                VerticalSlide2.setPower(slidePower);
                telemetry.addData("Current VS1", slideAmps);
                telemetry.addData("Current VS2", VerticalSlide2.getCurrent(CurrentUnit.MILLIAMPS));
            }
        }
    });
    public void setOnRung(boolean setRung) {
        if (setRung) {
            setSlidePosition((int) (SlidePosition + 1), 1800);
        }
    }
    public double getSlidePosition() {
        return SlidePosition;
    }
}
