package org.firstinspires.ftc.teamcode.secondrobot.horizontalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalWristActions {
    public ServoImplEx horizontalWristServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    private double backwardPosIn = ConfigurationSecondRobot.horizontalWristTransfer;
    private double backwardPosOut = ConfigurationSecondRobot.horizontalWristHover;
    private double forwardPosOut = ConfigurationSecondRobot.horizontalWristIntake;
    public HorizontalWristActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        horizontalWristServo = hardwareMap.get(ServoImplEx.class, ConfigConstants.HORIZONTAL_WRIST);
        horizontalWristServo.setPosition(backwardPosIn);
    }
    public boolean override = true;
    boolean wasOverride = false;
    public void override(boolean input) {
        if (input &! wasOverride) {
            override = !override;
            forward = true;
        }
        wasOverride = input;
    }
    public void setOverride(boolean input){
        override = input;
    }

    //if the override is on or the slide is in, set the servo to go up.
    //otherwise, the servo goes down and is either forward or backward depending on how the driver selects.
    public void update() {
        if (override) {
            horizontalWristServo.setPosition(backwardPosIn);
        } else if (forward) {
            horizontalWristServo.setPosition(forwardPosOut);
        } else {
            horizontalWristServo.setPosition(backwardPosOut);
        }
        telemetry.addData("override", override);
        telemetry.addData("forward", forward);
    }
    boolean wasSlideIn = false;
    public void setIsSlideIn(boolean isIt) {
//        if (isIt && !wasSlideIn) {
//            override = true;
//        }
//        wasSlideIn = isIt;
//        telemetry.addData("wasslidein", wasSlideIn);
    }
    boolean wasInput = false;
    public boolean forward = false;
    public void flipping(boolean input) {
        //if the driver presses the button, flip the servo between forwards and backwards
        if (input && !wasInput) {
            forward =! forward;
            if (override) {
                setOverride(false);
                forward = false;
            }
        }
        wasInput = input;
        update();
    }

    public void setForward(boolean forward) {
        this.forward = forward;
    }
    double position = 0.9;
    public void manual(boolean activate, boolean reverse) {
        if (activate) {
            if (!reverse) {
                position = position - 0.005;
            } else {
                position = position + 0.005;
            }
        }
        horizontalWristServo.setPosition(position);
    }
}
