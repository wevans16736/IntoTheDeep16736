package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.firstRobot.ConfigurationFirstRobot;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalWristActions {
    public ServoImplEx horizontalWristServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    private double backwardPosIn = ConfigurationFirstRobot.horizontalWristTransfer;
    private double backwardPosOut = ConfigurationFirstRobot.horizontalWristHover;
    private double forwardPosOut = ConfigurationFirstRobot.horizontalWristIntake;
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

    //if the override is on or the slide is in, set the servo to go up.
    //otherwise, the servo goes down and is either forward or backward depending on how the driver selects.
    public void update() {
        if (override || isSlideIn) {
            horizontalWristServo.setPosition(backwardPosIn);
        } else if (forward) {
            horizontalWristServo.setPosition(forwardPosOut);
        } else {
            horizontalWristServo.setPosition(backwardPosOut);
        }
    }
    boolean isSlideIn = true;
    public void setIsSlideIn(boolean isIt) {
        isSlideIn = isIt;
    }
    boolean wasInput = false;
    public boolean forward = false;
    public void flipping(boolean input) {
        //if the driver presses the button, flip the servo between forwards and backwards
        if (input && !wasInput) {
            if (!forward && !isSlideIn) {
                forward = true;
            } else if (forward){
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
