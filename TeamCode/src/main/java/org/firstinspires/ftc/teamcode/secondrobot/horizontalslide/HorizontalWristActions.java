package org.firstinspires.ftc.teamcode.secondrobot.horizontalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.ConfigurationFirstRobot;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

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
    public void forward() {
//        horizontalWristServo.setPosition(0.2);
        forward = true;
    }
    public void backward() {
//        horizontalWristServo.setPosition(0.9);
        forward = false;
    }
    public boolean override = true;
    boolean wasOverride = false;
    public void override(boolean input) {
        if (input &! wasOverride) {
            override = !override;
            forward();
        }
        wasOverride = input;
    }

    public void update() {
        if (override) {
            horizontalWristServo.setPosition(backwardPosIn);
        } else if (forward) {
            if (!isSlideIn) {
                horizontalWristServo.setPosition(forwardPosOut);
            } else {
                horizontalWristServo.setPosition(backwardPosIn);
            }
        } else {
            if (isSlideIn) {
                horizontalWristServo.setPosition(backwardPosIn);
            } else {
                if (override) {
                    horizontalWristServo.setPosition(backwardPosIn);
                } else {
                    horizontalWristServo.setPosition(backwardPosOut);
                }
            }
        }
    }
    boolean isSlideIn = true;
    public void setIsSlideIn(boolean isIt) {
        isSlideIn = isIt;
    }
    boolean wasInput = false;
    public boolean forward = false;
    public void flipping(boolean input) {
        if (input && !wasInput) {
            if (!forward && !isSlideIn) {
                forward();
            } else if (forward){
                backward();
            }
        }
        wasInput = input;
        update();
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
