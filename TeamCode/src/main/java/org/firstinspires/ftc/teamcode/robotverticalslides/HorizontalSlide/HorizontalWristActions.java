package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalWristActions {
    public Servo horizontalWristServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public HorizontalWristActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);

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
    boolean override = false;
    boolean wasOverride = false;
    public void override(boolean input) {
        if (input &! wasOverride) {
            override = !override;
        }
        wasOverride = input;
    }
    private double forwardPosOut = 0.2;
    private double backwardPosOut = 0.45;
    private double backwardPosIn = 0.85;
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
