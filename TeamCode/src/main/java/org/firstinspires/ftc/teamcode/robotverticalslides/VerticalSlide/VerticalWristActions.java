package org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class VerticalWristActions {
    public Servo verticalWristServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public VerticalWristActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);

        verticalWristServo.setPosition(backwardPos);
    }
    public void forward() {
//        verticalWristServo.setPosition(0.4);
        forward = true;
    }
    public void backward() {
//        verticalWristServo.setPosition(0.9);
        forward = false;
    }
    boolean isGrabberClosed = true;
    public void setGrabberClosed(boolean isIt) {
        isGrabberClosed = isIt;
    }
    boolean isSlideUp = false;
    public void setSlideUp(boolean isIt) {
        isSlideUp = isIt;
    }
    //this is a position to place it on the basket
    double forwardUp = 0.5;
    //this is a position to grab the butter from the wall
    double forwardDown = 0.30;
    //this is the position to grab the butter from the intake
    double backwardPos = 0.84;
    //If wrist is at highest point, set the wrist a bit higher to reach a higher point
    public void update() {
        if (forward) {
            if (isSlideUp) {
                verticalWristServo.setPosition(forwardUp);
            } else {
                verticalWristServo.setPosition(forwardDown);
            }
        } else {
            verticalWristServo.setPosition(backwardPos);
        }
    }
    boolean wasInput = false;
    boolean forward = false;
    //If the grabber is open, do nothing
    //If the grabber is closed and the button is pushed, it flips between forward and back
    public void flipping(boolean input) {
        if (isGrabberClosed) {
            if (input && !wasInput) {
                if (forward) {
                    backward();
                } else {
                    forward();
                }
            }
        }
        wasInput = input;
        update();
    }
    double position = 0.89;
    //Manually control the wrist. move with one button, reverse movement with other button
    public void manual(boolean activate, boolean reverse) {
        if (activate) {
            if (!reverse) {
                position = position - 0.005;
            } else {
                position = position + 0.005;
            }
        }
        verticalWristServo.setPosition(position);
    }
    public void autoFlipBack() {
        verticalWristServo.setPosition(backwardPos);
    }
    public void autoFlipForwardDown() {
        verticalWristServo.setPosition(forwardDown);
    }
    public void autoFlipForwardUp() {
        verticalWristServo.setPosition(forwardUp);
    }
}
