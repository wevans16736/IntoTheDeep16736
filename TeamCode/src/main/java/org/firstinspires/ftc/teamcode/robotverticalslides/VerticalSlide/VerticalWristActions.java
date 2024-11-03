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
    double forwardUp = 0.55;
    double forwardDown = 0.3;
    double backwardPos = 0.9;
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
}
