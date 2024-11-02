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

        horizontalWristServo.setPosition(0.65);
    }
    public void forward() {
        horizontalWristServo.setPosition(0.05);
        forward = true;
    }
    public void backward() {
        horizontalWristServo.setPosition(0.65);
        forward = false;
    }
    boolean isSlideIn = true;
    public void setIsSlideIn(boolean isIt) {
        isSlideIn = isIt;
    }
    boolean wasInput = false;
    boolean forward = false;
    public void flipping(boolean input) {
        if (input && !wasInput) {
            if (!forward && !isSlideIn) {
                forward();
            } else if (forward){
                backward();
            }
        }
        wasInput = input;
    }
}
