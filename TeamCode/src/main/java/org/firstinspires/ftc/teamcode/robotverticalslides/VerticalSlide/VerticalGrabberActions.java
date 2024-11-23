package org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomus.Configuration;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class VerticalGrabberActions {
    public Servo verticalGrabberServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public VerticalGrabberActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);

        verticalGrabberServo.setPosition(0.3);
    }
    public void open() {
        verticalGrabberServo.setPosition(Configuration.open);
    }
    public void close() {
        verticalGrabberServo.setPosition(0.5);
    }
    boolean isClose = true;
    public boolean isClose() {
        return isClose;
    }
    public void teleOp(boolean open, boolean close) {
        if (open) {
            open();
            isClose = false;
        }
        if (close) {
            close();
            isClose = true;
        }
    }
}
