package org.firstinspires.ftc.teamcode.secondrobot.horizontalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Pose;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalIntakeActions {
    public ServoImplEx intakeServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public HorizontalIntakeActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        intakeServo = hardwareMap.get(ServoImplEx.class, ConfigConstants.HORIZONTAL_INTAKE);

        intakeServo.setPosition(0);
    }

    boolean isVertGrabberClosed = false;
    boolean wasVertGrabberClosed = false;
    public void setIsVertGrabberClosed(boolean vertGrabberClosed) {
        isVertGrabberClosed = vertGrabberClosed;
    }
    double openStartTime = 0;
    public void close() {
        closed = true;
    }
    public void open() {
        closed = false;
    }
    boolean vertGrabberClosing = false;
    double vertGrabberClosingStartTime = 0;
    public void update() {
        if (!closed) {
            setPosition(Pose.horizontalGrabberWide);
            telemetry.addData("open", true);
        } else {
            setPosition(Pose.horizontalGrabberClose);
            telemetry.addData("closed", true);
        }
        //if the vertical grabber is closing, start a timer for .4 seconds. at the end of the .4 seconds, open the intake
        if (isVertGrabberClosed && !wasVertGrabberClosed){
            vertGrabberClosing = true;
            vertGrabberClosingStartTime = System.currentTimeMillis();
        }
        if (vertGrabberClosing && System.currentTimeMillis() > vertGrabberClosingStartTime + 100 && isVertGrabberClosed) {
            vertGrabberClosing = false;
            closed = false;
        }
        wasVertGrabberClosed = isVertGrabberClosed;
    }
    public void setPosition(double position) {
        intakeServo.setPosition(position);
    }
    boolean wasInput = false;
    public boolean closed = false;
    public void teleop(boolean input) {
        if (input && !wasInput) {
            if (closed) {
                open();
            } else {
                close();
            }
            update();
        }
        wasInput = input;
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
        intakeServo.setPosition(position);
    }
}
