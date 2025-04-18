package org.firstinspires.ftc.teamcode.State.verticalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.State.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalGrabberActions {
    public ServoImplEx verticalGrabberServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public VerticalGrabberActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        verticalGrabberServo = hardwareMap.get(ServoImplEx.class, ConfigConstants.VERTICAL_GRABBER);

        verticalGrabberServo.setPosition(ConfigurationSecondRobot.verticalClose);
    }
    public void open() {
        verticalGrabberServo.setPosition(ConfigurationSecondRobot.verticalOpen);
        isClose = false;
    }
    public void close() {
        //turn on the servo to close
//        if (!verticalGrabberServo.isPwmEnabled()) {
//            verticalGrabberServo.setPwmDisable();
//        }
        verticalGrabberServo.setPosition(ConfigurationSecondRobot.verticalCloseHard);
        isClose = true;
    }
    boolean isClose = true;
    public boolean isClose() {
        return isClose;
    }
    double openStartTime = 0;
    boolean wasClose = false;
    public void teleOp(boolean close) {
        if (close &! wasClose) {
            if (isClose) {
                open();
                openStartTime = System.currentTimeMillis();
            } else {
                close();
            }
        }
        //If it's been open for a while, turn off the servo
//        if (!isClose && System.currentTimeMillis() > openStartTime + 420) {
//            if (verticalGrabberServo.isPwmEnabled()) {
//                verticalGrabberServo.setPwmDisable();
//            }
//        }
        wasClose = close;
    }
}
