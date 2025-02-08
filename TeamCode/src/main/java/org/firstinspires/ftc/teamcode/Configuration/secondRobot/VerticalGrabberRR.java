package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalGrabberRR {
    Servo verticalGrabberServo;
    public VerticalGrabberRR(HardwareMap hardwareMap){
        verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
        verticalGrabberServo.setPosition(ConfigurationSecondRobot.verticalClose);
    }

    public class VerticalGrabberAction implements Action{
        double position;
        public VerticalGrabberAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            verticalGrabberServo.setPosition(position);
            return false;
        }
    }
    public Action verticalGrabberAction(double position){return new VerticalGrabberAction(position);}
    public void setPose(double pose){
        verticalGrabberServo.setPosition(pose);
    }
}
