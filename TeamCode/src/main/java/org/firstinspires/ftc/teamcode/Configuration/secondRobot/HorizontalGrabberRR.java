package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalGrabberRR {
    Servo HorizontalGrabberServo;
    public HorizontalGrabberRR(HardwareMap hardwareMap){
        HorizontalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
        HorizontalGrabberServo.setDirection(Servo.Direction.REVERSE);
        HorizontalGrabberServo.setPosition(ConfigurationSecondRobot.horizontalGrabberOpen);
    }

    public class HorizontalGrabberAction implements Action {
        double position;
        public HorizontalGrabberAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            HorizontalGrabberServo.setPosition(position);
            return false;
        }
    }
    public Action horizontalGrabberAction(double position) {return new HorizontalGrabberAction(position);}

    public void setPose(double pose){
        HorizontalGrabberServo.setPosition(pose);
    }
}
