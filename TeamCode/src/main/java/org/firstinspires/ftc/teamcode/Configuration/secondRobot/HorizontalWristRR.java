package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalWristRR {
    Servo HorizontalWristServo;
    public HorizontalWristRR(HardwareMap hardwareMap){
        HorizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
        HorizontalWristServo.setPosition(0);
    }

    public class HorizontalWristAction implements Action{
        double position;
        public HorizontalWristAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            HorizontalWristServo.setPosition(position);
            return false;
        }
    }
    public Action horizontalWristAction(double position){return new HorizontalWristAction(position);}
}