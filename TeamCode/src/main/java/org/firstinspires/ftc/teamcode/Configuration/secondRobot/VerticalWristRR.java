package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomus.firstRobot.YellowAuto;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalWristRR {
    Servo VerticalWristServo;
    public VerticalWristRR(HardwareMap hardwareMap){
        VerticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
//        VerticalWristServo.setPosition(ConfigurationSecondRobot.verticalWristIntake);
        VerticalWristServo.setPosition(.8);
    }
    public VerticalWristRR(HardwareMap hardwareMap, boolean teleop){
        if(teleop){
            VerticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            VerticalWristServo. setPosition(Pose.verticalWristTransfer);
        }
    }

    public class VerticalWristAction implements Action {
        double position;
        public VerticalWristAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            VerticalWristServo.setPosition(position);
            return false;
        }
    }
    public Action verticalWristAction(double position){return new VerticalWristAction(position);}

    public void setPose(double pose){
        VerticalWristServo.setPosition(pose);
    }
}
