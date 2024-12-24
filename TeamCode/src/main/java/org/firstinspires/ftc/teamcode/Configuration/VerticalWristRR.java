package org.firstinspires.ftc.teamcode.Configuration;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class VerticalWristRR {
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public VerticalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(ConfigurationFirstRobot.verticalWristIntake);
        }

        public class VerticalWristPosition implements Action {
            double position = ConfigurationFirstRobot.verticalWristIntake;

            public VerticalWristPosition(double position) {
                this.position = position;
            }

            public VerticalWristPosition() {
                this.position = ConfigurationFirstRobot.verticalWristIntake;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalWristServo.setPosition(position);
                return false;
            }
        }

        public Action verticalWristPosition(double position) {
            return new VerticalWristPosition(position);
        }

        public Action verticalWristPosition() {
            return new VerticalWristPosition();
        }
    }