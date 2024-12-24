package org.firstinspires.ftc.teamcode.Configuration;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class VerticalGrabberRR {
        public Servo verticalGrabberServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public VerticalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }

        public class VerticalGrabberPosition implements Action {
            double position = ConfigurationFirstRobot.verticalClose;

            public VerticalGrabberPosition(double position) {
                this.position = position;
            }

            public VerticalGrabberPosition() {
                this.position = ConfigurationFirstRobot.verticalClose;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalGrabberServo.setPosition(position);
                return false;
            }
        }

        public Action verticalGrabberPosition(double position) {
            return new VerticalGrabberPosition(position);
        }

        public Action verticalGrabberPosition() {
            return new VerticalGrabberPosition();
        }
    }