package org.firstinspires.ftc.teamcode.Configuration;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;
public class HorizontalWristRR {
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(Configuration.horizontalWristTransfer);
        }

        public class HorizontalWristPosition implements Action {
            double position = Configuration.horizontalWristTransfer;

            public HorizontalWristPosition(double position) {
                this.position = position;
            }

            public HorizontalWristPosition() {
                this.position = Configuration.horizontalWristTransfer;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                horizontalWristServo.setPosition(position);
                return false;
            }
        }

        public Action horizontalWristPosition(double position) {
            return new HorizontalWristPosition(position);
        }

        public Action horizontalWristPosition() {
            return new HorizontalWristPosition();
        }
    }