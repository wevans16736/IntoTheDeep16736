package org.firstinspires.ftc.teamcode.Configuration.firstRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

@Deprecated
public class HorizontalRollRR {
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalRollRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(ConfigurationFirstRobot.flat);
        }

        public class HorizontalRollPosition implements Action {
            double position = ConfigurationFirstRobot.flat;

            public HorizontalRollPosition(double position) {
                this.position = position;
            }

            public HorizontalRollPosition() {
                this.position = ConfigurationFirstRobot.flat;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rollServo.setPosition(position);
                return false;
            }
        }

        public Action horizontalRollPosition(double position) {
            return new HorizontalRollPosition(position);
        }

        public Action horizontalRollPosition() {
            return new HorizontalRollPosition();
        }
    }