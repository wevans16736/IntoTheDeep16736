package org.firstinspires.ftc.teamcode.Configuration.firstRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

@Deprecated
public class HorizontalGrabberRR {
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(ConfigurationFirstRobot.horizontalGrabberClose);
        }

        public class HorizontalIntakePosition implements Action {
            double position = ConfigurationFirstRobot.horizontalGrabberClose;

            public HorizontalIntakePosition(double position) {
                this.position = position;
            }

            public HorizontalIntakePosition() {
                this.position = ConfigurationFirstRobot.horizontalGrabberClose;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo.setPosition(position);
                return false;
            }
        }

        public Action horizontalIntakePosition(double position) {
            return new HorizontalIntakePosition(position);
        }

        public Action horizontalIntakePosition() {
            return new HorizontalIntakePosition();
        }
    }