package org.firstinspires.ftc.teamcode.Configuration;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;
public class HorizontalWristRR {
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(Configuration.backwardPosIn);
        }

        public class HorizontalWristPosition implements Action {
            double position = Configuration.backwardPosIn;

            public HorizontalWristPosition(double position) {
                this.position = position;
            }

            public HorizontalWristPosition() {
                this.position = Configuration.backwardPosIn;
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