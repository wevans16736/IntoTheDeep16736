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
public class HorizontalRollRR {
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalRollRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(Configuration.flat);
        }

        public class HorizontalRollPosition implements Action {
            double position = Configuration.flat;

            public HorizontalRollPosition(double position) {
                this.position = position;
            }

            public HorizontalRollPosition() {
                this.position = Configuration.flat;
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