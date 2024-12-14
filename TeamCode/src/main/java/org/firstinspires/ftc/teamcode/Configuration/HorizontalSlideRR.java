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

public class HorizontalSlideRR {
        public DcMotorEx horizontalSlide2 = null;
        private Telemetry telemetry;

        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            horizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            horizontalSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
            horizontalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalSlide2.setTargetPosition(0);
            horizontalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private boolean initialized = false;

        public class HorizontalSLidePosition implements Action {
            int position = Configuration.retractSlide;
            private boolean initialized = false;

            public HorizontalSLidePosition(int position) {
                this.position = position;
            }

            public HorizontalSLidePosition() {
                this.position = Configuration.retractSlide;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 3600;
                if (!initialized) {
                    horizontalSlide2.setTargetPosition(position);
                    horizontalSlide2.setVelocity(velocity);
                    horizontalSlide2.setTargetPosition(-position);
                    horizontalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = horizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }

        public Action horizontalSlidePosition(int position) {
            return new HorizontalSLidePosition(position);
        }

        public Action horizontalSlidePosition() {
            return new HorizontalSLidePosition();
        }
    }