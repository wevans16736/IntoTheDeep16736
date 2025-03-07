package org.firstinspires.ftc.teamcode.Configuration.firstRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

@Deprecated
public class VerticalSlideRR {
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap) {
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class VerticalSlidePosition implements Action {
            int position = 0;
            private boolean initialized = false;
            double currentPosition = verticalSlide1.getCurrentPosition();

            public VerticalSlidePosition(int position) {
                this.position = Math.abs(position);
            }

            public VerticalSlidePosition() {
                this.position = Math.abs(ConfigurationFirstRobot.bottom);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(position);
                    verticalSlide2.setVelocity(velocity);
                    initialized = true;
                }

                currentPosition = verticalSlide1.getCurrentPosition();
                if(Math.abs(currentPosition) > position){
                    initialized = false;
                    return false;
                }else{
                    return true;
                }
            }
        }

        public Action verticalSlidePosition() {
            return new VerticalSlidePosition();
        }

        public Action verticalSlidePosition(int position) {
            return new VerticalSlidePosition(position);
        }
    }