package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import java.util.Arrays;

@Config
@Autonomous(name = "Final Auto", group = "Autonomous")
public class FinalAuto extends LinearOpMode {
    public class VerticalSlideRR {
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap) {
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        private boolean initialized = false;
        public class VerticalSlidePosition implements Action {
            int position = 0;
            private boolean initialized = false;

            public VerticalSlidePosition(int position) {
                this.position = position;
            }

            public VerticalSlidePosition() {
                this.position = Configuration.bottom;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action verticalSlidePosition() {
            return new VerticalSlidePosition();}
        public Action verticalSlidePosition(int position) {
            return new VerticalSlidePosition(position);
        }
    }

    public class HorizontalSlideRR {
        public DcMotorEx horizontalSlide2 = null;
        private Telemetry telemetry;

        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            horizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            horizontalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
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
                double velocity = 1800;
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
            double position = Configuration.close;
            public VerticalGrabberPosition(double position){
                this.position = position;
            }
            public VerticalGrabberPosition(){
                this.position = Configuration.close;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalGrabberServo.setPosition(position);
                return false;
            }
        }
        public Action verticalGrabberPosition(double position) {return new VerticalGrabberPosition(position);}
        public Action verticalGrabberPosition() {return new VerticalGrabberPosition();}
    }
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public VerticalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(Configuration.backwardPos);
        }
        public class VerticalWristPosition implements Action{
            double position = Configuration.backwardPos;
            public VerticalWristPosition(double position){
                this.position = position;
            }
            public VerticalWristPosition(){
                this.position = Configuration.backwardPos;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(position);
                return false;
            }
        }
        public Action verticalWristPosition(double position) {return new VerticalWristPosition(position);}
        public Action verticalWristPosition() {return  new VerticalWristPosition();}
    }
    public class HorizontalGrabberRR{
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(Configuration.floorClose);
        }
        public class HorizontalIntakePosition implements Action{
            double position = Configuration.floorClose;
            public HorizontalIntakePosition(double position){
                this.position = position;
            }
            public HorizontalIntakePosition(){
                this.position = Configuration.floorClose;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalIntakePosition(double position) {return new HorizontalIntakePosition(position);}
        public Action horizontalIntakePosition() {return new HorizontalIntakePosition();}
    }
    public class HorizontalWristRR{
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(Configuration.backwardPosIn);
        }
        public class HorizontalWristPosition implements Action{
            double position = Configuration.backwardPosIn;
            public HorizontalWristPosition(double position){
                this.position = position;
            }
            public HorizontalWristPosition(){
                this.position = Configuration.backwardPosIn;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                horizontalWristServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalWristPosition(double position) {return new HorizontalWristPosition(position);}
        public Action horizontalWristPosition() {return new HorizontalWristPosition();}
    }
    public class HorizontalRollRR{
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalRollRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(Configuration.flat);
        }
        public class HorizontalRollPosition implements Action{
            double position = Configuration.flat;
            public HorizontalRollPosition(double position){
                this.position = position;
            }
            public HorizontalRollPosition(){
                this.position = Configuration.flat;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rollServo.setPosition(position);
                return false;
            }
        }
        public Action horizontalRollPosition(double position) {return new HorizontalRollPosition(position);}
        public Action horizontalRollPosition() {return new HorizontalRollPosition();}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);

        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
        HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);

        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);

        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);

        Pose2d currentPose = new Pose2d(0,0,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                horizontalWristRR.horizontalWristPosition(.7),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

        boolean pickRight = false;
        boolean pickLeft = false;
        boolean pickHang = false;
        boolean pickButter = false;
        boolean pickPostHang = false;

        int hangSide = 1;

        telemetry.clearAll();
        while(!gamepad1.cross) {
            telemetry.addLine("Which Side");
            telemetry.addLine("square-right: "+ pickRight);
            telemetry.addLine("circle-left: "+ pickLeft);
            telemetry.addLine("cross-confirm");
            if(gamepad1.square && !gamepad1.circle){
                pickRight = false;
                pickLeft = true;
            }
            if(!gamepad1.square && gamepad1.circle){
                pickRight = true;
                pickLeft = false;
            }
            telemetry.update();
        }
        if(pickRight){
            telemetry.clearAll();
            while(!gamepad1.right_bumper){
                telemetry.addLine("square-hang: "+ pickHang);
                telemetry.addLine("triangle-butter"+ pickButter);
                telemetry.addLine("circle-postHang"+ pickPostHang);
                telemetry.addLine("right bumper-confirm");
                if(gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
                    pickHang = !pickHang;
                }
                if(!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
                    pickButter = !pickButter;
                }
                if(!gamepad1.square && !gamepad1.triangle && gamepad1.circle && !gamepad1.cross){
                    pickPostHang = !pickPostHang;
                }
                telemetry.clearAll();
                telemetry.update();
            }
        }
        if(pickLeft){
            telemetry.clearAll();
            while(!gamepad1.right_bumper) {
                telemetry.addLine("square-hang" + pickHang);
                telemetry.addLine("triangle-butter" + pickButter);
                telemetry.addLine("right bumper-confirm");
                if (gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
                    pickHang = !pickHang;
                }
                if (!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross) {
                    pickButter = !pickButter;
                }
                telemetry.clearAll();
                telemetry.update();
            }
        }

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);

        TrajectoryActionBuilder hang = drive.actionBuilder(currentPose)
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .strafeTo(new Vector2d(-16, 28))//todo might add hang velocity and acceleration
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.open))
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .strafeTo(new Vector2d(-16, 24));

        TrajectoryActionBuilder butterRight = drive.actionBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(20,13, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0,horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .splineToLinearHeading(new Pose2d(34.25, 29.9, Math.toRadians(-90)), Math.toRadians(90))
                //got to the first butter, will execute a transfer system
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .afterTime(.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                .afterTime(.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                .strafeTo(new Vector2d(47, 29.9))
                //butter is in the robot held by horizontal grabber
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                //butter is now held by vertical grabber
                .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(.5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterTime(.5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                //first butter is on the other side of the robot while the horizontal slide extended
                .afterTime(1, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(1, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .waitSeconds(1)
                //held the second butter and drop the first butter
                .afterTime(1.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                .afterTime(1.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                //todo can I grab the third butter while having the vertical wrist in the human to drop the butter?
                .strafeToLinearHeading(new Vector2d(47, 29.9), Math.toRadians(30))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.open))

                //second butter in the transfer system
                .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(.5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterTime(.5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                //second butter is other side of the robot and the horizontal slide is extended
                .afterTime(1, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(1, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .waitSeconds(1)
                //second butter dropped and the third butter is picked up
                .strafeToLinearHeading(new Vector2d(39,20), Math.toRadians(90))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                //third butter is in the transfer system
                .afterTime(.5, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                //third butter is now held by vertical grabber
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                //third butter is now other side of the robot
                .afterTime(1.5, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                //third butter is now dropped
                .waitSeconds(1.5);

        TrajectoryActionBuilder postHang = drive.actionBuilder(currentPose)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                .strafeTo(new Vector2d(39, 19)) //todo might want to add human player acceleration
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                .waitSeconds(.4)
                //robot pick the butter from the wall
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .strafeToLinearHeading(new Vector2d(-12, 24),Math.toRadians(90))
                .strafeTo(new Vector2d(-12, 28)) //todo might add hang velocity and acceleration
                //hang the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .strafeTo(new Vector2d(-12, 24))
                //approach the human player area
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                .strafeTo(new Vector2d(39,19)) //todo might want to add human player acceleration
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                .waitSeconds(.4)
                //grab the second butter from the wall
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .strafeToLinearHeading(new Vector2d(-10, 24),Math.toRadians(90))
                .strafeTo(new Vector2d(-10, 28))//todo might add hang velocity and acceleration
                //hang the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .strafeTo(new Vector2d(-10, 24))
                //approach the human player area
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                .strafeTo(new Vector2d(39,19)) //todo might want to add human player acceleration
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                .waitSeconds(.4)
                //grab the third butter from the wall
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .strafeToLinearHeading(new Vector2d(-8, 24),Math.toRadians(90))
                .strafeTo(new Vector2d(-8, 28)) //todo might add hang velocity and acceleration
                //hang the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .strafeTo(new Vector2d(-8, 24))
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.bottom));



        TrajectoryActionBuilder chosenTrajectory;
        new SequentialAction(startPosition.build());
        chosenTrajectory = startPosition;

        if(pickRight){
            if(pickHang){
                Action actionHang = chosenTrajectory.endTrajectory().fresh()
                        .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                        .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        .strafeTo(new Vector2d(-16, 28))//todo might add hang velocity and acceleration
                        .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.open))
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .strafeTo(new Vector2d(-16, 24))
                        .build();
                Actions.runBlocking(new SequentialAction(actionHang));
                chosenTrajectory = hang;
            }
            if(pickButter){
                Action actionRightButter = chosenTrajectory.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(20,13, Math.toRadians(0)), Math.toRadians(0))
                        .afterTime(0,horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                        .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                        .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .splineToLinearHeading(new Pose2d(34.25, 29.9, Math.toRadians(-90)), Math.toRadians(90))
                        //got to the first butter, will execute a transfer system
                        .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                        .afterTime(.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                        .afterTime(.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                        .strafeTo(new Vector2d(47, 29.9))
                        //butter is in the robot held by horizontal grabber
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                        //butter is now held by vertical grabber
                        .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        .afterTime(.5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                        .afterTime(.5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                        //first butter is on the other side of the robot while the horizontal slide extended
                        .afterTime(1, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(1, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                        .waitSeconds(1)
                        //held the second butter and drop the first butter
                        .afterTime(1.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                        .afterTime(1.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                        //todo can I grab the third butter while having the vertical wrist in the human to drop the butter?
                        .strafeToLinearHeading(new Vector2d(47, 29.9), Math.toRadians(30))
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.open))

                        //second butter in the transfer system
                        .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        .afterTime(.5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                        .afterTime(.5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                        //second butter is other side of the robot and the horizontal slide is extended
                        .afterTime(1, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(1, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                        .waitSeconds(1)
                        //second butter dropped and the third butter is picked up
                        .strafeToLinearHeading(new Vector2d(39,20), Math.toRadians(90))
                        .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .afterTime(0, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                        .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                        //third butter is in the transfer system
                        .afterTime(.5, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(.5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                        //third butter is now held by vertical grabber
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        //third butter is now other side of the robot
                        .afterTime(1.5, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        //third butter is now dropped
                        .waitSeconds(15)
                        .build();

                Actions.runBlocking(new SequentialAction(actionRightButter));
                chosenTrajectory = butterRight;
            }
            if(pickPostHang){
                Action actionPostHang = chosenTrajectory.endTrajectory().fresh()
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                        .strafeTo(new Vector2d(39, 19)) //todo might want to add human player acceleration
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                        .waitSeconds(.4)
                        //robot pick the butter from the wall
                        .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                        .strafeToLinearHeading(new Vector2d(-12, 24),Math.toRadians(90))
                        .strafeTo(new Vector2d(-12, 28)) //todo might add hang velocity and acceleration
                        //hang the butter
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .strafeTo(new Vector2d(-12, 24))
                        //approach the human player area
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                        .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                        .strafeTo(new Vector2d(39,19)) //todo might want to add human player acceleration
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                        .waitSeconds(.4)
                        //grab the second butter from the wall
                        .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                        .strafeToLinearHeading(new Vector2d(-10, 24),Math.toRadians(90))
                        .strafeTo(new Vector2d(-10, 28))//todo might add hang velocity and acceleration
                        //hang the butter
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .strafeTo(new Vector2d(-10, 24))
                        //approach the human player area
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                        .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                        .strafeToLinearHeading(new Vector2d(39, 24), Math.toRadians(-90))
                        .strafeTo(new Vector2d(39,19)) //todo might want to add human player acceleration
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                        .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                        .waitSeconds(.4)
                        //grab the third butter from the wall
                        .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                        .strafeToLinearHeading(new Vector2d(-8, 24),Math.toRadians(90))
                        .strafeTo(new Vector2d(-8, 28)) //todo might add hang velocity and acceleration
                        //hang the butter
                        .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                        .afterTime(1, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                        .strafeTo(new Vector2d(-8, 24))
                        .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                        .build();
                Actions.runBlocking(new SequentialAction(actionPostHang));
                chosenTrajectory = postHang;
            }
            Action actionParkRight = chosenTrajectory.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(40, 15), Math.toRadians(0))
                    .waitSeconds(1)
                    .build();
            Actions.runBlocking(new SequentialAction(actionParkRight));
        }
        if(pickLeft){

        }
    }
}
