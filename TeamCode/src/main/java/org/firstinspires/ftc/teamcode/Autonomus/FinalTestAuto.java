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
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Config
@Autonomous(name = "Final Test Auto", group = "Autonomous")
public class FinalTestAuto extends LinearOpMode {
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

                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),

                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));
//
//        boolean pickRight = false;
//        boolean pickLeft = false;
//        boolean pickHang = false;
//        boolean pickButter = false;
//
//        telemetry.clearAll();
//        while(!gamepad1.cross) {
//            telemetry.addLine("Which Side?" + pickRight);
//            if(gamepad1.square && !gamepad1.circle){
//                pickRight = false;
//                pickLeft = true;
//            }
//            if(!gamepad1.square && gamepad1.circle){
//                pickRight = true;
//                pickLeft = false;
//            }
//            telemetry.update();
//        }
//        if(pickRight){
//            telemetry.clearAll();
//            while(!gamepad1.right_bumper){
//                telemetry.addLine("square-hang: "+ pickHang);
//                telemetry.addLine("triangle-butter"+ pickButter);
//                if(gamepad1.square && !gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
//                    pickHang = true;
//                }
//                if(!gamepad1.square && gamepad1.triangle && !gamepad1.circle && !gamepad1.cross){
//                    pickButter = true;
//                }
//                telemetry.update();
//            }
//        }



        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        VelConstraint hangAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint hangAcceleration = new ProfileAccelConstraint(-30, 50);

        VelConstraint parkVelOverride = new TranslationalVelConstraint(100);
        VelConstraint parkAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100),
                new AngularVelConstraint(Math.toRadians(80))
        ));
        AccelConstraint parkAccelOverride = new ProfileAccelConstraint(-100, 100);

        VelConstraint humanAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.toRadians(180))
        ));
        AccelConstraint humanAccelOverride = new ProfileAccelConstraint(-15, 50);

        TurnConstraints turnConstraints = new TurnConstraints(Math.toRadians(180), -Math.toRadians(450), Math.toRadians(360));


        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);

        TrajectoryActionBuilder hang = drive.actionBuilder(currentPose)
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-16, 30), hangAngularOverride, hangAcceleration)
                .waitSeconds(.125)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(.5, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0))
                .strafeTo(new Vector2d(-10, 25),parkVelOverride, parkAccelOverride);


//        TrajectoryActionBuilder butterRight = drive.actionBuilder(currentPose)
////                //first butter
////                .splineToLinearHeading(new Pose2d(20,13, Math.toRadians(0)), Math.toRadians(0), parkAngularOverride, parkAccelOverride)
////                .afterTime(0,horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
////                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
////                .splineToLinearHeading(new Pose2d(34.25, 29.9, Math.toRadians(-90)), Math.toRadians(90), hangAngularOverride, parkAccelOverride)
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
////                .waitSeconds(.5)
////                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosOut))
////                .splineToLinearHeading(new Pose2d(35, 19, Math.toRadians(90)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
////                .waitSeconds(.5)
////
////                //second butter
////                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
////                .turn(Math.toRadians(180), turnConstraints)
////                .strafeToLinearHeading(new Vector2d(47, 30.5), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
////                .waitSeconds(.5)
////                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosOut))
////                .splineToLinearHeading(new Pose2d(36, 19, Math.toRadians(90)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
////                .waitSeconds(.5)
////                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
////                .afterTime(.25, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
////                .afterTime(.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
////                .afterTime(.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
////                .afterTime(.25, horizontalRollRR.horizontalRollPosition(Configuration.flat));

        TrajectoryActionBuilder postHang = drive.actionBuilder(currentPose)
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .strafeTo(new Vector2d(39, 20))
                .turn(Math.toRadians(180), turnConstraints)
                .strafeTo(new Vector2d(39, 19), humanAngularOverride, humanAccelOverride)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                .waitSeconds(.45)
                .turn(Math.toRadians(180), turnConstraints)
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(39, 12), parkAngularOverride, parkAccelOverride)
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .splineToLinearHeading(new Pose2d(-12, 29, Math.toRadians(90)), Math.toRadians(90), hangAngularOverride, hangAcceleration)
                .waitSeconds(.125)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0))
                .strafeTo(new Vector2d(-10, 24), parkVelOverride, parkAccelOverride)
                //second hang
                .turn(Math.toRadians(180), turnConstraints)
                .strafeToLinearHeading(new Vector2d(39, 27), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
                .strafeTo(new Vector2d(39, 18), parkVelOverride, parkAccelOverride)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
                .waitSeconds(.45)
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(39, 23), parkAngularOverride, parkAccelOverride)
                .splineToLinearHeading(new Pose2d(-8, 29, Math.toRadians(90)), Math.toRadians(90), hangAngularOverride, hangAcceleration)
                .waitSeconds(.125)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0));

        TrajectoryActionBuilder chosenTrajectory;
        Actions.runBlocking(new SequentialAction(startPosition.build()));
        chosenTrajectory = startPosition;

        Action actionHang = chosenTrajectory.endTrajectory().fresh()
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.25)
                .strafeTo(new Vector2d(-16, 28), hangAngularOverride, hangAcceleration)
                .waitSeconds(.125)
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(.5, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .afterTime(.5, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0))
                .strafeTo(new Vector2d(-10, 25),parkVelOverride, parkAccelOverride)
                .build();
        Actions.runBlocking(new SequentialAction(actionHang));
        chosenTrajectory = hang;

        Action actionRightButter = chosenTrajectory.endTrajectory().fresh()
                //first butter
                .splineToLinearHeading(new Pose2d(20,13, Math.toRadians(0)), Math.toRadians(0), parkAngularOverride, parkAccelOverride)
                .afterTime(0,horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .splineToLinearHeading(new Pose2d(34.25, 29.9, Math.toRadians(-90)), Math.toRadians(90), hangAngularOverride, parkAccelOverride)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .waitSeconds(.5)
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosOut))
                .splineToLinearHeading(new Pose2d(35, 19, Math.toRadians(90)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .waitSeconds(.5)

                //second butter
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .turn(Math.toRadians(180), turnConstraints)
                .strafeToLinearHeading(new Vector2d(47, 30.5), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .waitSeconds(.5)
                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosOut))
                .splineToLinearHeading(new Pose2d(36, 19, Math.toRadians(90)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .waitSeconds(.5)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .afterTime(.25, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
                .afterTime(.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
                .afterTime(.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
                .afterTime(.25, horizontalRollRR.horizontalRollPosition(Configuration.flat))

                //third butter
//                .afterTime(0, horizontalRollRR.horizontalRollPosition(Configuration.sideway))
//                .afterTime(.5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
//                .splineToLinearHeading(new Pose2d(43, 40.5, Math.toRadians(180)), Math.toRadians(0), parkAngularOverride, parkAccelOverride)
//                .splineToLinearHeading(new Pose2d(47.75, 40.5, Math.toRadians(180)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
//                .afterTime(.5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
//                .waitSeconds(1)
//                .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosOut))
//                .splineToLinearHeading(new Pose2d(39, 17, Math.toRadians(90)), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
//                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
//                .afterTime(.25, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose))
//                .afterTime(.25, horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide))
//                .afterTime(.25, horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn))
//                .afterTime(.25, horizontalRollRR.horizontalRollPosition(Configuration.flat))
                .build();

        Actions.runBlocking(new SequentialAction(actionRightButter));
//        chosenTrajectory = butterRight;

//        Action actionPostHang = chosenTrajectory.endTrajectory().fresh()
//                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
//                .strafeTo(new Vector2d(39, 20))
//                .turn(Math.toRadians(180), turnConstraints)
//                .strafeTo(new Vector2d(39, 19), humanAngularOverride, humanAccelOverride)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
//                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
//                .waitSeconds(.45)
//                .turn(Math.toRadians(180), turnConstraints)
//                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
//                .strafeTo(new Vector2d(39, 12), parkAngularOverride, parkAccelOverride)
//                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
//                .splineToLinearHeading(new Pose2d(-12, 29, Math.toRadians(90)), Math.toRadians(90), hangAngularOverride, hangAcceleration)
//                .waitSeconds(.125)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
//                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0))
//                .strafeTo(new Vector2d(-10, 24), parkVelOverride, parkAccelOverride)
//                //second hang
//                .turn(Math.toRadians(180), turnConstraints)
//                .strafeToLinearHeading(new Vector2d(39, 27), Math.toRadians(-90), parkAngularOverride, parkAccelOverride)
//                .strafeTo(new Vector2d(39, 18), parkVelOverride, parkAccelOverride)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
//                .afterTime(.4, verticalSlideRR.verticalSlidePosition(-100))
//                .waitSeconds(.45)
//                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
//                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
//                .strafeTo(new Vector2d(39, 23), parkAngularOverride, parkAccelOverride)
//                .splineToLinearHeading(new Pose2d(-8, 29, Math.toRadians(90)), Math.toRadians(90), hangAngularOverride, hangAcceleration)
//                .waitSeconds(.125)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
//                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
//                .afterTime(.25, verticalSlideRR.verticalSlidePosition(0))

//        Actions.runBlocking(new SequentialAction(actionPostHang));
        chosenTrajectory = postHang;

        Action actionParkRight = chosenTrajectory.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(40, 18, Math.toRadians(90)),Math.toRadians(90), parkAngularOverride, parkAccelOverride)
                .waitSeconds(1)
                .build();
//        Actions.runBlocking(new SequentialAction(actionParkRight));
    }
}
