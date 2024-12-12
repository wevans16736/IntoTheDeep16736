package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.Mutex;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import java.util.Arrays;

@Config
@Autonomous(name = "1. Final Auto", group = "Autonomous")
public class FinalAuto extends LinearOpMode {
    VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
    HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);

    VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
    HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);

    VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
    HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);

    HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);

    RobotSpecial robotSpecial = new RobotSpecial();
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
            return new VerticalSlidePosition();
        }

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

            public VerticalGrabberPosition(double position) {
                this.position = position;
            }

            public VerticalGrabberPosition() {
                this.position = Configuration.close;
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

    public class VerticalWristRR {
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public VerticalWristRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(Configuration.backwardPos);
        }

        public class VerticalWristPosition implements Action {
            double position = Configuration.backwardPos;

            public VerticalWristPosition(double position) {
                this.position = position;
            }

            public VerticalWristPosition() {
                this.position = Configuration.backwardPos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalWristServo.setPosition(position);
                return false;
            }
        }

        public Action verticalWristPosition(double position) {
            return new VerticalWristPosition(position);
        }

        public Action verticalWristPosition() {
            return new VerticalWristPosition();
        }
    }

    public class HorizontalGrabberRR {
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;

        public HorizontalGrabberRR(HardwareMap opModeHardware, Telemetry opModeTelemetry) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(Configuration.floorClose);
        }

        public class HorizontalIntakePosition implements Action {
            double position = Configuration.floorClose;

            public HorizontalIntakePosition(double position) {
                this.position = position;
            }

            public HorizontalIntakePosition() {
                this.position = Configuration.floorClose;
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
    public class RobotSpecial {
        public class TransferSystem implements Action{
            boolean primeHorizontal = false;
            boolean pickLeft = false;
            public TransferSystem(){
                primeHorizontal = false;
            }
            public TransferSystem(boolean pickLeft, boolean requestHorizontal){
                primeHorizontal = requestHorizontal;
                this.pickLeft = pickLeft;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(pickLeft) {
                    Actions.runBlocking(new SequentialAction(
                            //let go of the butter on top
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.5),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                            new SleepAction(2),
                            //extend the horizontal Slide
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.extend),
                            horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut),
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.5),
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            verticalSlideRR.verticalSlidePosition(Configuration.topBasket),
                            verticalWristRR.verticalWristPosition(Configuration.forwardUp)
                    ));
                }
                if(!pickLeft) {
                if (primeHorizontal) {
                    Actions.runBlocking(new SequentialAction(
                            //let go of the butter if it is up ontop of the basket
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            //butter go to the other side while priming the horizontal slide for other butter
                            verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                            horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut),
                            horizontalSlideRR.horizontalSlidePosition(Configuration.extend),
                            new SleepAction(1.5),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.5),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                    ));
                }
                if (!primeHorizontal) {
                    Actions.runBlocking(new SequentialAction(
                            //grab the butter from the floor
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(.5),
                            //retract the slide
                            horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                            horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                            new SleepAction(1),
                            //vertical grabber close
                            verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                            new SleepAction(.25),
                            //horizontal grabber open
                            horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                            new SleepAction(.25),
                            verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                            new SleepAction(1),
                            verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                            new SleepAction(.25),
                            verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                    ));
                }
            }
                return false;
        }
    }
    public Action transferSystem(boolean pickLeft, boolean primeHorizontal) {return new TransferSystem(pickLeft, primeHorizontal);}
    public Action transferSystem() {return new TransferSystem();}
}

@Override
    public void runOpMode() throws InterruptedException  {
        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),

                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),

                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);

        TrajectoryActionBuilder hang = drive.actionBuilder(currentPose)
            .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.4)
                .strafeTo(new Vector2d(-16, 28))
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterTime(.25, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterTime(.25, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 24));

        TrajectoryActionBuilder ButterRight = drive.actionBuilder(currentPose)
                .afterDisp(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterDisp(5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .afterDisp(5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .strafeToSplineHeading(new Vector2d(36, 27.45), Math.toRadians(-90))
                .afterTime(0, robotSpecial.transferSystem())
                .waitSeconds(.5)
                .strafeTo(new Vector2d(48.25, 27.75))
                .afterTime(0, robotSpecial.transferSystem())
                .waitSeconds(2);

        TrajectoryActionBuilder PostHang = drive.actionBuilder(currentPose)
                //approach the human player
                .afterDisp(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .strafeToSplineHeading(new Vector2d(45, 20), Math.toRadians(-90))
                //grab the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .waitSeconds(.25)
                //lift the slide partly and approach the hang
                .afterTime(0, verticalSlideRR.verticalSlidePosition(-100))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(-16, 24))
                //hang
                .afterDisp(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 28))
                //approach the human player
                .strafeToSplineHeading(new Vector2d(45, 20), Math.toRadians(-90))
                //grab the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .waitSeconds(.25)
                //lift the slide partly and approach the hang
                .afterTime(0, verticalSlideRR.verticalSlidePosition(-100))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(-16, 24))
                //hang
                .afterDisp(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom));

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder chosenTrajectory;
        Actions.runBlocking(new SequentialAction(startPosition.build()));
        chosenTrajectory = startPosition;

        Action ActionHang = chosenTrajectory.endTrajectory().fresh()
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .waitSeconds(.4)
                .strafeTo(new Vector2d(-16, 28))
                .afterDisp(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(2, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 24))
                .build();

        Actions.runBlocking(new SequentialAction(ActionHang));
        chosenTrajectory = hang;

        Action ActionButterRight = chosenTrajectory.endTrajectory().fresh()
                .afterDisp(0, verticalGrabberRR.verticalGrabberPosition(Configuration.open))
                .afterDisp(0, verticalWristRR.verticalWristPosition(Configuration.backwardPos))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.extend))
                .afterDisp(5, horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut))
                .afterDisp(5, horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen))
                .strafeToSplineHeading(new Vector2d(36, 27.45), Math.toRadians(-90))
                .afterTime(0, robotSpecial.transferSystem(false, true))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(48.25, 27.75))
                .afterTime(0, robotSpecial.transferSystem(false, false))
                .waitSeconds(2)
                .build();

        Actions.runBlocking(new SequentialAction(ActionButterRight));
        chosenTrajectory = ButterRight;

        Action ActionPostHang= chosenTrajectory.endTrajectory().fresh()
                //approach the human player
                .afterDisp(0, verticalWristRR.verticalWristPosition(Configuration.forwardDown))
                .strafeToSplineHeading(new Vector2d(45, 20), Math.toRadians(-90))
                //grab the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .waitSeconds(.25)
                //lift the slide partly and approach the hang
                .afterTime(0, verticalSlideRR.verticalSlidePosition(-100))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(-16, 24))
                //hang
                .afterDisp(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .strafeTo(new Vector2d(-16, 28))
                //approach the human player
                .strafeToSplineHeading(new Vector2d(45, 20), Math.toRadians(-90))
                //grab the butter
                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.close))
                .waitSeconds(.25)
                //lift the slide partly and approach the hang
                .afterTime(0, verticalSlideRR.verticalSlidePosition(-100))
                .afterDisp(5, horizontalSlideRR.horizontalSlidePosition(Configuration.highBar))
                .strafeTo(new Vector2d(-16, 24))
                //hang
                .afterDisp(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterDisp(2, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
                .build();

        Actions.runBlocking(new SequentialAction(ActionPostHang));
        chosenTrajectory = PostHang;

    }
}