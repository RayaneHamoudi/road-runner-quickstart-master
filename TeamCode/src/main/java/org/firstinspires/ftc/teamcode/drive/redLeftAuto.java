package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous
public class redLeftAuto extends LinearOpMode {
    private Servo clawLeft;
    private Servo clawRight;

    private DcMotor liftLeft;
    private DcMotor liftRight;

    OpenCvInternalCamera phoneCam;
    powerPlayVisionV2 pipeline;

    enum State {
        DEFAULT,
        LIFT,
        DROP,
        DOWN,
        PARK
    }

    private State state = State.DEFAULT;

    int[] values = {700, 560, 270, 50, 0};
    int cycle = 0;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");

        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        clawLeft.setPosition(0);
        clawRight.setPosition(.5);

        //hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new powerPlayVisionV2();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            //iwasherer
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.setMsTransmissionInterval(50);



        Pose2d startPose = new Pose2d(-36.7, -65, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(73)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    state = State.LIFT;
                })
                .forward(3)
                .addTemporalMarker(() -> {
                    state = State.DROP;
                })
                .back(3)
                .strafeRight(13)
                .turn(Math.toRadians(180))


                //reaches cone tower
                //.setReversed(true)
                .forward(27)
                .addTemporalMarker(() -> {
                    //close claw and go up at cone
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    state = State.LIFT;
                })

                //cycle 1
                //wait a bit so the cone tower doesn't fall and head to tall junction
                .waitSeconds(0.3)
                .back(28)
                .turn(Math.toRadians(-135))
                .forward(6.3)
                .addTemporalMarker(() -> {
                    clawLeft.setPosition(0.5);
                    clawRight.setPosition(0);
                    state = State.DROP;
                })
                .waitSeconds(0.5)
                .back(7)
                .setReversed(true)

                .turn(Math.toRadians(135))
                .forward(28)
                //cycle 2
                .addTemporalMarker(() -> {
                    //close claw and go up at cone
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    state = State.LIFT;
                })
                .waitSeconds(0.5)

                //.setReversed(true)
                //.lineToLinearHeading(new Pose2d(-25, -9, Math.toRadians(90)))
                .back(27)
                .turn(Math.toRadians(-135))
                .forward(6.5)
                .addTemporalMarker(() -> {
                    state = State.DROP;
                })
                .waitSeconds(0.5)
                .back(7)
                .setReversed(true)
                .turn(Math.toRadians(45))
                .addTemporalMarker(() -> {
                    //park in corresponding signal sleeve marker
                    state = State.PARK;
                })
                .build();

        TrajectorySequence parkingTraj1 = drive.trajectorySequenceBuilder(traj1.end())
                .strafeLeft(26)
                .build();
        TrajectorySequence parkingTraj2 = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(3)
                .build();
        TrajectorySequence parkingTraj3 = drive.trajectorySequenceBuilder(traj1.end())
                .strafeRight(26)
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getPosition());
            telemetry.update();
            // do not do what the bottom says
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        powerPlayVisionV2.ParkingPosition POSITION = pipeline.getPosition();
        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case LIFT: {
                    liftRight.setTargetPosition(4400);
                    liftLeft.setTargetPosition(4400);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.7);
                    liftRight.setPower(0.7);


                    if (liftLeft.getCurrentPosition() > liftRight.getTargetPosition() || liftLeft.getCurrentPosition() > liftRight.getTargetPosition()) {
                        liftLeft.setPower(0.05);
                        liftRight.setPower(0.05);
                    }
                    break;

                }
                case DROP: {
                    clawLeft.setPosition(0.5);
                    clawRight.setPosition(0);
                    state = State.DOWN;
                    break;
                }
                case DOWN: {
                    liftRight.setTargetPosition(values[cycle]);
                    liftLeft.setTargetPosition(values[cycle]);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.7);
                    liftRight.setPower(0.7);

                    if (liftLeft.getCurrentPosition() < liftRight.getTargetPosition() || liftLeft.getCurrentPosition() < liftRight.getTargetPosition()) {
                        liftLeft.setPower(0);
                        liftRight.setPower(0);
                        state = State.DEFAULT;
                        cycle++;
                    }
                    break;
                }
                case PARK: {
                    telemetry.addLine("Parking");
                    telemetry.update();
                    switch (POSITION){
                        case LEFT:{
                            drive.followTrajectorySequence(parkingTraj3);
                            state = State.DEFAULT;
                            break;
                        }
                        case CENTER:{
                            drive.followTrajectorySequence(parkingTraj2);
                            state = State.DEFAULT;
                            break;
                        }
                        case RIGHT:{
                            drive.followTrajectorySequence(parkingTraj1);
                            state = State.DEFAULT;
                            break;
                        }
                    }
                    break;
                }
            }
            drive.update();
        }
    }
}