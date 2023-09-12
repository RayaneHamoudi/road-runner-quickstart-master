package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class splineAuto extends LinearOpMode{
    private Servo clawLeft;
    private Servo clawRight;


    private DcMotor liftLeft;
    private DcMotor liftRight;

    OpenCvInternalCamera phoneCam;
    powerPlayVisionV1 pipeline;

    enum State{
        DEFAULT,
        LIFT,
        DROP,
        DOWN
    }
    private State state = State.DEFAULT;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");

        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        clawLeft.setPosition(0);
        clawRight.setPosition(.5);

        //hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new powerPlayVisionV1();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
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

        Pose2d startPose = new Pose2d(37.2, 63.2, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(37.2,63.2, Math.toRadians(-90))).setTangent(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    state = State.DEFAULT;
                })
                .splineToSplineHeading(new Pose2d(36.4,56, Math.toRadians(-34)), Math.toRadians(-34))
                .splineToSplineHeading(new Pose2d(41.2,53.2, Math.toRadians(-29)), Math.toRadians(-29))
                .addDisplacementMarker(() -> {
                                    clawLeft.setPosition(0.5);
                                    clawRight.setPosition(0);
                    // open claw and back away from grnd junc
                })
                .splineToSplineHeading(new Pose2d(38.8,49.6, Math.toRadians(158)), Math.toRadians(158))
                .splineToSplineHeading(new Pose2d(34.8,47.6, Math.toRadians(-87)), Math.toRadians(-87))
                .splineToSplineHeading(new Pose2d(35,45, Math.toRadians(-81)), Math.toRadians(-81))
                .addTemporalMarker(() -> {
                                    clawLeft.setPosition(0);
                                    clawRight.setPosition(0.5);
                    // pick up signal sleeve cone
                    //
                })
                .splineToSplineHeading(new Pose2d(36,32.4, Math.toRadians(-92)), Math.toRadians(-92))
                .splineToSplineHeading(new Pose2d(35.6,22.8, Math.toRadians(171)), Math.toRadians(171))
                .splineToSplineHeading(new Pose2d(29,24, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(() -> {
                    state = State.LIFT;
                })
                .build();




        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getPosition());
            telemetry.update();
            // do not do what the bottom says
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        powerPlayVisionV1.Position POSITION = pipeline.getPosition();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(traj1);


        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case LIFT: {
                    telemetry.addData("Lifting", "var");
                    telemetry.update();
                    liftRight.setTargetPosition(2850);
                    liftLeft.setTargetPosition(2850);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.5);
                    liftRight.setPower(0.5);


                    if (liftLeft.getCurrentPosition() > liftRight.getTargetPosition() || liftLeft.getCurrentPosition() > liftRight.getTargetPosition()) {
                        liftLeft.setPower(0.01);
                        liftRight.setPower(0.01);
                        //drive.followTrajectorySequence(traj2);
                        state = State.DROP;
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
                    liftRight.setTargetPosition(50);
                    liftLeft.setTargetPosition(50);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.5);
                    liftRight.setPower(0.5);


                    if (liftLeft.getCurrentPosition() < liftRight.getTargetPosition() || liftLeft.getCurrentPosition() < liftRight.getTargetPosition()) {
                        liftLeft.setPower(0.1);
                        liftRight.setPower(0.1);
                        state = State.DEFAULT;
                    }
                    break;
                }
            }
            drive.update();
        }

    }}