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

import java.util.Arrays;
import java.util.List;

@Autonomous
public class newAuto extends LinearOpMode{
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

    int [] values = {570, 390, 250, 65, 0};
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

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    state = State.DEFAULT;
                })
                .forward(4)
                .turn(Math.toRadians(45))
                .forward(10)
                .addDisplacementMarker(() -> {
                    clawLeft.setPosition(0.5);
                    clawRight.setPosition(0);
                    // open claw and back away from grnd junc
                })
                .back(10)
                .turn(Math.toRadians(-45))
                .forward(18)

                .addTemporalMarker(() -> {
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    // pick up signal sleeve cone
                    //
                })
                .turn(Math.toRadians(-90))
                .strafeLeft(19)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    state = State.LIFT;
                })
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(4)
                .build();


        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                //.lineToSplineHeading(new Pose2d(15, -45, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-49, -12, Math.toRadians(180)))
                .strafeRight(1)
                .forward(15)
                .addTemporalMarker(() -> {
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.5);
                    // pick up stacked cones
                    //
                })
                .addTemporalMarker(() -> {
                    state = State.LIFT;
                })
                .waitSeconds(1)
              /*  .back(22)
                .turn(Math.toRadians(45))
*/
                .splineToConstantHeading(new Vector2d(-29, -19), Math.toRadians(180))
                .turn(Math.toRadians(-45))
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
                    liftRight.setTargetPosition(2850);
                    liftLeft.setTargetPosition(2850);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.5);
                    liftRight.setPower(0.5);


                    if (liftLeft.getCurrentPosition() > liftRight.getTargetPosition() || liftLeft.getCurrentPosition() > liftRight.getTargetPosition()) {
                        liftLeft.setPower(0.01);
                        liftRight.setPower(0.01);
                        drive.followTrajectorySequence(traj2);
                        state = State.DROP;
                    }
                    break;

                }
                case DROP: {
                    clawLeft.setPosition(0.5);
                    clawRight.setPosition(0);
                    drive.followTrajectorySequenceAsync(traj3);
                    state = State.DOWN;
                    break;
                }
                case DOWN: {
                    liftRight.setTargetPosition(values[cycle]);
                    liftLeft.setTargetPosition(values[cycle]);

                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftLeft.setPower(0.5);
                    liftRight.setPower(0.5);


                    if (liftLeft.getCurrentPosition() < liftRight.getTargetPosition() || liftLeft.getCurrentPosition() < liftRight.getTargetPosition()) {
                        liftLeft.setPower(0);
                        liftRight.setPower(0);
                        state = State.DEFAULT;
                        cycle++;
                    }
                    break;
                }
            }
            drive.update();
            telemetry.addData("POSE", drive.getPoseEstimate());
            telemetry.update();
        }

    }}