package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "6455")
public class testPPAuto extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    powerPlayVisionV1 pipeline;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

        Pose2d startPose = new Pose2d(-40, -65, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


/*        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-60, -65), Math.toRadians(250)).setReversed(true)
                .splineTo(new Vector2d(-40, -40), Math.toRadians(30)).setReversed(false)
                .splineTo(new Vector2d(20, -65), Math.toRadians(0))
                .splineTo(new Vector2d(50, -65), Math.toRadians(0))
                .build();*/

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(45, 0))
                .build();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-33.2,72, Math.toRadians(-88)))
                .splineToSplineHeading(new Pose2d(-32,50.4, Math.toRadians(-68)), Math.toRadians(-68))
                .splineToSplineHeading(new Pose2d(-28.4,42.8, Math.toRadians(-34)), Math.toRadians(-34))
                .splineToSplineHeading(new Pose2d(-24,38.4, Math.toRadians(-34)), Math.toRadians(-34))
                .splineToSplineHeading(new Pose2d(-15.6,32, Math.toRadians(-32)), Math.toRadians(-32))
                .splineToSplineHeading(new Pose2d(-10.8,28.8, Math.toRadians(-23)), Math.toRadians(-23))
                .splineToSplineHeading(new Pose2d(-6,26.8, Math.toRadians(-31)), Math.toRadians(-31))
                .build();

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getPosition());
            telemetry.update();
// do not do what the bottom says
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        powerPlayVisionV1.Position POSITION = pipeline.getPosition();
        telemetry.addData("position", POSITION);
        telemetry.update();
        drive.followTrajectoryAsync(trajectory1);
        //drive.followTrajectorySequenceAsync(traj);
        while(opModeIsActive() && !isStopRequested()){
            switch (POSITION){

                case LEFT:
                {
                    /* Your autonomous code */
                    break;
                }

                case RIGHT:
                {
                    /* Your autonomous code */
                    break;
                }

                case CENTER:
                {
                    /* Your autonomous code*/
                    break;
                }
            }

            drive.update();
        }
    }
}
