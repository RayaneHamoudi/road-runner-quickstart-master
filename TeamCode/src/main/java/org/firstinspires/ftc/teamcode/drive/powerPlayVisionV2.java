package org.firstinspires.ftc.teamcode.drive;

import org.opencv.core.Core;

import org.opencv.core.Mat;

import org.opencv.core.Point;

import org.opencv.core.Rect;

import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;



public class powerPlayVisionV2 extends OpenCvPipeline {

    public enum ParkingPosition {

        LEFT,

        CENTER,

        RIGHT

    }



    // TOPLEFT anchor point for the bounding box

    public static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(120, 120);

    // Width and height for the bounding box

    public static int REGION_WIDTH = 15;

    public static int REGION_HEIGHT = 15;



    // Color definitions

    private final Scalar

            PINK  = new Scalar(228, 202, 2303),

    GREEN    = new Scalar(0, 255, 0),

    CYAN = new Scalar(0, 0, 120);



    // Anchor point definitions

    Point sleeve_pointA = new Point(

            SLEEVE_TOPLEFT_ANCHOR_POINT.x,

            SLEEVE_TOPLEFT_ANCHOR_POINT.y);

    Point sleeve_pointB = new Point(

            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,

            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



    // Running variable storing the parking position

    private volatile ParkingPosition position = ParkingPosition.LEFT;



    @Override

    public Mat processFrame(Mat input) {

        Point sleeve_pointA = new Point(

                SLEEVE_TOPLEFT_ANCHOR_POINT.x,

                SLEEVE_TOPLEFT_ANCHOR_POINT.y);

        Point sleeve_pointB = new Point(

                SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,

                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));

        Scalar sumColors = Core.sumElems(areaMat);



        double maxColor = Math.max(sumColors.val[0], Math.max(sumColors.val[1], sumColors.val[2]));



        if (sumColors.val[0] == maxColor) {

            position = ParkingPosition.CENTER;

            Imgproc.rectangle(

                    input,

                    sleeve_pointA,

                    sleeve_pointB,

                    PINK,

                    2

            );



        } else if (sumColors.val[1] == maxColor) {

            position = ParkingPosition.RIGHT;

            Imgproc.rectangle(

                    input,

                    sleeve_pointA,

                    sleeve_pointB,

                    GREEN,

                    2

            );

        } else {

            position = ParkingPosition.LEFT;

            Imgproc.rectangle(

                    input,

                    sleeve_pointA,

                    sleeve_pointB,

                    CYAN,

                    2

            );

        }



        areaMat.release();
        return input;

    }



    // Returns an enum being the current position where the robot will park

    public ParkingPosition getPosition() {

        return position;

    }

}