package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.nio.Buffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

public class powerPlayVisionV1 extends OpenCvPipeline {

    Scalar cyanLow = new Scalar(103.4, 154.4, 136.0);
    Scalar cyanHigh = new Scalar(255, 255, 255);

    Scalar pinkLow = new Scalar(92, 136, 127.4);
    Scalar pinkHigh = new Scalar(218, 160, 179.9);

    Scalar greenLow = new Scalar(65.2, 62.3, 53.8);
    Scalar greenHigh = new Scalar(120, 183, 191);

    Mat cyanMat = new Mat();
    Mat pinkMat = new Mat();
    Mat greenMat = new Mat();
    Mat ycrcbMat = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    Mat sum = new Mat();

    enum Stage
    {
        input,
        cyan,
        pink,
        green,
        sum,
        CONTOURS
    }
    enum Position
    {
        CENTER,
        RIGHT,
        LEFT
    }
    private Stage[] stages = Stage.values();
    int stageNum = 0;
    private volatile Position position = Position.LEFT;
    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }
    @Override
    public Mat processFrame(Mat input) {
        doOperations(input);
        switch (stages[stageNum])
        {
            case input:
            {
                return input;
            }
            case cyan:
            {
                return cyanMat;
            }
            case pink:
            {
                return pinkMat;
            }
            case green:
            {
                return greenMat;
            }
            case sum:
            {
                return sum;
            }
            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }
        }
        return input;
    }
    private void doOperations(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, cyanLow, cyanHigh, cyanMat);
        Core.inRange(ycrcbMat, pinkLow, pinkHigh, pinkMat);
        Core.inRange(ycrcbMat, greenLow, greenHigh, greenMat);
        Core.add(cyanMat, pinkMat, sum);
        Core.add(sum, greenMat, sum);

        ArrayList<MatOfPoint> buffer = new ArrayList<>();
        ArrayList<MatOfPoint> storage = new ArrayList<>();
        input.copyTo(contoursOnPlainImageMat);

        Imgproc.findContours(sum, buffer, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double [] arr = {0, 100};
        filterContours(buffer,0,0,20,200,20,300, arr, 1000,
                0, 0, 1000, input, storage);
        Imgproc.drawContours(contoursOnPlainImageMat, storage, -1, new Scalar(0, 255, 0), 1, 8);

        sum.release();
        Core.bitwise_and(input, input, sum, cyanMat);
        Core.bitwise_and(input, input, sum, pinkMat);
        Core.bitwise_and(input, input, sum, greenMat);


        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < storage.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(storage.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }
        if(storage.size() > 0){
            MatOfPoint maxContour = storage.get(maxValIdx);
            Rect br = Imgproc.boundingRect(maxContour);
            Mat submat = input.submat(br);

            Scalar mean = Core.mean(submat);
            double red = mean.val[0];
            double green = mean.val[1];
            double blue = mean.val[2];
            if (red > green && red > blue) {
                position = Position.CENTER;
            } else if (green > red && green > blue) {
                position = Position.RIGHT;
            } else {
                position = Position.LEFT;
            }
        }
    }
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, Mat input, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
                continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount) continue;
            final double ratio = bb.width / (double) bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }
    public Position getPosition(){
        return position;



    }
}
