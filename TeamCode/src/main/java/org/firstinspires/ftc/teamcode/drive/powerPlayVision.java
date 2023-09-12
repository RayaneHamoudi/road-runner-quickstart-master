package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
// i was here
public class powerPlayVision extends OpenCvPipeline
{
    ArrayList<MatOfPoint> storage = new ArrayList<>();
    int numContoursFound;
    Mat dest = new Mat();
    Mat cbMat = new Mat();
    Mat redMat = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    enum Stage
    {
        BLUR,
        cB,
        redMat,
        CONTOURS;
    }
    enum Position
    {
        CENTER,
        RIGHT,
        LEFT;
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
    public Mat processFrame(Mat input)
    {
        findContours(input);

        switch (stages[stageNum])
        {
            case BLUR:
            {
                return dest;
            }

            case cB:
            {
                return cbMat;
            }

            case redMat:
            {
                return redMat;
            }
            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }
        }
        return input;
    }

    public int getNumContoursFound()
    {
        return numContoursFound;
    }

    private ArrayList<MatOfPoint> findContours(Mat input){
        storage.clear();

        Imgproc.blur(input, dest, new Size(4.0, 4.0));
        Imgproc.cvtColor(dest, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, 2);

        //Imgproc.threshold(cbMat, blueMat, 150, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, redMat, 120, 255, Imgproc.THRESH_BINARY_INV);

        //Core.add(redMat, blueMat, sum);

        Imgproc.findContours(redMat, storage, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double [] arr = {40, 100};
        ArrayList<MatOfPoint> Buffer = new ArrayList<>();
        filterContours(storage,0,0,50,200,50,300, arr, 1000,
                0, 0, 1000, input, Buffer);
        numContoursFound = Buffer.size();

        input.copyTo(contoursOnPlainImageMat);

        Imgproc.drawContours(contoursOnPlainImageMat, Buffer, -1, new Scalar(0, 255, 0), 1, 8);

        if(Buffer.size() == 1) {
            Rect r = Imgproc.boundingRect(Buffer.get(0));
            Mat imageROI = input.submat(r);
            Scalar sumColors = Core.sumElems(imageROI);
            //RGB
            double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));
            if (sumColors.val[0] == minColor) {
                position = Position.RIGHT;
            } else if (sumColors.val[1] == minColor) {
                position = Position.CENTER;
            } else {
                position = Position.LEFT;
            }
        }

        return Buffer;

    }
    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param solidity the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
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
    public Position getPosition() {
        return position;
    }
}
