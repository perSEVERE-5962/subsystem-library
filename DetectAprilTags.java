// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vec3;

import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image and sent to the dashboard.
 *
 * <p>Be aware that the performance on this is much worse than a coprocessor solution!
 */
public class DetectAprilTags extends SubsystemBase {
  private static DetectAprilTags instance;

  private static Vec3 posSingle = new Vec3(0, 0, 0);
  private static ArrayList<Vec3> posArray = new ArrayList<>();
  private static Vec3 rotSingle = new Vec3(0, 0, 0);
  private static ArrayList<Vec3> rotArray = new ArrayList<>();
  public static int amountOfDetections = 0;
  private static ArrayList<Integer> tagId = new ArrayList<>();

  private final String family = "tag36h11"; // Usual tag family that FRC uses
  private final double tagSize = Units.inchesToMeters(6.75); // Units are in meters
  private final double[] focalData = {
    1011.3749416937393,
    1008.5391755084075,
    645.4955139388737,
    508.32877656020196
  }; // 1280 x 720 camera

  private final int brightness = 50;
  private final int resWidth = 1280;
  private final int resHeight = 720;
  private final int fps = 30;

  // Dividing some values by some amount makes the resolution worse but makes the fps better

  public static DetectAprilTags activate() {
    if (instance == null) {
      instance = new DetectAprilTags();
      instance.initDetector();
    }
    return instance;
  }

  private void initDetector() {
    var visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily(family, 0);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000 // No
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config(
            tagSize, focalData[0], focalData[1], focalData[2], focalData[3]);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // Set camera settings
    camera.setBrightness(brightness);
    camera.setResolution(resWidth, resHeight);
    camera.setFPS(fps);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    // We'll output to NT
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      amountOfDetections = detections.length;

      // have not seen any tags yet
      tags.clear();

      tagId.clear();
      posArray.clear();
      rotArray.clear();
      for (AprilTagDetection detection : detections) {
        // remember we saw this tag
        tags.add((long) detection.getId());
        tagId.add(detection.getId());

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Integer.toString(detection.getId()),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);

        // Determine pose
        Transform3d pose = estimator.estimate(detection);
        // Get rotation
        Rotation3d rot = pose.getRotation();

        // Set the values to their respective arrays
        posSingle.setAll(pose.getX(), pose.getY(), pose.getZ());
        rotSingle.setAll(rot.getX(), rot.getY(), rot.getZ());
        posArray.add(posSingle);
        rotArray.add(rotSingle);
      }

      // put list of tags onto dashboard
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    pubTags.close();
    detector.close();
  }

  /**
   * Pos 0 (X): Left/right
   *
   * <p>Pos 1 (Y): Up/down
   *
   * <p>Pos 2 (Z): Forward/backward
   *
   * @return The position of the april tag index specified, or null if such april tag doesn't exist.
   */
  public static Vec3 getAprilTagPos(int index) {
    posArray.trimToSize();
    if (index > posArray.size() - 1) {
      return null;
    }
    return posArray.get(index);
  }

  /**
   * Rot 0 (Pitch)
   *
   * <p>Rot 1 (Yaw)
   *
   * <p>Rot 2 (Roll)
   *
   * @return The rotation of the april tag index specified, or null if such april tag doesn't exist.
   */
  public static Vec3 getAprilTagRot(int index) {
    rotArray.trimToSize();
    if (index > rotArray.size() - 1) {
      return null;
    }
    return rotArray.get(index);
  }

  public static int getAprilTagIdFromIndex(int index) {
    tagId.trimToSize();
    if (index < 0 || index >= tagId.size()) {
      return -1;
    }
    return tagId.get(index);
  }

  public static Integer getAprilTagIndexFromId(int id) {
    tagId.trimToSize();
    return tagId.indexOf(id);
  }

  public static void displayAprilTagInformation() {
    for (int i = 0; i < amountOfDetections; i++) {
      Vec3 pos = getAprilTagPos(i);
      Vec3 rotPos = getAprilTagRot(i);
      Integer id = getAprilTagIdFromIndex(i);
      NetworkTableEntry entryX = NetworkTableInstance.getDefault().getEntry("Tag Pos X: " + i);
      NetworkTableEntry entryY = NetworkTableInstance.getDefault().getEntry("Tag Pos Y: " + i);
      NetworkTableEntry entryZ = NetworkTableInstance.getDefault().getEntry("Tag Pos Z: " + i);
      NetworkTableEntry entryRotX = NetworkTableInstance.getDefault().getEntry("Tag Rot X: " + i);
      NetworkTableEntry entryRotY = NetworkTableInstance.getDefault().getEntry("Tag Rot Y: " + i);
      NetworkTableEntry entryRotZ = NetworkTableInstance.getDefault().getEntry("Tag Rot Z: " + i);
      NetworkTableEntry angle = NetworkTableInstance.getDefault().getEntry("Angle to tag: " + i);
      NetworkTableEntry ID = NetworkTableInstance.getDefault().getEntry("Tag ID: " + i);
      NetworkTableEntry count = NetworkTableInstance.getDefault().getEntry("Tag count");
      if (pos != null && rotPos != null && id != null) {
        count.setInteger(amountOfDetections);
        ID.setInteger(id);
        entryX.setDouble(pos.getX());
        entryY.setDouble(pos.getY());
        entryZ.setDouble(pos.getZ());
        entryRotX.setDouble(rotPos.getX());
        entryRotY.setDouble(rotPos.getY());
        entryRotZ.setDouble(rotPos.getZ());
        angle.setDouble(Math.toDegrees(Math.atan2(pos.getX(), pos.getZ())));
      }
    }
  }
}
