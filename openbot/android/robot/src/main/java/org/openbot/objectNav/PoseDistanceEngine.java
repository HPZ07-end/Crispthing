package org.openbot.objectNav;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import com.google.mediapipe.framework.image.BitmapImageBuilder;
import com.google.mediapipe.framework.image.MPImage;
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark;
import com.google.mediapipe.tasks.core.BaseOptions;
import com.google.mediapipe.tasks.vision.core.RunningMode;
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker;
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarkerResult;

import java.util.List;
import java.util.Locale;

/**
 * 使用肩膀和髋部关键点计算人体躯干在画面中的相对高度。
 *
 * 当前阶段只负责：
 * 1. 加载 MediaPipe Pose 模型
 * 2. 识别肩、髋四个关键点
 * 3. 计算 torsoScale
 *
 * 暂时不计算真实厘米距离。
 */
public final class PoseDistanceEngine implements AutoCloseable {

    private static final String TAG = "PoseDistanceEngine";

    private static final String MODEL_ASSET_PATH =
            "pose/pose_landmarker_lite.task";

    // MediaPipe Pose 关键点编号
    private static final int LEFT_SHOULDER = 11;
    private static final int RIGHT_SHOULDER = 12;
    private static final int LEFT_HIP = 23;
    private static final int RIGHT_HIP = 24;

    // 第一版先使用较宽松的阈值
    private static final float MIN_VISIBILITY = 0.50f;

    private PoseLandmarker poseLandmarker;

    public PoseDistanceEngine(Context context) {
        BaseOptions baseOptions =
                BaseOptions.builder()
                        .setModelAssetPath(MODEL_ASSET_PATH)
                        .build();

        PoseLandmarker.PoseLandmarkerOptions options =
                PoseLandmarker.PoseLandmarkerOptions.builder()
                        .setBaseOptions(baseOptions)
                        .setRunningMode(RunningMode.IMAGE)
                        .setNumPoses(1)
                        .setMinPoseDetectionConfidence(0.50f)
                        .setMinPosePresenceConfidence(0.50f)
                        .setMinTrackingConfidence(0.50f)
                        .build();

        poseLandmarker =
                PoseLandmarker.createFromOptions(
                        context.getApplicationContext(),
                        options);

        Log.d(TAG, "Pose model loaded successfully");
    }

    /**
     * 对一张完整 Bitmap 进行肩髋测量。
     *
     * 注意：
     * MediaPipe IMAGE 模式是同步执行的，
     * 后面必须从后台线程调用，不能长期放在主线程。
     */
    public Measurement measure(Bitmap sourceBitmap) {
        if (sourceBitmap == null
                || sourceBitmap.isRecycled()
                || poseLandmarker == null) {

            return Measurement.invalid("bitmap_or_engine_invalid");
        }

        Bitmap inputBitmap = sourceBitmap;
        boolean ownsBitmapCopy = false;

        if (sourceBitmap.getConfig() != Bitmap.Config.ARGB_8888) {
            inputBitmap =
                    sourceBitmap.copy(
                            Bitmap.Config.ARGB_8888,
                            false);

            ownsBitmapCopy = true;
        }

        if (inputBitmap == null) {
            return Measurement.invalid("bitmap_copy_failed");
        }

        MPImage mpImage = null;

        try {
            mpImage =
                    new BitmapImageBuilder(inputBitmap)
                            .build();

            PoseLandmarkerResult result =
                    poseLandmarker.detect(mpImage);

            List<List<NormalizedLandmark>> detectedPoses =
                    result.landmarks();

            if (detectedPoses == null
                    || detectedPoses.isEmpty()) {

                return Measurement.invalid("no_pose");
            }

            List<NormalizedLandmark> landmarks =
                    detectedPoses.get(0);

            if (landmarks == null
                    || landmarks.size() <= RIGHT_HIP) {

                return Measurement.invalid(
                        "landmark_count_invalid");
            }

            NormalizedLandmark leftShoulder =
                    landmarks.get(LEFT_SHOULDER);

            NormalizedLandmark rightShoulder =
                    landmarks.get(RIGHT_SHOULDER);

            NormalizedLandmark leftHip =
                    landmarks.get(LEFT_HIP);

            NormalizedLandmark rightHip =
                    landmarks.get(RIGHT_HIP);

            float leftShoulderVisibility =
                    getVisibility(leftShoulder);

            float rightShoulderVisibility =
                    getVisibility(rightShoulder);

            float leftHipVisibility =
                    getVisibility(leftHip);

            float rightHipVisibility =
                    getVisibility(rightHip);

            float minimumVisibility =
                    Math.min(
                            Math.min(
                                    leftShoulderVisibility,
                                    rightShoulderVisibility),
                            Math.min(
                                    leftHipVisibility,
                                    rightHipVisibility));

            if (minimumVisibility < MIN_VISIBILITY) {
                return Measurement.invalid(
                        "landmark_visibility_low");
            }

            /*
             * 由于 y 已经除以整张输入图像的高度，
             * 所以下面的高度天然就是：
             *
             * 躯干像素高度 / 整张画面像素高度
             */

            float leftTorsoHeight =
                    Math.abs(
                            leftHip.y()
                                    - leftShoulder.y());

            float rightTorsoHeight =
                    Math.abs(
                            rightHip.y()
                                    - rightShoulder.y());

            float shoulderCenterY =
                    (leftShoulder.y()
                            + rightShoulder.y()) / 2.0f;

            float hipCenterY =
                    (leftHip.y()
                            + rightHip.y()) / 2.0f;

            float centerTorsoHeight =
                    Math.abs(
                            hipCenterY
                                    - shoulderCenterY);

            /*
             * 三组高度取中位数。
             * 可以减少某一侧关键点轻微抖动造成的影响。
             */
            float torsoScale =
                    median(
                            leftTorsoHeight,
                            rightTorsoHeight,
                            centerTorsoHeight);

            if (!Float.isFinite(torsoScale)
                    || torsoScale <= 0.01f) {

                return Measurement.invalid(
                        "torso_scale_invalid");
            }

            Measurement measurement =
                    Measurement.valid(
                            torsoScale,
                            leftTorsoHeight,
                            rightTorsoHeight,
                            centerTorsoHeight,
                            minimumVisibility);

            Log.d(
                    TAG,
                    String.format(
                            Locale.US,
                            "valid=true torsoScale=%.4f "
                                    + "left=%.4f right=%.4f "
                                    + "center=%.4f visibility=%.3f",
                            measurement.getTorsoScale(),
                            measurement.getLeftTorsoHeight(),
                            measurement.getRightTorsoHeight(),
                            measurement.getCenterTorsoHeight(),
                            measurement.getMinimumVisibility()));

            return measurement;

        } catch (RuntimeException exception) {
            Log.e(
                    TAG,
                    "Pose measurement failed",
                    exception);

            return Measurement.invalid(
                    "pose_exception");

        } finally {
            if (mpImage != null) {
                mpImage.close();
            }

            if (ownsBitmapCopy
                    && inputBitmap != null
                    && !inputBitmap.isRecycled()) {

                inputBitmap.recycle();
            }
        }
    }

    private static float getVisibility(
            NormalizedLandmark landmark) {

        if (landmark == null) {
            return 0.0f;
        }

        return landmark
                .visibility()
                .orElse(0.0f);
    }

    private static float median(
            float first,
            float second,
            float third) {

        return Math.max(
                Math.min(first, second),
                Math.min(
                        Math.max(first, second),
                        third));
    }

    @Override
    public void close() {
        if (poseLandmarker != null) {
            poseLandmarker.close();
            poseLandmarker = null;
        }

        Log.d(TAG, "Pose engine closed");
    }

    public static final class Measurement {

        private final boolean valid;
        private final float torsoScale;
        private final float leftTorsoHeight;
        private final float rightTorsoHeight;
        private final float centerTorsoHeight;
        private final float minimumVisibility;
        private final String invalidReason;

        private Measurement(
                boolean valid,
                float torsoScale,
                float leftTorsoHeight,
                float rightTorsoHeight,
                float centerTorsoHeight,
                float minimumVisibility,
                String invalidReason) {

            this.valid = valid;
            this.torsoScale = torsoScale;
            this.leftTorsoHeight = leftTorsoHeight;
            this.rightTorsoHeight = rightTorsoHeight;
            this.centerTorsoHeight = centerTorsoHeight;
            this.minimumVisibility = minimumVisibility;
            this.invalidReason = invalidReason;
        }

        public static Measurement valid(
                float torsoScale,
                float leftTorsoHeight,
                float rightTorsoHeight,
                float centerTorsoHeight,
                float minimumVisibility) {

            return new Measurement(
                    true,
                    torsoScale,
                    leftTorsoHeight,
                    rightTorsoHeight,
                    centerTorsoHeight,
                    minimumVisibility,
                    "");
        }

        public static Measurement invalid(
                String reason) {

            return new Measurement(
                    false,
                    -1.0f,
                    -1.0f,
                    -1.0f,
                    -1.0f,
                    0.0f,
                    reason);
        }

        public boolean isValid() {
            return valid;
        }

        public float getTorsoScale() {
            return torsoScale;
        }

        public float getLeftTorsoHeight() {
            return leftTorsoHeight;
        }

        public float getRightTorsoHeight() {
            return rightTorsoHeight;
        }

        public float getCenterTorsoHeight() {
            return centerTorsoHeight;
        }

        public float getMinimumVisibility() {
            return minimumVisibility;
        }

        public String getInvalidReason() {
            return invalidReason;
        }
    }
}