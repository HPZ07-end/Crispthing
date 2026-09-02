package org.openbot.objectNav;

import android.content.Context;
import android.graphics.Bitmap;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Collections;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OnnxValue;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;
import timber.log.Timber;

public class ReIdEngine implements AutoCloseable {

    private static final String MODEL_PATH =
            "reid/osnet_x0_25_msmt17.onnx";

    private static final String INPUT_NAME = "input";

    private static final int INPUT_WIDTH = 128;
    private static final int INPUT_HEIGHT = 256;
    private static final int CHANNEL_COUNT = 3;
    private static final int EMBEDDING_SIZE = 512;

    private static final float[] MEAN = {
            0.485f,
            0.456f,
            0.406f
    };

    private static final float[] STD = {
            0.229f,
            0.224f,
            0.225f
    };

    private final OrtEnvironment environment;
    private final OrtSession session;

    public ReIdEngine(Context context)
            throws IOException, OrtException {

        environment = OrtEnvironment.getEnvironment();

        byte[] modelBytes =
                readAssetBytes(
                        context.getApplicationContext(),
                        MODEL_PATH);

        session = environment.createSession(modelBytes);

        Timber.i(
                "ReID model loaded. Inputs=%s, Outputs=%s",
                session.getInputNames(),
                session.getOutputNames());
    }

    /**
     * 将人物图片转换成已经进行 L2 归一化的 512 维特征。
     */
    public synchronized float[] extractEmbedding(Bitmap bitmap)
            throws OrtException {

        if (bitmap == null || bitmap.isRecycled()) {
            throw new IllegalArgumentException(
                    "ReID input bitmap is null or recycled");
        }

        Bitmap resizedBitmap =
                Bitmap.createScaledBitmap(
                        bitmap,
                        INPUT_WIDTH,
                        INPUT_HEIGHT,
                        true);

        FloatBuffer inputBuffer =
                bitmapToFloatBuffer(resizedBitmap);

        if (resizedBitmap != bitmap
                && !resizedBitmap.isRecycled()) {
            resizedBitmap.recycle();
        }

        long[] inputShape = {
                1,
                CHANNEL_COUNT,
                INPUT_HEIGHT,
                INPUT_WIDTH
        };

        try (
                OnnxTensor inputTensor =
                        OnnxTensor.createTensor(
                                environment,
                                inputBuffer,
                                inputShape);

                OrtSession.Result result =
                        session.run(
                                Collections.singletonMap(
                                        INPUT_NAME,
                                        inputTensor))
        ) {

            if (result.size() == 0) {
                throw new OrtException(
                        "ReID model returned no output");
            }

            OnnxValue outputValue = result.get(0);
            Object value = outputValue.getValue();

            if (!(value instanceof float[][])) {
                throw new OrtException(
                        "Unexpected ReID output type: "
                                + value.getClass().getName());
            }

            float[][] output = (float[][]) value;

            if (output.length != 1
                    || output[0].length != EMBEDDING_SIZE) {

                throw new OrtException(
                        "Unexpected ReID output shape");
            }

            float[] embedding = output[0].clone();

            l2Normalize(embedding);

            return embedding;
        }
    }

    /**
     * Android Bitmap 为 ARGB，OSNet 需要：
     * [1, 3, 256, 128] 的 RGB、CHW 排列。
     */
    private FloatBuffer bitmapToFloatBuffer(
            Bitmap bitmap) {

        int[] pixels =
                new int[INPUT_WIDTH * INPUT_HEIGHT];

        bitmap.getPixels(
                pixels,
                0,
                INPUT_WIDTH,
                0,
                0,
                INPUT_WIDTH,
                INPUT_HEIGHT);

        FloatBuffer buffer =
                ByteBuffer
                        .allocateDirect(
                                CHANNEL_COUNT
                                        * INPUT_WIDTH
                                        * INPUT_HEIGHT
                                        * Float.BYTES)
                        .order(ByteOrder.nativeOrder())
                        .asFloatBuffer();

        // R 通道
        for (int pixel : pixels) {
            float red =
                    ((pixel >> 16) & 0xFF) / 255.0f;

            buffer.put(
                    (red - MEAN[0]) / STD[0]);
        }

        // G 通道
        for (int pixel : pixels) {
            float green =
                    ((pixel >> 8) & 0xFF) / 255.0f;

            buffer.put(
                    (green - MEAN[1]) / STD[1]);
        }

        // B 通道
        for (int pixel : pixels) {
            float blue =
                    (pixel & 0xFF) / 255.0f;

            buffer.put(
                    (blue - MEAN[2]) / STD[2]);
        }

        buffer.rewind();

        return buffer;
    }

    private void l2Normalize(float[] vector) {
        double squaredSum = 0.0;

        for (float value : vector) {
            squaredSum += value * value;
        }

        double length = Math.sqrt(squaredSum);

        if (length < 1e-12) {
            return;
        }

        for (int i = 0; i < vector.length; i++) {
            vector[i] =
                    (float) (vector[i] / length);
        }
    }

    public static float cosineSimilarity(
            float[] first,
            float[] second) {

        if (first == null || second == null) {
            throw new IllegalArgumentException(
                    "Embedding cannot be null");
        }

        if (first.length != second.length) {
            throw new IllegalArgumentException(
                    "Embedding lengths do not match");
        }

        float dotProduct = 0.0f;

        for (int i = 0; i < first.length; i++) {
            dotProduct += first[i] * second[i];
        }

        // extractEmbedding() 已经执行过 L2 归一化，
        // 所以点积就是余弦相似度。
        return dotProduct;
    }

    private byte[] readAssetBytes(
            Context context,
            String assetPath)
            throws IOException {

        try (
                InputStream inputStream =
                        context.getAssets().open(assetPath);

                ByteArrayOutputStream outputStream =
                        new ByteArrayOutputStream()
        ) {

            byte[] buffer = new byte[8192];
            int length;

            while ((length = inputStream.read(buffer)) != -1) {
                outputStream.write(buffer, 0, length);
            }

            return outputStream.toByteArray();
        }
    }

    @Override
    public void close() {
        try {
            session.close();
        } catch (OrtException exception) {
            Timber.e(
                    exception,
                    "Failed to close ReID session");
        }
    }
}