package org.openbot.markerfollow;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.camera.core.ImageProxy;

import org.openbot.R;
import org.openbot.common.CameraFragment;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class MarkerFollowFragment extends CameraFragment {

    private TextView statusText;
    private MarkerOverlayView markerOverlayView;

    private UdpTargetSender udpSender;

    // 你的电脑 WLAN IPv4 地址
    private static final String PC_IP = "183.173.177.106";
    private static final int PC_PORT = 4210;

    private long lastProcessTime = 0;
    private static final long PROCESS_INTERVAL_MS = 150;

    // 紫色 HSV 参数
    private static final float H_MIN = 250f;
    private static final float H_MAX = 320f;
    private static final float S_MIN = 0.25f;
    private static final float V_MIN = 0.20f;

    // 采样步长，越大越省性能，但框会粗糙
    private static final int SAMPLE_STEP = 8;

    // 单个紫色色块太小就不要
    private static final int MIN_COMPONENT_SAMPLES = 8;
    private static final float MIN_BLOB_AREA_RATIO = 0.0010f;

    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater,
            @Nullable ViewGroup container,
            @Nullable Bundle savedInstanceState) {

        View view = inflateFragment(
                R.layout.fragment_marker_follow,
                inflater,
                container
        );

        statusText = view.findViewById(R.id.marker_follow_status);
        markerOverlayView = view.findViewById(R.id.marker_overlay);

        udpSender = new UdpTargetSender(PC_IP, PC_PORT);

        if (statusText != null) {
            statusText.setText(
                    "Marker Follow Page Running\n"
                            + "Multi purple blob detection\n"
                            + "UDP target: " + PC_IP + ":" + PC_PORT
            );
        }

        return view;
    }

    @Override
    public void processFrame(Bitmap bitmap, ImageProxy imageProxy) {
        try {
            long now = System.currentTimeMillis();

            if (bitmap != null && now - lastProcessTime > PROCESS_INTERVAL_MS) {
                lastProcessTime = now;

                DetectResult result = detectPurpleBlobs(bitmap);

                sendTargetMessage(result);
                updateUI(result);
            }

        } finally {
            if (imageProxy != null) {
                imageProxy.close();
            }
        }
    }

    @Override
    public void processControllerKeyData(String data) {
        // 当前不用 OpenBot controller，先留空
    }

    @Override
    public void processUSBData(String data) {
        // 当前不处理 USB 数据，先留空
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();

        if (udpSender != null) {
            udpSender.close();
            udpSender = null;
        }

        statusText = null;
        markerOverlayView = null;
    }

    private DetectResult detectPurpleBlobs(Bitmap bitmap) {
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();

        int gridW = (width + SAMPLE_STEP - 1) / SAMPLE_STEP;
        int gridH = (height + SAMPLE_STEP - 1) / SAMPLE_STEP;

        boolean[] mask = new boolean[gridW * gridH];
        boolean[] visited = new boolean[gridW * gridH];

        float[] hsv = new float[3];

        int totalSamples = gridW * gridH;

        // 1. 先生成紫色像素的网格 mask
        for (int gy = 0; gy < gridH; gy++) {
            for (int gx = 0; gx < gridW; gx++) {
                int x = Math.min(gx * SAMPLE_STEP, width - 1);
                int y = Math.min(gy * SAMPLE_STEP, height - 1);

                int pixel = bitmap.getPixel(x, y);
                Color.colorToHSV(pixel, hsv);

                float h = hsv[0];
                float s = hsv[1];
                float v = hsv[2];

                if (isMarkerColor(h, s, v)) {
                    mask[index(gx, gy, gridW)] = true;
                }
            }
        }

        List<PurpleBlob> blobs = new ArrayList<>();

        // 2. 连通区域检测：把一块一块紫色分开
        for (int gy = 0; gy < gridH; gy++) {
            for (int gx = 0; gx < gridW; gx++) {
                int startIndex = index(gx, gy, gridW);

                if (!mask[startIndex] || visited[startIndex]) {
                    continue;
                }

                PurpleBlob blob = floodFillBlob(
                        gx,
                        gy,
                        mask,
                        visited,
                        gridW,
                        gridH,
                        width,
                        height,
                        totalSamples
                );

                if (blob.sampleCount >= MIN_COMPONENT_SAMPLES
                        && blob.areaRatio >= MIN_BLOB_AREA_RATIO) {
                    blobs.add(blob);
                }
            }
        }

        // 3. 选一个当前目标：先简单选面积最大的紫色色块
        int selectedIndex = -1;
        float bestArea = 0;

        for (int i = 0; i < blobs.size(); i++) {
            PurpleBlob blob = blobs.get(i);
            if (blob.areaRatio > bestArea) {
                bestArea = blob.areaRatio;
                selectedIndex = i;
            }
        }

        DetectResult result = new DetectResult();
        result.imageWidth = width;
        result.imageHeight = height;
        result.blobs = blobs;
        result.selectedIndex = selectedIndex;

        if (selectedIndex >= 0) {
            result.visible = true;
            result.targetBlob = blobs.get(selectedIndex);
        } else {
            result.visible = false;
            result.targetBlob = null;
        }

        return result;
    }

    private PurpleBlob floodFillBlob(
            int startGx,
            int startGy,
            boolean[] mask,
            boolean[] visited,
            int gridW,
            int gridH,
            int imageWidth,
            int imageHeight,
            int totalSamples) {

        ArrayDeque<int[]> queue = new ArrayDeque<>();
        queue.add(new int[]{startGx, startGy});
        visited[index(startGx, startGy, gridW)] = true;

        int minGx = startGx;
        int maxGx = startGx;
        int minGy = startGy;
        int maxGy = startGy;
        int count = 0;

        while (!queue.isEmpty()) {
            int[] current = queue.removeFirst();

            int gx = current[0];
            int gy = current[1];

            count++;

            if (gx < minGx) minGx = gx;
            if (gx > maxGx) maxGx = gx;
            if (gy < minGy) minGy = gy;
            if (gy > maxGy) maxGy = gy;

            // 4 邻域：上下左右
            int[][] neighbors = {
                    {gx - 1, gy},
                    {gx + 1, gy},
                    {gx, gy - 1},
                    {gx, gy + 1}
            };

            for (int[] nb : neighbors) {
                int nx = nb[0];
                int ny = nb[1];

                if (nx < 0 || nx >= gridW || ny < 0 || ny >= gridH) {
                    continue;
                }

                int ni = index(nx, ny, gridW);

                if (mask[ni] && !visited[ni]) {
                    visited[ni] = true;
                    queue.add(new int[]{nx, ny});
                }
            }
        }

        PurpleBlob blob = new PurpleBlob();

        blob.sampleCount = count;
        blob.areaRatio = (float) count / (float) totalSamples;

        blob.minX = minGx * SAMPLE_STEP;
        blob.minY = minGy * SAMPLE_STEP;
        blob.maxX = Math.min((maxGx + 1) * SAMPLE_STEP, imageWidth - 1);
        blob.maxY = Math.min((maxGy + 1) * SAMPLE_STEP, imageHeight - 1);

        blob.centerX = (blob.minX + blob.maxX) / 2.0f;
        blob.centerY = (blob.minY + blob.maxY) / 2.0f;

        blob.normalizedX = (blob.centerX - imageWidth / 2.0f) / (imageWidth / 2.0f);

        return blob;
    }

    private int index(int gx, int gy, int gridW) {
        return gy * gridW + gx;
    }

    private boolean isMarkerColor(float h, float s, float v) {
        return h >= H_MIN && h <= H_MAX && s >= S_MIN && v >= V_MIN;
    }

    private void sendTargetMessage(DetectResult result) {
        if (udpSender == null) {
            return;
        }

        String message;

        if (result.visible && result.targetBlob != null) {
            PurpleBlob blob = result.targetBlob;

            // 最后一个数现在改成 quality，1.00 表示明确看到目标
            message = String.format(
                    Locale.US,
                    "TARGET,1,%.2f,%.4f,1.00",
                    blob.normalizedX,
                    blob.areaRatio
            );
        } else {
            message = "TARGET,0,0,0,0";
        }

        udpSender.send(message);
    }

    private void updateUI(DetectResult result) {
        if (!isAdded()) {
            return;
        }

        requireActivity().runOnUiThread(() -> {
            if (markerOverlayView != null) {
                if (result.blobs == null || result.blobs.isEmpty()) {
                    markerOverlayView.clearBoxes();
                } else {
                    List<MarkerOverlayView.BoxData> boxes = new ArrayList<>();

                    for (int i = 0; i < result.blobs.size(); i++) {
                        PurpleBlob blob = result.blobs.get(i);

                        boxes.add(new MarkerOverlayView.BoxData(
                                blob.minX,
                                blob.minY,
                                blob.maxX,
                                blob.maxY,
                                blob.normalizedX,
                                blob.areaRatio,
                                i == result.selectedIndex
                        ));
                    }

                    markerOverlayView.updateBoxes(
                            boxes,
                            result.imageWidth,
                            result.imageHeight
                    );
                }
            }

            if (statusText != null) {
                if (result.visible && result.targetBlob != null) {
                    PurpleBlob blob = result.targetBlob;

                    String direction;
                    if (blob.normalizedX < -0.2f) {
                        direction = "LEFT";
                    } else if (blob.normalizedX > 0.2f) {
                        direction = "RIGHT";
                    } else {
                        direction = "CENTER";
                    }

                    String text = String.format(
                            Locale.US,
                            "PURPLE BLOBS: %d\n"
                                    + "TARGET = blob %d\n"
                                    + "x = %.2f\n"
                                    + "size = %.4f\n"
                                    + "direction = %s\n"
                                    + "UDP: TARGET,1,%.2f,%.4f,1.00",
                            result.blobs.size(),
                            result.selectedIndex + 1,
                            blob.normalizedX,
                            blob.areaRatio,
                            direction,
                            blob.normalizedX,
                            blob.areaRatio
                    );

                    statusText.setText(text);
                    statusText.setBackgroundColor(Color.parseColor("#AA4B0082"));

                } else {
                    String text = "PURPLE BLOBS: 0\n"
                            + "TARGET LOST\n"
                            + "UDP: TARGET,0,0,0,0";

                    statusText.setText(text);
                    statusText.setBackgroundColor(Color.parseColor("#AA660000"));
                }
            }
        });
    }

    private static class DetectResult {
        boolean visible = false;
        int imageWidth = 1;
        int imageHeight = 1;
        List<PurpleBlob> blobs = new ArrayList<>();
        int selectedIndex = -1;
        PurpleBlob targetBlob = null;
    }

    private static class PurpleBlob {
        int sampleCount = 0;

        float minX = 0;
        float minY = 0;
        float maxX = 0;
        float maxY = 0;

        float centerX = 0;
        float centerY = 0;

        float normalizedX = 0;
        float areaRatio = 0;
    }
}