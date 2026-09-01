package org.openbot.objectNav;

import android.graphics.Bitmap;

import androidx.lifecycle.ViewModel;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TargetGalleryViewModel extends ViewModel {

    private final List<Bitmap> gallerySamples =
            new ArrayList<>();

    private String currentPersonId = "";

    private boolean confirmed = false;

    private float[] galleryEmbedding;

    /*
     * 用户注册目标人物时，站在喜欢的跟随位置所得到的
     * 肩—髋完整画面尺度。
     *
     * -1 表示尚未完成距离标定。
     */
    private float baselineTorsoScale = -1.0f;

    /**
     * 开始一次全新的注册。
     * 同时清除旧身份和旧距离偏好。
     */
    public synchronized void beginNewRegistration() {
        clearSamplesInternal();

        galleryEmbedding = null;

        baselineTorsoScale = -1.0f;

        long number =
                System.currentTimeMillis()
                        % 1_000_000L;

        currentPersonId =
                String.format(
                        Locale.US,
                        "P-%06d",
                        number);

        confirmed = false;
    }

    public synchronized int addSample(
            Bitmap sample,
            int maxSamples) {

        if (sample == null) {
            return gallerySamples.size();
        }

        if (gallerySamples.size() >= maxSamples) {
            if (!sample.isRecycled()) {
                sample.recycle();
            }

            return gallerySamples.size();
        }

        gallerySamples.add(sample);

        return gallerySamples.size();
    }

    public synchronized int getSampleCount() {
        return gallerySamples.size();
    }

    public synchronized List<Bitmap> getSamplesSnapshot() {
        return new ArrayList<>(
                gallerySamples);
    }

    public synchronized String getCurrentPersonId() {
        return currentPersonId;
    }

    public synchronized void confirmCurrentPerson() {
        /*
         * 身份特征和距离基准都存在时，
         * 才允许正式确认。
         */
        confirmed =
                hasGalleryEmbedding()
                        && hasBaselineTorsoScale();
    }

    public synchronized boolean isConfirmed() {
        return confirmed;
    }

    /**
     * 设置注册时计算出的肩髋基准尺度。
     */
    public synchronized void setBaselineTorsoScale(
            float scale) {

        if (!Float.isFinite(scale)
                || scale <= 0.0f) {

            baselineTorsoScale = -1.0f;
            confirmed = false;
            return;
        }

        baselineTorsoScale = scale;
    }

    public synchronized float getBaselineTorsoScale() {
        return baselineTorsoScale;
    }

    public synchronized boolean hasBaselineTorsoScale() {
        return Float.isFinite(baselineTorsoScale)
                && baselineTorsoScale > 0.0f;
    }

    /**
     * 清除整个人物注册结果。
     */
    public synchronized void clearSamples() {
        clearSamplesInternal();

        galleryEmbedding = null;

        baselineTorsoScale = -1.0f;

        currentPersonId = "";

        confirmed = false;
    }

    private void clearSamplesInternal() {
        for (Bitmap bitmap : gallerySamples) {
            if (bitmap != null
                    && !bitmap.isRecycled()) {

                bitmap.recycle();
            }
        }

        gallerySamples.clear();
    }

    public synchronized void setGalleryEmbedding(
            float[] embedding) {

        if (embedding == null
                || embedding.length != 512) {

            galleryEmbedding = null;
            confirmed = false;
            return;
        }

        galleryEmbedding =
                embedding.clone();
    }

    public synchronized float[] getGalleryEmbedding() {
        if (galleryEmbedding == null) {
            return null;
        }

        return galleryEmbedding.clone();
    }

    public synchronized boolean hasGalleryEmbedding() {
        return galleryEmbedding != null
                && galleryEmbedding.length == 512;
    }

    /**
     * 是否同时具备完整身份和距离注册结果。
     */
    public synchronized boolean hasCompleteRegistration() {
        return confirmed
                && hasGalleryEmbedding()
                && hasBaselineTorsoScale();
    }

    @Override
    protected void onCleared() {
        clearSamples();

        super.onCleared();
    }
}