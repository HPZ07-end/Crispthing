package org.openbot.markerfollow;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class MarkerOverlayView extends View {

    public static class BoxData {
        public float minX;
        public float minY;
        public float maxX;
        public float maxY;
        public float normalizedX;
        public float areaRatio;
        public boolean selected;

        public BoxData(
                float minX,
                float minY,
                float maxX,
                float maxY,
                float normalizedX,
                float areaRatio,
                boolean selected) {
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
            this.normalizedX = normalizedX;
            this.areaRatio = areaRatio;
            this.selected = selected;
        }
    }

    private final Paint normalBoxPaint = new Paint();
    private final Paint selectedBoxPaint = new Paint();
    private final Paint textPaint = new Paint();

    private final List<BoxData> boxes = new ArrayList<>();

    private int imageWidth = 1;
    private int imageHeight = 1;

    public MarkerOverlayView(Context context) {
        super(context);
        init();
    }

    public MarkerOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public MarkerOverlayView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        normalBoxPaint.setColor(Color.MAGENTA);
        normalBoxPaint.setStyle(Paint.Style.STROKE);
        normalBoxPaint.setStrokeWidth(4f);
        normalBoxPaint.setAntiAlias(true);

        selectedBoxPaint.setColor(Color.YELLOW);
        selectedBoxPaint.setStyle(Paint.Style.STROKE);
        selectedBoxPaint.setStrokeWidth(8f);
        selectedBoxPaint.setAntiAlias(true);

        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(34f);
        textPaint.setAntiAlias(true);
        textPaint.setStyle(Paint.Style.FILL);
    }

    public void updateBoxes(List<BoxData> newBoxes, int imageWidth, int imageHeight) {
        boxes.clear();

        if (newBoxes != null) {
            boxes.addAll(newBoxes);
        }

        this.imageWidth = Math.max(imageWidth, 1);
        this.imageHeight = Math.max(imageHeight, 1);

        invalidate();
    }

    public void clearBoxes() {
        boxes.clear();
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (boxes.isEmpty()) {
            return;
        }

        float viewWidth = getWidth();
        float viewHeight = getHeight();

        float scaleX = viewWidth / imageWidth;
        float scaleY = viewHeight / imageHeight;

        for (int i = 0; i < boxes.size(); i++) {
            BoxData box = boxes.get(i);

            float left = box.minX * scaleX;
            float top = box.minY * scaleY;
            float right = box.maxX * scaleX;
            float bottom = box.maxY * scaleY;

            Paint paint = box.selected ? selectedBoxPaint : normalBoxPaint;
            canvas.drawRect(left, top, right, bottom, paint);

            String info = String.format(
                    Locale.US,
                    "%s%d x=%.2f size=%.4f",
                    box.selected ? "TARGET " : "BLOB ",
                    i + 1,
                    box.normalizedX,
                    box.areaRatio
            );

            canvas.drawText(info, left, Math.max(top - 12, 40), textPaint);
        }
    }
}