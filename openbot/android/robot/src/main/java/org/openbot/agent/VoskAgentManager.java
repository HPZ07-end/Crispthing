package org.openbot.agent;

import android.app.Activity;
import android.util.Log;
import android.widget.Toast;

import org.json.JSONException;
import org.json.JSONObject;
import org.vosk.Model;
import org.vosk.Recognizer;
import org.vosk.android.RecognitionListener;
import org.vosk.android.SpeechService;
import org.vosk.android.StorageService;

import java.io.IOException;
import java.util.Locale;

public class VoskAgentManager implements RecognitionListener {

    private static final String TAG = "VoskAgentManager";

    private static final long WAKE_ACTIVE_MS = 8000L;
    private static final long COMMAND_DEBOUNCE_MS = 1500L;

    /**
     * 语音模块最终只输出这五种标准命令。
     */
    public enum VoiceCommand {
        AUTO,
        MANUAL,
        STOP,
        ESTOP,
        REREGISTER
    }

    public interface VoiceCommandListener {
        void onVoiceCommand(VoiceCommand command);
    }

    private final Activity activity;
    private final VoiceCommandListener listener;

    private Model model;
    private SpeechService speechService;

    private boolean modelReady = false;
    private boolean listening = false;

    private long awakeUntilTime = 0L;
    private long lastWakeToastTime = 0L;

    private VoiceCommand lastCommand = null;
    private long lastCommandTime = 0L;

    public VoskAgentManager(
            Activity activity,
            VoiceCommandListener listener) {

        this.activity = activity;
        this.listener = listener;
    }

    public void initAndStart() {
        if (modelReady) {
            startListening();
            return;
        }

        showToast("语音模型加载中...");

        StorageService.unpack(
                activity,
                "model-cn",
                "model",
                loadedModel -> {
                    model = loadedModel;
                    modelReady = true;

                    showToast("语音控制已启动");
                    startListening();
                },
                exception -> {
                    Log.e(
                            TAG,
                            "Failed to unpack model",
                            exception);

                    showToast(
                            "语音模型加载失败："
                                    + exception.getMessage());
                });
    }

    public void startListening() {
        if (!modelReady || model == null || listening) {
            return;
        }

        try {
            Recognizer recognizer =
                    new Recognizer(
                            model,
                            16000.0f);

            speechService =
                    new SpeechService(
                            recognizer,
                            16000.0f);

            speechService.startListening(this);

            listening = true;

            Log.d(
                    TAG,
                    "Vosk listening started");

        } catch (IOException exception) {
            Log.e(
                    TAG,
                    "Failed to start speech service",
                    exception);

            showToast(
                    "语音启动失败："
                            + exception.getMessage());
        }
    }

    public void stopListening() {
        listening = false;

        if (speechService != null) {
            speechService.stop();
            speechService.shutdown();
            speechService = null;
        }
    }

    public void shutdown() {
        stopListening();

        if (model != null) {
            model.close();
            model = null;
        }

        modelReady = false;
        awakeUntilTime = 0L;
        lastCommand = null;
    }

    @Override
    public void onResult(String hypothesis) {
        handleHypothesis(
                hypothesis,
                false);
    }

    @Override
    public void onFinalResult(String hypothesis) {
        handleHypothesis(
                hypothesis,
                false);

        listening = false;
    }

    @Override
    public void onPartialResult(String hypothesis) {
        /*
         * 临时结果可能只识别出句子的一部分。
         *
         * 例如“紧急停止”可能先被识别成“停止”，
         * 因此 partial 阶段只处理唤醒词，不执行命令。
         */
        handleHypothesis(
                hypothesis,
                true);
    }

    @Override
    public void onError(Exception exception) {
        Log.e(
                TAG,
                "Vosk error",
                exception);

        listening = false;

        showToast(
                "语音识别错误："
                        + exception.getMessage());
    }

    @Override
    public void onTimeout() {
        listening = false;

        Log.d(
                TAG,
                "Vosk timeout");
    }

    private void handleHypothesis(
            String hypothesis,
            boolean isPartial) {

        String text =
                extractTextFromVoskResult(
                        hypothesis);

        String normalized =
                normalizeText(text);

        if (normalized.isEmpty()) {
            return;
        }

        long now =
                System.currentTimeMillis();

        if (containsWakeWord(normalized)) {
            awakeUntilTime =
                    now + WAKE_ACTIVE_MS;

            normalized =
                    removeWakeWord(normalized);

            if (now - lastWakeToastTime > 2000L) {
                lastWakeToastTime = now;
                showToast("小车已唤醒");
            }
        }

        /*
         * partial 只负责唤醒，不负责执行。
         */
        if (isPartial) {
            return;
        }

        if (now > awakeUntilTime) {
            return;
        }

        if (normalized.isEmpty()) {
            return;
        }

        VoiceCommand command =
                parseVoiceCommand(
                        normalized);

        if (command == null) {
            Log.d(
                    TAG,
                    "Unknown voice text: "
                            + normalized);

            return;
        }

        /*
         * 防止同一个 Vosk 结果在短时间内重复触发。
         */
        if (command == lastCommand
                && now - lastCommandTime
                < COMMAND_DEBOUNCE_MS) {

            return;
        }

        lastCommand = command;
        lastCommandTime = now;

        Log.d(
                TAG,
                "Voice command: "
                        + command
                        + ", text="
                        + normalized);

        activity.runOnUiThread(
                () -> {
                    if (listener != null) {
                        listener.onVoiceCommand(
                                command);
                    }
                });
    }

    /**
     * 所有自然语言关键词只在这里维护。
     */
    private VoiceCommand parseVoiceCommand(
            String command) {

        /*
         * 紧急停止必须放在普通停止之前。
         */
        if (command.contains("紧急停止")
                || command.contains("紧急停车")
                || command.contains("急停")
                || command.contains("emergencystop")
                || command.contains("estop")) {

            return VoiceCommand.ESTOP;
        }

        if (command.contains("重新注册")
                || command.contains("重新录制")
                || command.contains("重新识别")
                || command.contains("换人")
                || command.contains("重录")) {

            return VoiceCommand.REREGISTER;
        }

        if (command.contains("人工模式")
                || command.contains("人工控制")
                || command.contains("手动模式")
                || command.contains("手动控制")
                || command.contains("切换人工")
                || command.contains("切换手动")
                || command.contains("manual")) {

            return VoiceCommand.MANUAL;
        }

        if (command.contains("自动模式")
                || command.contains("自动跟随")
                || command.contains("切换自动")
                || command.contains("跟随")
                || command.contains("开始跟")
                || command.contains("跟着我")
                || command.contains("跟我")
                || command.contains("跟上")
                || command.contains("继续走")
                || command.contains("继续跟随")
                || command.contains("出发")
                || command.contains("auto")
                || command.contains("follow")) {

            return VoiceCommand.AUTO;
        }

        if (command.contains("停止跟随")
                || command.contains("停止")
                || command.contains("停车")
                || command.contains("别动")
                || command.contains("暂停")
                || command.contains("stop")) {

            return VoiceCommand.STOP;
        }

        return null;
    }

    private String normalizeText(String text) {
        if (text == null) {
            return "";
        }

        return text
                .toLowerCase(Locale.ROOT)
                .replace(" ", "")
                .replace("　", "")
                .replace("，", "")
                .replace(",", "")
                .replace("。", "")
                .replace(".", "")
                .replace("！", "")
                .replace("!", "")
                .replace("？", "")
                .replace("?", "")
                .trim();
    }

    private boolean containsWakeWord(String text) {
        return text.contains("小车小车")
                || text.contains("小车")
                || text.contains("车车");
    }

    private String removeWakeWord(String text) {
        return text
                .replace("小车小车", "")
                .replace("小车", "")
                .replace("车车", "")
                .trim();
    }

    private String extractTextFromVoskResult(
            String hypothesis) {

        try {
            JSONObject jsonObject =
                    new JSONObject(
                            hypothesis);

            String text =
                    jsonObject
                            .optString(
                                    "text",
                                    "")
                            .trim();

            if (text.isEmpty()) {
                text =
                        jsonObject
                                .optString(
                                        "partial",
                                        "")
                                .trim();
            }

            return text;

        } catch (JSONException exception) {
            Log.w(
                    TAG,
                    "Invalid Vosk result: "
                            + hypothesis);

            return "";
        }
    }

    private void showToast(String message) {
        activity.runOnUiThread(
                () ->
                        Toast.makeText(
                                        activity,
                                        message,
                                        Toast.LENGTH_SHORT)
                                .show());
    }
}