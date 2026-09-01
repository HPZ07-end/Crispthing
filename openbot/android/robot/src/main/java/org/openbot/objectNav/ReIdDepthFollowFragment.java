package org.openbot.objectNav;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.TypedValue;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Toast;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.content.SharedPreferences;
import androidx.preference.PreferenceManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageProxy;
import androidx.navigation.Navigation;
import androidx.lifecycle.ViewModelProvider;
import androidx.appcompat.app.AlertDialog;
import android.Manifest;
import android.content.pm.PackageManager;
import android.util.Log;
import androidx.core.content.ContextCompat;
import org.openbot.agent.VoskAgentManager;
import com.google.android.material.bottomsheet.BottomSheetBehavior;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.ArrayList;
import org.jetbrains.annotations.NotNull;
import org.openbot.R;
import org.openbot.common.CameraFragment;
//import org.openbot.databinding.FragmentObjectNavBinding;
import org.openbot.databinding.FragmentReidDepthFollowBinding;
import org.openbot.env.BorderedText;
import org.openbot.env.ImageUtils;
import org.openbot.tflite.Detector;
import org.openbot.tflite.Model;
import org.openbot.tflite.Network;
import org.openbot.tracking.MultiBoxTracker;
import org.openbot.utils.CameraUtils;
import org.openbot.utils.Constants;
import org.openbot.utils.Enums;
import org.openbot.utils.MovingAverage;
import org.openbot.utils.PermissionUtils;
import org.openbot.vehicle.Control;
import timber.log.Timber;
import android.os.Looper;
import ai.onnxruntime.OrtException;

public class ReIdDepthFollowFragment extends CameraFragment {
    private FragmentReidDepthFollowBinding binding;
    private Handler handler;
    private HandlerThread handlerThread;

    private boolean computingNetwork = false;
    public static float MINIMUM_CONFIDENCE_TF_OD_API = 0.5f;

    private static final float TEXT_SIZE_DIP = 10;

    private Detector detector;

    private boolean mirrorControl;
    private Matrix frameToCropTransform;
    private Bitmap croppedBitmap;
    private int sensorOrientation;
    private Bitmap cropCopyBitmap;
    private Matrix cropToFrameTransform;

    private MultiBoxTracker tracker;

    private Model model;
    private Network.Device device = Network.Device.CPU;
    private int numThreads = -1;
    private String classType = "person";

    private long lastProcessingTimeMs = -1;
    private long frameNum = 0;

    private final boolean isBenchmarkMode = false;
    private long processedFrames = 0;
    private final int movingAvgSize = 100;
    //ReID参数：
    // 每隔多少次 person 检测执行一次 ReID，避免手机负载过高
    private static final int LIVE_REID_FRAME_INTERVAL = 6;

    // 一帧最多比较面积最大的 3 个人
    private static final int MAX_LIVE_REID_CANDIDATES = 3;

    private volatile float bestLiveSimilarity = -1.0f;
    private MovingAverage movingAvgProcessingTimeMs = new MovingAverage(movingAvgSize);

    private AlertDialog targetReviewDialog;//弹窗
    private ReIdEngine reIdEngine;
    private static final int REQUEST_RECORD_AUDIO_FOR_VOSK = 6201;

    private static final float REID_MATCH_THRESHOLD = 0.75f;

    private long targetSequence = 0;
    private long lastTargetSendTimeMs = 0;

    private long commandSequence = 0;

    private static final long TARGET_SEND_INTERVAL_MS = 200;

    // 改成电脑当前 Wi-Fi 的 IPv4 地址
    private static final String PREF_UDP_TARGET_IP =
            "udp_target_ip";

    private static final String PREF_UDP_TARGET_PORT =
            "udp_target_port";

    private static final int DEFAULT_UDP_TARGET_PORT =
            5005;

    private UdpTargetSender udpTargetSender;
    private VoskAgentManager voskAgentManager;
    private enum TargetState {
        UNREGISTERED,//未注册
        COUNTDOWN,//倒计时状态
        REGISTERING,//正在采集
        REVIEWING,//等待用户确认
        SEARCHING //已确认准备搜索目标
    }
    private static final String KEY_TARGET_STATE = "target_state";
    private volatile TargetState targetState = TargetState.UNREGISTERED;

    private final Handler registrationHandler =
            new Handler(Looper.getMainLooper());

    private static final long REGISTRATION_CAPTURE_INTERVAL_MS = 150L;
    private static final int MAX_GALLERY_SAMPLES = 24;
    private static final int MIN_GALLERY_SAMPLES = 6;

    private static final int REID_SAMPLE_WIDTH = 128;
    private static final int REID_SAMPLE_HEIGHT = 256;

    private TargetGalleryViewModel targetGalleryViewModel;

    private PoseDistanceEngine poseDistanceEngine;

    private int poseFrameCounter = 0;

    // 每隔 3 次有效目标识别运行一次 Pose，避免手机过热
    private static final int POSE_FRAME_INTERVAL = 1;

    /*
     * 注册时的跟随距离标定参数。
     *
     * registrationTorsoScales：
     * 保存用户站在喜欢的跟随位置时，
     * 多帧肩膀—髋部的 fullScale。
     *
     * baselineTorsoScale：
     * 注册完成后取多帧中位数，
     * 作为用户喜欢的跟随尺度。
     */
    private static final long REGISTRATION_POSE_INTERVAL_MS = 250L;

    private static final int MIN_REGISTRATION_POSE_SAMPLES = 6;

    private static final int MAX_REGISTRATION_POSE_SAMPLES = 20;

    private final List<Float> registrationTorsoScales =
            new ArrayList<>();

    private volatile long lastRegistrationPoseCaptureMs = 0L;

    private volatile float baselineTorsoScale = -1.0f;

    /*
     * 自动跟随时的肩髋尺度滤波。
     */
    private static final int RUNTIME_POSE_WINDOW_SIZE = 5;

    // 单帧相对已有稳定值变化超过 25%，暂时认为是异常值???

    // Pose 暂时识别失败时，保留上一次结果 600ms
    private static final long RUNTIME_POSE_GRACE_MS = 2000L;

    private final List<Float> runtimeTorsoScales =
            new ArrayList<>();

    private volatile float latestRelativeDistance = -1.0f;

    private volatile long lastValidRuntimePoseMs = 0L;
    private volatile long lastRegistrationCaptureMs = 0L;

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        targetGalleryViewModel =
                new ViewModelProvider(
                        requireActivity())
                        .get(TargetGalleryViewModel.class);

        baselineTorsoScale =
                targetGalleryViewModel
                        .getBaselineTorsoScale();
    }

    @Override
    public View onCreateView(
            @NotNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        binding = FragmentReidDepthFollowBinding.inflate(inflater, container, false);

        return inflateFragment(binding, inflater, container);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        baselineTorsoScale =
                targetGalleryViewModel
                        .getBaselineTorsoScale();

        Log.d(
                "POSE_BASELINE",
                String.format(
                        Locale.US,
                        "restored baseline=%.4f complete=%s",
                        baselineTorsoScale,
                        targetGalleryViewModel
                                .hasCompleteRegistration()));
        voskAgentManager =
                new VoskAgentManager(
                        requireActivity(),
                        this::handleVoiceCommand);

        try {
            poseDistanceEngine =
                    new PoseDistanceEngine(
                            requireContext()
                                    .getApplicationContext());

            Log.d(
                    "POSE_TEST",
                    "PoseDistanceEngine initialized");

        } catch (RuntimeException exception) {
            Log.e(
                    "POSE_TEST",
                    "PoseDistanceEngine initialization failed",
                    exception);

            poseDistanceEngine = null;
        }

        startVoskAgentIfPossible();

        TargetState restoredState =
                TargetState.UNREGISTERED;

        if (savedInstanceState != null) {

            String savedStateName =
                    savedInstanceState.getString(
                            KEY_TARGET_STATE);

            if (savedStateName != null) {
                try {
                    restoredState =
                            TargetState.valueOf(
                                    savedStateName);

                } catch (IllegalArgumentException ignored) {
                    restoredState =
                            TargetState.UNREGISTERED;
                }
            }
        }

        /*
         * 注册倒计时或采集过程中发生页面重建，
         * 本次注册数据可能不完整，因此要求重新注册。
         */
        if (restoredState == TargetState.COUNTDOWN
                || restoredState == TargetState.REGISTERING) {

            targetGalleryViewModel.clearSamples();

            clearRegistrationPoseSamples();

            resetRuntimeDistance();

            baselineTorsoScale = -1.0f;

            restoredState =
                    TargetState.UNREGISTERED;
        }

        /*
         * ViewModel 中已经有完整注册结果时，
         * 无论 savedInstanceState 是否存在，
         * 都恢复成 SEARCHING。
         */
        if (targetGalleryViewModel
                .hasCompleteRegistration()) {

            baselineTorsoScale =
                    targetGalleryViewModel
                            .getBaselineTorsoScale();

            restoredState =
                    TargetState.SEARCHING;
        }

        updateTargetUi(restoredState);

        if (restoredState == TargetState.REVIEWING
                && targetGalleryViewModel.getSampleCount()
                >= MIN_GALLERY_SAMPLES) {

            binding.getRoot().post(
                    this::showTargetReviewDialog);
        }

        binding.registerTargetButton.setOnClickListener(
                v -> startTargetRegistration());

        binding.confidenceValue.setText((int) (MINIMUM_CONFIDENCE_TF_OD_API * 100) + "%");

        binding.plusConfidence.setOnClickListener(
                v -> {
                    String trimConfValue = binding.confidenceValue.getText().toString().trim();
                    int confValue = Integer.parseInt(trimConfValue.substring(0, trimConfValue.length() - 1));
                    if (confValue >= 95) return;
                    confValue += 5;
                    binding.confidenceValue.setText(confValue + "%");
                    MINIMUM_CONFIDENCE_TF_OD_API = confValue / 100f;
                });
        binding.minusConfidence.setOnClickListener(
                v -> {
                    String trimConfValue = binding.confidenceValue.getText().toString().trim();
                    int confValue = Integer.parseInt(trimConfValue.substring(0, trimConfValue.length() - 1));
                    if (confValue <= 5) return;
                    confValue -= 5;
                    binding.confidenceValue.setText(confValue + "%");
                    MINIMUM_CONFIDENCE_TF_OD_API = confValue / 100f;
                });

        binding.controllerContainer.speedInfo.setText(getString(R.string.speedInfo, "---,---"));

        if (vehicle.getConnectionType().equals("USB")) {
            binding.usbToggle.setVisibility(View.VISIBLE);
            binding.bleToggle.setVisibility(View.GONE);
        } else if (vehicle.getConnectionType().equals("Bluetooth")) {
            binding.bleToggle.setVisibility(View.VISIBLE);
            binding.usbToggle.setVisibility(View.GONE);
        }

        classType = preferencesManager.getObjectType();
        binding.classType.setOnItemSelectedListener(
                new AdapterView.OnItemSelectedListener() {
                    @Override
                    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                        classType = parent.getItemAtPosition(position).toString();
                        preferencesManager.setObjectType(classType);
                    }

                    @Override
                    public void onNothingSelected(AdapterView<?> parent) {}
                });
        binding.deviceSpinner.setSelection(preferencesManager.getDevice());
        setNumThreads(preferencesManager.getNumThreads());
        binding.threads.setText(String.valueOf(getNumThreads()));

        binding.cameraToggle.setOnClickListener(v -> toggleCamera());

        binding.mirrorControl.setOnClickListener(v -> mirrorControl());

        List<String> models =
                getModelNames(f -> f.type.equals(Model.TYPE.DETECTOR) && f.pathType != Model.PATH_TYPE.URL);
        initModelSpinner(binding.modelSpinner, models, preferencesManager.getObjectNavModel());

        setAnalyserResolution(Enums.Preview.HD.getValue());
        binding.deviceSpinner.setOnItemSelectedListener(
                new AdapterView.OnItemSelectedListener() {
                    @Override
                    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                        String selected = parent.getItemAtPosition(position).toString();
                        setDevice(Network.Device.valueOf(selected.toUpperCase()));
                    }

                    @Override
                    public void onNothingSelected(AdapterView<?> parent) {}
                });

        binding.plus.setOnClickListener(
                v -> {
                    String threads = binding.threads.getText().toString().trim();
                    int numThreads = Integer.parseInt(threads);
                    if (numThreads >= 9) return;
                    setNumThreads(++numThreads);
                    binding.threads.setText(String.valueOf(numThreads));
                });
        binding.minus.setOnClickListener(
                v -> {
                    String threads = binding.threads.getText().toString().trim();
                    int numThreads = Integer.parseInt(threads);
                    if (numThreads == 1) return;
                    setNumThreads(--numThreads);
                    binding.threads.setText(String.valueOf(numThreads));
                });
        BottomSheetBehavior.from(binding.aiBottomSheet).setState(BottomSheetBehavior.STATE_EXPANDED);

        mViewModel
                .getUsbStatus()
                .observe(getViewLifecycleOwner(), status -> binding.usbToggle.setChecked(status));

        binding.usbToggle.setChecked(vehicle.isUsbConnected());
        binding.bleToggle.setChecked(vehicle.bleConnected());

        binding.usbToggle.setOnClickListener(
                v -> {
                    binding.usbToggle.setChecked(vehicle.isUsbConnected());
                    Navigation.findNavController(requireView()).navigate(R.id.open_usb_fragment);
                });
        binding.bleToggle.setOnClickListener(
                v -> {
                    binding.bleToggle.setChecked(vehicle.bleConnected());
                    Navigation.findNavController(requireView()).navigate(R.id.open_bluetooth_fragment);
                });

        setSpeedMode(Enums.SpeedMode.getByID(preferencesManager.getSpeedMode()));
        setControlMode(Enums.ControlMode.getByID(preferencesManager.getControlMode()));
        setDriveMode(Enums.DriveMode.getByID(preferencesManager.getDriveMode()));

        binding.controllerContainer.controlMode.setOnClickListener(
                v -> {
                    Enums.ControlMode controlMode =
                            Enums.ControlMode.getByID(preferencesManager.getControlMode());
                    if (controlMode != null) setControlMode(Enums.switchControlMode(controlMode));
                });
        binding.controllerContainer.driveMode.setOnClickListener(
                v -> setDriveMode(Enums.switchDriveMode(vehicle.getDriveMode())));

        binding.controllerContainer.speedMode.setOnClickListener(
                v ->
                        setSpeedMode(
                                Enums.toggleSpeed(
                                        Enums.Direction.CYCLIC.getValue(),
                                        Enums.SpeedMode.getByID(preferencesManager.getSpeedMode()))));

        binding.autoSwitch.setOnClickListener(
                v -> {

                    boolean enableAuto =
                            binding.autoSwitch
                                    .isChecked();

                    if (enableAuto) {

                        /*
                         * 没有完成身份和距离注册，
                         * 不允许进入自动模式。
                         */
                        if (!targetGalleryViewModel
                                .hasCompleteRegistration()) {

                            binding.autoSwitch
                                    .setChecked(false);

                            Toast.makeText(
                                            requireContext(),
                                            "请先完成身份和跟随距离注册",
                                            Toast.LENGTH_SHORT)
                                    .show();

                            return;
                        }

                        baselineTorsoScale =
                                targetGalleryViewModel
                                        .getBaselineTorsoScale();

                        resetRuntimeDistance();

                        sendCommandPacket(
                                "AUTO");

                        setNetworkEnabled(
                                true);

                    } else {

                        sendCommandPacket(
                                "MANUAL");

                        resetRuntimeDistance();

                        setNetworkEnabled(
                                false);

                        vehicle.setControl(
                                0,
                                0);
                    }
                });
        binding.dynamicSpeed.setChecked(preferencesManager.getDynamicSpeed());
        binding.dynamicSpeed.setOnClickListener(
                v -> {
                    preferencesManager.setDynamicSpeed(binding.dynamicSpeed.isChecked());
                    tracker.setDynamicSpeed(preferencesManager.getDynamicSpeed());
                });
    }

    private void startVoskAgentIfPossible() {
        if (!isAdded() || voskAgentManager == null) {
            return;
        }

        if (ContextCompat.checkSelfPermission(
                requireContext(),
                Manifest.permission.RECORD_AUDIO)
                == PackageManager.PERMISSION_GRANTED) {

            voskAgentManager.initAndStart();

        } else {
            requestPermissions(
                    new String[] {
                            Manifest.permission.RECORD_AUDIO
                    },
                    REQUEST_RECORD_AUDIO_FOR_VOSK);
        }
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode,
            @NonNull String[] permissions,
            @NonNull int[] grantResults) {

        super.onRequestPermissionsResult(
                requestCode,
                permissions,
                grantResults);

        if (requestCode != REQUEST_RECORD_AUDIO_FOR_VOSK) {
            return;
        }

        if (grantResults.length > 0
                && grantResults[0]
                == PackageManager.PERMISSION_GRANTED) {

            if (voskAgentManager != null) {
                voskAgentManager.initAndStart();
            }

        } else {
            Toast.makeText(
                            requireContext(),
                            "未授予麦克风权限，语音控制不可用",
                            Toast.LENGTH_LONG)
                    .show();
        }
    }

    private void handleVoiceCommand(
            VoskAgentManager.VoiceCommand command) {

        if (command == null || binding == null) {
            return;
        }

        switch (command) {

            case ESTOP:
                /*
                 * 急停优先级最高。
                 * Arduino 收到后应进入锁定状态，
                 * 不能因为后续 TARGET 数据自动恢复。
                 */
                sendCommandPacket("ESTOP");

                setNetworkEnabled(false);
                vehicle.setControl(0, 0);

                Toast.makeText(
                                requireContext(),
                                "已发送：紧急停止",
                                Toast.LENGTH_SHORT)
                        .show();
                break;

            case STOP:
                sendCommandPacket("STOP");

                setNetworkEnabled(false);
                vehicle.setControl(0, 0);

                Toast.makeText(
                                requireContext(),
                                "已发送：停止跟随",
                                Toast.LENGTH_SHORT)
                        .show();
                break;

            case MANUAL:
                sendCommandPacket("MANUAL");

                setNetworkEnabled(false);
                vehicle.setControl(0, 0);

                Toast.makeText(
                                requireContext(),
                                "已切换：人工模式",
                                Toast.LENGTH_SHORT)
                        .show();
                break;

            case AUTO:
                if (!targetGalleryViewModel
                        .hasCompleteRegistration()) {

                    Toast.makeText(
                                    requireContext(),
                                    "请先完成身份和跟随距离注册",
                                    Toast.LENGTH_SHORT)
                            .show();

                    return;
                }

                baselineTorsoScale =
                        targetGalleryViewModel
                                .getBaselineTorsoScale();

                resetRuntimeDistance();

                sendCommandPacket("AUTO");

                setNetworkEnabled(true);

                Toast.makeText(
                                requireContext(),
                                "已切换：自动跟随模式",
                                Toast.LENGTH_SHORT)
                        .show();
                break;

            case REREGISTER:
                /*
                 * 重新注册期间手机仍需运行 person 检测，
                 * 但板子必须先停车。
                 */
                sendCommandPacket("STOP");
                vehicle.setControl(0, 0);

                startTargetRegistration();

                Toast.makeText(
                                requireContext(),
                                "已停车，开始重新注册人物",
                                Toast.LENGTH_SHORT)
                        .show();
                break;
        }
    }

    private String buildCommandPacket(String command) {
        return String.format(
                Locale.US,
                "CMD,%d,%s\n",
                commandSequence++,
                command);
    }
    private String buildTargetPacket(
            boolean valid,
            float xError,
            float relativeDistance,
            float similarity
    ) {
        long sequence = targetSequence++;

        if (!valid) {
            return String.format(
                    Locale.US,
                    "TARGET,%d,0,0.00,-1.0,0.00\n",
                    sequence
            );
        }

        xError = Math.max(-1.0f, Math.min(1.0f, xError));
        similarity = Math.max(0.0f, Math.min(1.0f, similarity));

        return String.format(
                Locale.US,
                "TARGET,%d,1,%.3f,%.3f,%.3f\n",
                sequence,
                xError,
                relativeDistance,
                similarity
        );
    }

    private synchronized void reloadUdpTargetSender() {

        if (!isAdded()) {
            return;
        }

        SharedPreferences preferences =
                PreferenceManager
                        .getDefaultSharedPreferences(
                                requireContext());

        String targetIp =
                preferences.getString(
                        PREF_UDP_TARGET_IP,
                        "");

        String targetPortText =
                preferences.getString(
                        PREF_UDP_TARGET_PORT,
                        String.valueOf(
                                DEFAULT_UDP_TARGET_PORT));

        if (targetIp == null) {
            targetIp = "";
        }

        targetIp =
                targetIp.trim();

        int targetPort;

        try {
            targetPort =
                    Integer.parseInt(
                            targetPortText == null
                                    ? ""
                                    : targetPortText.trim());

        } catch (NumberFormatException exception) {

            targetPort =
                    -1;
        }

        /*
         * 先关闭旧发送器。
         */
        if (udpTargetSender != null) {
            udpTargetSender.close();
            udpTargetSender = null;
        }

        /*
         * 没填写或端口不合法时，
         * 暂时不启用 UDP，但 USB 发送仍然可用。
         */
        if (targetIp.isEmpty()
                || targetPort <= 0
                || targetPort > 65535) {

            Log.w(
                    "UDP_CONFIG",
                    "UDP 设置无效，ip="
                            + targetIp
                            + " port="
                            + targetPortText);

            return;
        }

        udpTargetSender =
                new UdpTargetSender(
                        targetIp,
                        targetPort);

        Log.d(
                "UDP_CONFIG",
                "UDP 接收端已加载："
                        + targetIp
                        + ":"
                        + targetPort);
    }
    private void sendPacketToBoard(String packet) {
        if (packet == null || packet.isEmpty()) {
            return;
        }

        Log.d(
                "BOARD_TX",
                packet.trim());

        // UDP：当前用于电脑测试，后续也可发给 ESP32
        if (udpTargetSender != null) {
            udpTargetSender.send(packet);
        }

        // USB：连接 USB-TTL 后发送给 Arduino
        if (vehicle.isUsbConnected()) {
            vehicle.sendStringToDevice(packet);

            Log.d(
                    "TARGET_USB",
                    "USB sent: " + packet.trim());
        }
    }

    private void sendCommandPacket(String command) {
        String packet =
                buildCommandPacket(command);

        sendPacketToBoard(packet);
    }

    private void testPoseDistance(
            Bitmap fullFrameBitmap,
            RectF targetBox) {

        if (poseDistanceEngine == null
                || fullFrameBitmap == null
                || fullFrameBitmap.isRecycled()
                || targetBox == null) {

            return;
        }

        poseFrameCounter++;

        if (poseFrameCounter % POSE_FRAME_INTERVAL != 0) {
            return;
        }

        int frameWidth =
                fullFrameBitmap.getWidth();

        int frameHeight =
                fullFrameBitmap.getHeight();

        // 扩大人物框，防止肩膀或髋部被裁掉
        float paddingX =
                targetBox.width() * 0.15f;

        float paddingY =
                targetBox.height() * 0.10f;

        int cropLeft =
                Math.max(
                        0,
                        (int) Math.floor(
                                targetBox.left - paddingX));

        int cropTop =
                Math.max(
                        0,
                        (int) Math.floor(
                                targetBox.top - paddingY));

        int cropRight =
                Math.min(
                        frameWidth,
                        (int) Math.ceil(
                                targetBox.right + paddingX));

        int cropBottom =
                Math.min(
                        frameHeight,
                        (int) Math.ceil(
                                targetBox.bottom + paddingY));

        int cropWidth =
                cropRight - cropLeft;

        int cropHeight =
                cropBottom - cropTop;

        if (cropWidth < 32 || cropHeight < 64) {
            Log.d(
                    "POSE_TEST",
                    "valid=false reason=crop_too_small");
            markRuntimePoseUnavailable(
                    "crop_too_small");
            return;
        }

        Bitmap personCrop = null;
        Bitmap uprightCrop = null;

        try {
            // 按检测框从原始画面裁剪
            personCrop =
                    Bitmap.createBitmap(
                            fullFrameBitmap,
                            cropLeft,
                            cropTop,
                            cropWidth,
                            cropHeight);

            /*
             * 和 ReID 一样，把人物旋转到头朝上的方向。
             * MediaPipe Pose 不能稳定识别横着的人。
             */
            uprightCrop =
                    rotateBitmapToUpright(
                            personCrop);

            PoseDistanceEngine.Measurement measurement =
                    poseDistanceEngine.measure(
                            uprightCrop);

            if (!measurement.isValid()) {
                markRuntimePoseUnavailable(
                        measurement.getInvalidReason());
                Log.d(
                        "POSE_TEST",
                        "valid=false reason="
                                + measurement.getInvalidReason()
                                + " rawCrop="
                                + cropWidth
                                + "x"
                                + cropHeight
                                + " uprightCrop="
                                + uprightCrop.getWidth()
                                + "x"
                                + uprightCrop.getHeight());

                return;
            }

            /*
             * 确定整张画面旋转到正方向后的高度。
             *
             * 旋转 90° 或 270°：
             * 正向画面的高度 = 原画面宽度。
             */
            int normalizedRotation =
                    ((sensorOrientation % 360) + 360) % 360;

            int uprightFrameHeight;

            if (normalizedRotation == 90
                    || normalizedRotation == 270) {

                uprightFrameHeight =
                        frameWidth;

            } else {
                uprightFrameHeight =
                        frameHeight;
            }

            /*
             * measurement.torsoScale 是相对于 uprightCrop 高度的比例。
             *
             * 乘以 uprightCrop 高度：
             * 得到肩膀到髋部的像素高度。
             *
             * 再除以正向完整画面的高度：
             * 得到可以比较远近的 fullScale。
             */
            float fullFrameTorsoScale =
                    measurement.getTorsoScale()
                            * uprightCrop.getHeight()
                            / (float) uprightFrameHeight;

            float relativeDistance =
                    updateRuntimeDistance(
                            fullFrameTorsoScale);

            Log.d(
                    "POSE_TEST",
                    String.format(
                            Locale.US,
                            "valid=true "
                                    + "fullScale=%.4f "
                                    + "cropScale=%.4f "
                                    + "left=%.4f "
                                    + "right=%.4f "
                                    + "center=%.4f "
                                    + "visibility=%.3f "
                                    + "rotation=%d "
                                    + "rawCrop=%dx%d "
                                    + "uprightCrop=%dx%d "
                                    + "frame=%dx%d "
                                    + "relative=%.3f",
                            fullFrameTorsoScale,
                            measurement.getTorsoScale(),
                            measurement.getLeftTorsoHeight(),
                            measurement.getRightTorsoHeight(),
                            measurement.getCenterTorsoHeight(),
                            measurement.getMinimumVisibility(),
                            normalizedRotation,
                            cropWidth,
                            cropHeight,
                            uprightCrop.getWidth(),
                            uprightCrop.getHeight(),
                            frameWidth,
                            frameHeight,
                            relativeDistance));

        } catch (RuntimeException exception) {
            markRuntimePoseUnavailable(
                    "pose_exception");
            Log.e(
                    "POSE_TEST",
                    "Pose distance test failed",
                    exception);

        } finally {
            /*
             * rotateBitmapToUpright 在不需要旋转时，
             * uprightCrop 和 personCrop 可能是同一个对象。
             */
            if (uprightCrop != null
                    && uprightCrop != personCrop
                    && !uprightCrop.isRecycled()) {

                uprightCrop.recycle();
            }

            if (personCrop != null
                    && !personCrop.isRecycled()) {

                personCrop.recycle();
            }
        }
    }

    private void outputTargetPacket(
            boolean valid,
            float xError,
            float relativeDistance,
            float similarity) {

        long now =
                SystemClock.elapsedRealtime();

        if (now - lastTargetSendTimeMs
                < TARGET_SEND_INTERVAL_MS) {

            return;
        }

        lastTargetSendTimeMs = now;

        String packet =
                buildTargetPacket(
                        valid,
                        xError,
                        relativeDistance,
                        similarity);

        Log.d(
                "TARGET_TX",
                packet.trim());

        sendPacketToBoard(
                packet);
    }

    private void outputTargetPacket(
            boolean valid,
            float xError,
            float similarity) {

        outputTargetPacket(
                valid,
                xError,
                -1.0f,
                similarity);
    }

    private void mirrorControl() {
        mirrorControl = !mirrorControl;
    }

    private void clearRegistrationPoseSamples() {
        synchronized (registrationTorsoScales) {
            registrationTorsoScales.clear();
        }
    }

    private int getRegistrationPoseSampleCount() {
        synchronized (registrationTorsoScales) {
            return registrationTorsoScales.size();
        }
    }
    private void startTargetRegistration() {
        registrationHandler.removeCallbacksAndMessages(null);

        /*
         * 开始注册前，先通知板子停车。
         * 防止板子仍然停留在上一次 AUTO 状态。
         */
        sendCommandPacket(
                "STOP");

        vehicle.setControl(
                0,
                0);
        /*
         * 清空上一次注册的人物图片和距离偏好。
         */
        targetGalleryViewModel.beginNewRegistration();

        clearRegistrationPoseSamples();

        resetRuntimeDistance();

        baselineTorsoScale = -1.0f;

        lastRegistrationCaptureMs = 0L;

        lastRegistrationPoseCaptureMs = 0L;

        setNetworkEnabled(true);


        updateTargetUi(TargetState.COUNTDOWN);

        startRegistrationCountdown(3);
    }

    private void startRegistrationCountdown(int number) {
        if (!isAdded() || binding == null) {
            return;
        }

        binding.registrationCountdownText.setVisibility(View.VISIBLE);

        if (number > 0) {
            binding.registrationCountdownText.setText(
                    String.valueOf(number));

            binding.targetStatusText.setText("请站在希望小车保持的位置，" + number + " 秒后开始注册");

            registrationHandler.postDelayed(
                    () -> startRegistrationCountdown(number - 1),
                    1000);

            return;
        }

        binding.registrationCountdownText.setText("开始");
        binding.targetStatusText.setText("状态：开始录制，请保持当前位置并缓慢转身");

        registrationHandler.postDelayed(
                () -> {
                    if (!isAdded() || binding == null) {
                        return;
                    }

                    binding.registrationCountdownText.setVisibility(View.GONE);

                    updateTargetUi(TargetState.REGISTERING);

                    startRegistrationCaptureTimer();
                },
                600);
    }
    private void startRegistrationCaptureTimer() {
        lastRegistrationCaptureMs = 0L;

        registrationHandler.postDelayed(
                () -> {
                    if (!isAdded() || binding == null) {
                        return;
                    }

                    int sampleCount =
                            targetGalleryViewModel
                                    .getSampleCount();

                    int poseSampleCount =
                            getRegistrationPoseSampleCount();

                    if (sampleCount
                            >= MIN_GALLERY_SAMPLES
                            && poseSampleCount
                            >= MIN_REGISTRATION_POSE_SAMPLES) {

                        updateTargetUi(
                                TargetState.REVIEWING);

                        showTargetReviewDialog();

                    } else {

                        targetGalleryViewModel
                                .clearSamples();

                        clearRegistrationPoseSamples();

                        baselineTorsoScale =
                                -1.0f;

                        updateTargetUi(
                                TargetState.UNREGISTERED);

                        binding.targetStatusText.setText(
                                "注册失败：身份图片 "
                                        + sampleCount
                                        + "/"
                                        + MIN_GALLERY_SAMPLES
                                        + "，距离样本 "
                                        + poseSampleCount
                                        + "/"
                                        + MIN_REGISTRATION_POSE_SAMPLES);
                    }
                },
                4000);
    }
    private void updateTargetUi(TargetState state) {
        targetState = state;

        if (binding == null) {
            return;
        }

        switch (state) {
            case UNREGISTERED:
                binding.targetStatusText.setText("状态：未注册目标");
                binding.registerTargetButton.setText("重置/注册目标");
                binding.registerTargetButton.setEnabled(true);
                break;

            case COUNTDOWN:
                binding.targetStatusText.setText(
                        "请站在希望小车保持的位置！");

                binding.registerTargetButton.setText("准备中...");
                binding.registerTargetButton.setEnabled(false);
                break;

            case REGISTERING:
                binding.targetStatusText.setText(
                        "正在注册身份和跟随距离，请在原地缓慢转身");

                binding.registerTargetButton.setText("录制中...");
                binding.registerTargetButton.setEnabled(false);
                break;

            case REVIEWING:
                binding.targetStatusText.setText(
                        "状态：采集完成，等待确认人物");

                binding.registerTargetButton.setText("等待确认");
                binding.registerTargetButton.setEnabled(false);
                break;


            case SEARCHING:
                String personId =
                        targetGalleryViewModel.getCurrentPersonId();

                if (targetGalleryViewModel
                        .hasCompleteRegistration()) {

                    baselineTorsoScale =
                            targetGalleryViewModel
                                    .getBaselineTorsoScale();

                    binding.targetStatusText.setText(
                            String.format(
                                    Locale.US,
                                    "人物 %s 已注册｜ReID：512维｜跟随基准：%.4f",
                                    personId,
                                    baselineTorsoScale));

                } else {
                    binding.targetStatusText.setText(
                            "状态：身份或跟随距离注册不完整");
                }

                binding.registerTargetButton.setText("重新注册目标");
                binding.registerTargetButton.setEnabled(true);
                break;
        }
    }
    //选择最靠近画面中间的人
    private Detector.Recognition findMostCenteredPerson(
            List<Detector.Recognition> recognitions,
            int frameWidth,
            int frameHeight) {

        Detector.Recognition bestPerson = null;
        float bestScore = Float.MAX_VALUE;

        float frameCenterX = frameWidth / 2f;
        float frameCenterY = frameHeight / 2f;

        for (Detector.Recognition recognition : recognitions) {
            RectF location = recognition.getLocation();

            if (location == null) {
                continue;
            }

            // 人物框太小，说明距离过远，不采集
            if (location.width() < frameWidth * 0.08f
                    || location.height() < frameHeight * 0.20f) {
                continue;
            }

            float normalizedX =
                    (location.centerX() - frameCenterX) / frameWidth;

            float normalizedY =
                    (location.centerY() - frameCenterY) / frameHeight;

            // 人物不能太靠近画面边缘
            if (Math.abs(normalizedX) > 0.35f) {
                continue;
            }

            float score =
                    normalizedX * normalizedX
                            + 0.25f * normalizedY * normalizedY;

            if (score < bestScore) {
                bestScore = score;
                bestPerson = recognition;
            }
        }

        return bestPerson;
    }

    private Bitmap rotateBitmapToUpright(Bitmap source) {
        if (source == null) {
            return null;
        }

        int rotation = ((sensorOrientation % 360) + 360) % 360;

        if (rotation == 0) {
            return source;
        }

        Matrix matrix = new Matrix();
        matrix.postRotate(rotation);

        return Bitmap.createBitmap(
                source,
                0,
                0,
                source.getWidth(),
                source.getHeight(),
                matrix,
                true);
    }
    //裁剪并保存人物图片

    private Bitmap createReIdPersonSample(
            Bitmap frameBitmap,
            RectF personLocation) {

        if (frameBitmap == null
                || frameBitmap.isRecycled()
                || personLocation == null) {
            return null;
        }

        int left =
                Math.max(
                        0,
                        (int) Math.floor(personLocation.left));

        int top =
                Math.max(
                        0,
                        (int) Math.floor(personLocation.top));

        int right =
                Math.min(
                        frameBitmap.getWidth(),
                        (int) Math.ceil(personLocation.right));

        int bottom =
                Math.min(
                        frameBitmap.getHeight(),
                        (int) Math.ceil(personLocation.bottom));

        int width = right - left;
        int height = bottom - top;

        if (width < 32 || height < 64) {
            return null;
        }

        try {
            // 人物框坐标对应原始摄像头画面
            Bitmap personCrop =
                    Bitmap.createBitmap(
                            frameBitmap,
                            left,
                            top,
                            width,
                            height);

            // 转成头朝上的方向
            Bitmap uprightCrop =
                    rotateBitmapToUpright(personCrop);

            if (uprightCrop != personCrop
                    && !personCrop.isRecycled()) {
                personCrop.recycle();
            }

            // ReID 标准输入比例：宽 128，高 256
            Bitmap scaledSample =
                    Bitmap.createScaledBitmap(
                            uprightCrop,
                            128,
                            256,
                            true);

            if (scaledSample != uprightCrop
                    && !uprightCrop.isRecycled()) {
                uprightCrop.recycle();
            }

            return scaledSample;

        } catch (IllegalArgumentException exception) {
            Timber.e(
                    exception,
                    "Failed to create live ReID person crop");

            return null;
        }
    }

    private float medianOf(
            List<Float> values) {

        if (values == null || values.isEmpty()) {
            return -1.0f;
        }

        List<Float> sorted =
                new ArrayList<>(
                        values);

        sorted.sort(
                (first, second) ->
                        Float.compare(
                                first,
                                second));

        int size =
                sorted.size();

        int middle =
                size / 2;

        if (size % 2 == 1) {
            return sorted.get(
                    middle);
        }

        return (
                sorted.get(middle - 1)
                        + sorted.get(middle))
                / 2.0f;
    }

    /**
     * 根据注册过程中采集的 fullScale，
     * 剔除明显异常值并计算用户喜欢的跟随尺度。
     */
    private float calculateRegistrationBaseline() {

        List<Float> rawSamples;

        synchronized (registrationTorsoScales) {
            rawSamples =
                    new ArrayList<>(
                            registrationTorsoScales);
        }

        if (rawSamples.size()
                < MIN_REGISTRATION_POSE_SAMPLES) {

            Log.d(
                    "POSE_BASELINE",
                    "计算失败：原始样本不足，count="
                            + rawSamples.size());

            return -1.0f;
        }

        /*
         * 第一步：计算全部样本的中位数。
         *
         * 中位数不容易受到 0.0832 这种异常值影响。
         */
        float rawMedian =
                medianOf(
                        rawSamples);

        if (!Float.isFinite(rawMedian)
                || rawMedian <= 0.0f) {

            return -1.0f;
        }

        /*
         * 第二步：计算每个数据距离中位数有多远。
         */
        List<Float> deviations =
                new ArrayList<>();

        for (float sample : rawSamples) {
            deviations.add(
                    Math.abs(
                            sample - rawMedian));
        }

        /*
         * MAD：
         * Median Absolute Deviation
         * 中位数绝对偏差。
         */
        float mad =
                medianOf(
                        deviations);

        /*
         * 允许误差：
         * 至少 0.015；
         * 通常使用 3 倍 MAD；
         * 最大不超过当前中位数的 25%。
         */
        float relativeLimit =
                Math.max(
                        0.015f,
                        rawMedian * 0.25f);

        float allowedDeviation =
                Math.min(
                        Math.max(
                                0.015f,
                                mad * 3.0f),
                        relativeLimit);

        List<Float> filteredSamples =
                new ArrayList<>();

        for (float sample : rawSamples) {

            float difference =
                    Math.abs(
                            sample - rawMedian);

            if (difference
                    <= allowedDeviation) {

                filteredSamples.add(
                        sample);

            } else {
                Log.d(
                        "POSE_BASELINE",
                        String.format(
                                Locale.US,
                                "剔除异常值：%.4f "
                                        + "median=%.4f "
                                        + "difference=%.4f "
                                        + "limit=%.4f",
                                sample,
                                rawMedian,
                                difference,
                                allowedDeviation));
            }
        }

        if (filteredSamples.size()
                < MIN_REGISTRATION_POSE_SAMPLES) {

            Log.d(
                    "POSE_BASELINE",
                    "计算失败：过滤后样本不足，count="
                            + filteredSamples.size());

            return -1.0f;
        }

        float calculatedBaseline =
                medianOf(
                        filteredSamples);

        Log.d(
                "POSE_BASELINE",
                String.format(
                        Locale.US,
                        "计算成功 raw=%d filtered=%d "
                                + "rawMedian=%.4f "
                                + "mad=%.4f "
                                + "limit=%.4f "
                                + "baseline=%.4f",
                        rawSamples.size(),
                        filteredSamples.size(),
                        rawMedian,
                        mad,
                        allowedDeviation,
                        calculatedBaseline));

        return calculatedBaseline;
    }

    private void resetRuntimeDistance() {
        synchronized (runtimeTorsoScales) {
            runtimeTorsoScales.clear();
        }

        latestRelativeDistance = -1.0f;
        lastValidRuntimePoseMs = 0L;
    }

    /**
     * 输入当前肩髋尺度，经过异常值剔除和五帧中位数滤波，
     * 得到相对于注册位置的距离比例。
     */
    private float updateRuntimeDistance(
            float currentFullScale) {

        if (!Float.isFinite(baselineTorsoScale)
                || baselineTorsoScale <= 0.0f) {

            latestRelativeDistance = -1.0f;

            Log.d(
                    "POSE_DISTANCE",
                    "invalid reason=baseline_missing");

            return -1.0f;
        }

        if (!Float.isFinite(currentFullScale)
                || currentFullScale < 0.03f
                || currentFullScale > 0.60f) {

            markRuntimePoseUnavailable(
                    "scale_out_of_range");

            return getLatestRelativeDistance();
        }

        synchronized (runtimeTorsoScales) {

            /*
             * 已经积累至少 3 个稳定值后，
             * 检查新数值是否突然跳变。
             */
            runtimeTorsoScales.add(
                    currentFullScale);

            while (runtimeTorsoScales.size()
                    > RUNTIME_POSE_WINDOW_SIZE) {

                runtimeTorsoScales.remove(0);
            }

            float filteredScale =
                    medianOf(
                            runtimeTorsoScales);


            /*
             * 注册位置：
             * baseline ≈ current
             * relativeDistance ≈ 1
             *
             * 人走远：
             * current 变小
             * relativeDistance > 1
             *
             * 人靠近：
             * current 变大
             * relativeDistance < 1
             */
            float relativeDistance =
                    baselineTorsoScale
                            / filteredScale;

            relativeDistance =
                    Math.max(
                            0.40f,
                            Math.min(
                                    2.50f,
                                    relativeDistance));

            latestRelativeDistance =
                    relativeDistance;

            lastValidRuntimePoseMs =
                    SystemClock.elapsedRealtime();

            Log.d(
                    "POSE_DISTANCE",
                    String.format(
                            Locale.US,
                            "baseline=%.4f "
                                    + "current=%.4f "
                                    + "filtered=%.4f "
                                    + "relative=%.3f "
                                    + "window=%d",
                            baselineTorsoScale,
                            currentFullScale,
                            filteredScale,
                            relativeDistance,
                            runtimeTorsoScales.size()));

            return relativeDistance;
        }
    }

    private void markRuntimePoseUnavailable(
            String reason) {

        long now =
                SystemClock.elapsedRealtime();

        /*
         * 短暂识别失败时继续使用上一次稳定结果。
         * 超过宽限时间后，距离变为无效。
         */
        if (lastValidRuntimePoseMs <= 0L
                || now - lastValidRuntimePoseMs
                > RUNTIME_POSE_GRACE_MS) {

            latestRelativeDistance = -1.0f;
        }

        Log.d(
                "POSE_DISTANCE",
                "pose unavailable reason="
                        + reason
                        + " latest="
                        + latestRelativeDistance);
    }

    private float getLatestRelativeDistance() {

        if (!Float.isFinite(latestRelativeDistance)
                || latestRelativeDistance <= 0.0f) {

            return -1.0f;
        }

        long elapsed =
                SystemClock.elapsedRealtime()
                        - lastValidRuntimePoseMs;

        if (elapsed > RUNTIME_POSE_GRACE_MS) {
            latestRelativeDistance = -1.0f;
            return -1.0f;
        }

        return latestRelativeDistance;
    }

    /**
     * 注册人物期间，同时采集用户喜欢的跟随距离尺度。
     * 输入：
     * fullFrameBitmap：完整摄像头画面
     * personBox：目标人物在完整画面中的检测框
     * 输出：
     * 将有效 fullScale 保存到 registrationTorsoScales
     */
    private void captureRegistrationPoseScale(
            Bitmap fullFrameBitmap,
            RectF personBox) {

        if (targetState != TargetState.REGISTERING
                || poseDistanceEngine == null
                || fullFrameBitmap == null
                || fullFrameBitmap.isRecycled()
                || personBox == null) {

            return;
        }

        long now =
                SystemClock.elapsedRealtime();

        if (now - lastRegistrationPoseCaptureMs
                < REGISTRATION_POSE_INTERVAL_MS) {

            return;
        }

        synchronized (registrationTorsoScales) {
            if (registrationTorsoScales.size()
                    >= MAX_REGISTRATION_POSE_SAMPLES) {

                return;
            }
        }

        /*
         * 即使本次 Pose 识别失败，也等待一段时间再尝试，
         * 避免连续每帧运行 Pose 导致手机过热。
         */
        lastRegistrationPoseCaptureMs = now;

        int frameWidth =
                fullFrameBitmap.getWidth();

        int frameHeight =
                fullFrameBitmap.getHeight();

        /*
         * 人物检测框向四周扩展一点，
         * 防止肩膀或髋部位于检测框边缘而被截断。
         */
        float paddingX =
                personBox.width() * 0.15f;

        float paddingY =
                personBox.height() * 0.10f;

        int cropLeft =
                Math.max(
                        0,
                        (int) Math.floor(
                                personBox.left - paddingX));

        int cropTop =
                Math.max(
                        0,
                        (int) Math.floor(
                                personBox.top - paddingY));

        int cropRight =
                Math.min(
                        frameWidth,
                        (int) Math.ceil(
                                personBox.right + paddingX));

        int cropBottom =
                Math.min(
                        frameHeight,
                        (int) Math.ceil(
                                personBox.bottom + paddingY));

        int cropWidth =
                cropRight - cropLeft;

        int cropHeight =
                cropBottom - cropTop;

        if (cropWidth < 32 || cropHeight < 64) {
            Log.d(
                    "POSE_REGISTER",
                    "valid=false reason=crop_too_small");

            return;
        }

        Bitmap personCrop = null;
        Bitmap uprightCrop = null;

        try {
            personCrop =
                    Bitmap.createBitmap(
                            fullFrameBitmap,
                            cropLeft,
                            cropTop,
                            cropWidth,
                            cropHeight);

            /*
             * 将人物调整为头朝上的方向，
             * 保持与 ReID 注册的方向处理一致。
             */
            uprightCrop =
                    rotateBitmapToUpright(
                            personCrop);

            PoseDistanceEngine.Measurement measurement =
                    poseDistanceEngine.measure(
                            uprightCrop);

            if (!measurement.isValid()) {

                /*
                 * 这里只是注册样本采集失败，
                 * 不应该修改运行时的距离状态。
                 */
                Log.d(
                        "POSE_REGISTER",
                        "valid=false reason="
                                + measurement
                                .getInvalidReason());

                return;
            }

            /*
             * 根据当前画面方向，确定“完整正向画面”的高度。
             */
            int normalizedRotation =
                    ((sensorOrientation % 360) + 360) % 360;

            int uprightFrameHeight;

            if (normalizedRotation == 90
                    || normalizedRotation == 270) {

                uprightFrameHeight = frameWidth;

            } else {
                uprightFrameHeight = frameHeight;
            }

            /*
             * 将裁剪图中的肩髋比例映射回完整画面。
             */
            float fullScale =
                    measurement.getTorsoScale()
                            * uprightCrop.getHeight()
                            / (float) uprightFrameHeight;

            /*
             * 第一层异常保护。
             * 正常人体肩髋尺度通常不会接近 0，
             * 也不会占满整个画面。
             */
            if (!Float.isFinite(fullScale)
                    || fullScale < 0.03f
                    || fullScale > 0.60f) {

                Log.d(
                        "POSE_REGISTER",
                        String.format(
                                Locale.US,
                                "valid=false reason=scale_out_of_range "
                                        + "fullScale=%.4f",
                                fullScale));

                return;
            }

            int sampleCount;

            synchronized (registrationTorsoScales) {
                if (registrationTorsoScales.size()
                        >= MAX_REGISTRATION_POSE_SAMPLES) {

                    return;
                }

                registrationTorsoScales.add(
                        fullScale);

                sampleCount =
                        registrationTorsoScales.size();
            }

            Log.d(
                    "POSE_REGISTER",
                    String.format(
                            Locale.US,
                            "valid=true fullScale=%.4f "
                                    + "visibility=%.3f "
                                    + "sample=%d/%d",
                            fullScale,
                            measurement.getMinimumVisibility(),
                            sampleCount,
                            MAX_REGISTRATION_POSE_SAMPLES));

        } catch (RuntimeException exception) {
            Log.e(
                    "POSE_REGISTER",
                    "Registration pose capture failed",
                    exception);

        } finally {
            if (uprightCrop != null
                    && uprightCrop != personCrop
                    && !uprightCrop.isRecycled()) {

                uprightCrop.recycle();
            }

            if (personCrop != null
                    && !personCrop.isRecycled()) {

                personCrop.recycle();
            }
        }
    }

    private void captureRegistrationSample(
            Bitmap frameBitmap,
            List<Detector.Recognition> recognitions) {

        if (targetState != TargetState.REGISTERING) {
            return;
        }

        if (frameBitmap == null || recognitions == null || recognitions.isEmpty()) {
            return;
        }

        if (targetGalleryViewModel.getSampleCount() >= MAX_GALLERY_SAMPLES) {
            return;
        }

        long currentTime = SystemClock.elapsedRealtime();

        if (currentTime - lastRegistrationCaptureMs
                < REGISTRATION_CAPTURE_INTERVAL_MS) {
            return;
        }

        Detector.Recognition target =
                findMostCenteredPerson(
                        recognitions,
                        frameBitmap.getWidth(),
                        frameBitmap.getHeight());

        if (target == null || target.getLocation() == null) {
            return;
        }

        RectF location = new RectF(target.getLocation());
        /*
         * 同一次注册中采集用户喜欢的跟随尺度。
         */
        captureRegistrationPoseScale(
                frameBitmap,
                location);

        int left = Math.max(0, (int) Math.floor(location.left));
        int top = Math.max(0, (int) Math.floor(location.top));

        int right =
                Math.min(
                        frameBitmap.getWidth(),
                        (int) Math.ceil(location.right));

        int bottom =
                Math.min(
                        frameBitmap.getHeight(),
                        (int) Math.ceil(location.bottom));

        int width = right - left;
        int height = bottom - top;

        if (width < 32 || height < 64) {
            return;
        }

        try {
            // 先按照原始画面坐标裁剪人物
            Bitmap personCrop =
                    Bitmap.createBitmap(
                            frameBitmap,
                            left,
                            top,
                            width,
                            height);

// 裁剪完成后，再根据手机和摄像头方向把人物转正
            Bitmap uprightCrop =
                    rotateBitmapToUpright(personCrop);

            if (uprightCrop != personCrop
                    && personCrop != null
                    && !personCrop.isRecycled()) {
                personCrop.recycle();
            }

// 转正后再统一缩放为 ReID 输入比例
            Bitmap scaledSample =
                    Bitmap.createScaledBitmap(
                            uprightCrop,
                            REID_SAMPLE_WIDTH,
                            REID_SAMPLE_HEIGHT,
                            true);

            if (scaledSample != uprightCrop
                    && uprightCrop != null
                    && !uprightCrop.isRecycled()) {

                uprightCrop.recycle();
            }

            lastRegistrationCaptureMs = currentTime;

            int sampleCount =
                    targetGalleryViewModel.addSample(
                            scaledSample,
                            MAX_GALLERY_SAMPLES);

            int poseSampleCount =
                    getRegistrationPoseSampleCount();

            if (binding != null) {
                binding.targetStatusText.post(
                        () -> {
                            if (binding != null
                                    && targetState == TargetState.REGISTERING) {

                                binding.targetStatusText.setText(
                                        "正在注册，请保持当前位置并缓慢转身\n"
                                                + "身份图片："
                                                + sampleCount
                                                + "/"
                                                + MAX_GALLERY_SAMPLES
                                                + " 张"
                                                + "｜距离样本："
                                                + poseSampleCount
                                                + "/"
                                                + MIN_REGISTRATION_POSE_SAMPLES
                                                + "+ 组");
                            }
                        });
            }

        } catch (IllegalArgumentException exception) {
            Timber.e(exception, "Failed to crop registration sample");
        }
    }

    private void evaluateLiveReIdCandidates(
            Bitmap frameBitmap,
            List<Detector.Recognition> recognitions) {

        // 还没有注册并确认目标人物时，不进行 ReID
        if (targetState != TargetState.SEARCHING) {
            return;
        }

        if (!targetGalleryViewModel
                .hasCompleteRegistration()) {

            resetRuntimeDistance();

            outputTargetPacket(
                    false,
                    0.0f,
                    0.0f);

            return;
        }

        // 每隔若干帧运行一次 ReID
        if (frameNum % LIVE_REID_FRAME_INTERVAL != 0) {
            return;
        }

        float[] galleryEmbedding =
                targetGalleryViewModel.getGalleryEmbedding();


        if (galleryEmbedding == null
                || galleryEmbedding.length != 512) {

            outputTargetPacket(
                    false,
                    0.0f,
                    0.0f);

            return;
        }

        // 当前画面中没有检测到人
        if (recognitions == null || recognitions.isEmpty()) {

            resetRuntimeDistance();
            bestLiveSimilarity = -1.0f;

            updateLiveReIdText(
                    -1.0f,
                    0);

            outputTargetPacket(
                    false,
                    0.0f,
                    0.0f);

            return;
        }

        // 按人物框面积从大到小排序
        List<Detector.Recognition> candidates =
                new ArrayList<>(recognitions);

        candidates.sort(
                (first, second) -> {

                    RectF firstRect =
                            first.getLocation();

                    RectF secondRect =
                            second.getLocation();

                    float firstArea =
                            firstRect == null
                                    ? 0.0f
                                    : firstRect.width()
                                      * firstRect.height();

                    float secondArea =
                            secondRect == null
                                    ? 0.0f
                                    : secondRect.width()
                                      * secondRect.height();

                    return Float.compare(
                            secondArea,
                            firstArea);
                });

        float bestTargetSimilarity = -1.0f;
        RectF bestTargetBox = null;

        int processedCandidateCount = 0;

        try {
            ReIdEngine engine =
                    getOrCreateReIdEngine();

            for (Detector.Recognition candidate : candidates) {

                if (processedCandidateCount
                        >= MAX_LIVE_REID_CANDIDATES) {
                    break;
                }

                RectF location =
                        candidate.getLocation();

                if (location == null) {
                    continue;
                }

                Bitmap personSample =
                        createReIdPersonSample(
                                frameBitmap,
                                location);

                if (personSample == null) {
                    continue;
                }

                try {
                    float[] currentEmbedding =
                            engine.extractEmbedding(
                                    personSample);

                    float similarity =
                            ReIdEngine.cosineSimilarity(
                                    galleryEmbedding,
                                    currentEmbedding);

                    Timber.i(
                            "Live ReID candidate=%d similarity=%.4f",
                            processedCandidateCount,
                            similarity);

                    // 保存相似度最高的人物及其人物框
                    if (similarity > bestTargetSimilarity) {

                        bestTargetSimilarity =
                                similarity;

                        bestTargetBox =
                                new RectF(location);
                    }

                    processedCandidateCount++;

                } finally {

                    if (!personSample.isRecycled()) {
                        personSample.recycle();
                    }
                }
            }

        } catch (Exception exception) {

            Timber.e(
                    exception,
                    "Live ReID inference failed");

            resetRuntimeDistance();

            outputTargetPacket(
                    false,
                    0.0f,
                    0.0f);

            return;
        }

        bestLiveSimilarity =
                bestTargetSimilarity;

        updateLiveReIdText(
                bestTargetSimilarity,
                recognitions.size());



        boolean targetValid =
                bestTargetBox != null
                        && bestTargetSimilarity
                        >= REID_MATCH_THRESHOLD;

        if (targetValid) {
            testPoseDistance(
                    frameBitmap,
                    bestTargetBox);
        }

        if (!targetValid) {

            resetRuntimeDistance();

            outputTargetPacket(
                    false,
                    0.0f,
                    0.0f);

            return;
        }

        /*
         * xError 范围：
         *
         * -1：画面最左侧
         *  0：画面正中央
         * +1：画面最右侧
         */
        float frameCenterX =
                frameBitmap.getWidth() / 2.0f;

        float xError =
                (bestTargetBox.centerX() - frameCenterX)
                        / frameCenterX;

        xError =
                Math.max(
                        -1.0f,
                        Math.min(1.0f, xError));

        // 先记录人物框高度比例，稍后再用于粗略距离估计
        float boxHeightRatio =
                bestTargetBox.height()
                        / frameBitmap.getHeight();

        Log.d(
                "TARGET_BOX",
                String.format(
                        Locale.US,
                        "similarity=%.3f xError=%.3f boxHeightRatio=%.3f",
                        bestTargetSimilarity,
                        xError,
                        boxHeightRatio));

        float relativeDistance =
                getLatestRelativeDistance();

        outputTargetPacket(
                true,
                xError,
                relativeDistance,
                bestTargetSimilarity);
    }


    private void updateLiveReIdText(
            float similarity,
            int detectedPersonCount) {

        if (binding == null) {
            return;
        }

        binding.targetStatusText.post(
                () -> {
                    if (binding == null
                            || targetState
                            != TargetState.SEARCHING) {
                        return;
                    }

                    String personId =
                            targetGalleryViewModel
                                    .getCurrentPersonId();

                    if (detectedPersonCount == 0
                            || similarity < -0.5f) {

                        binding.targetStatusText.setText(
                                "人物 "
                                        + personId
                                        + "｜未检测到 person");

                        return;
                    }

                    binding.targetStatusText.setText(
                            String.format(
                                    Locale.US,
                                    "人物 %s｜检测到 %d 人｜最佳相似度 %.3f",
                                    personId,
                                    detectedPersonCount,
                                    similarity));
                });
    }


    private synchronized ReIdEngine getOrCreateReIdEngine()
            throws IOException, OrtException {

        if (reIdEngine == null) {
            reIdEngine =
                    new ReIdEngine(requireContext().getApplicationContext());
        }

        return reIdEngine;
    }

    private float[] buildGalleryEmbedding(List<Bitmap> samples)
            throws IOException, OrtException {

        if (samples == null || samples.isEmpty()) {
            throw new IllegalArgumentException(
                    "没有可用于 ReID 的人物样本");
        }

        ReIdEngine engine = getOrCreateReIdEngine();

        float[] galleryEmbedding = new float[512];
        int validCount = 0;

        for (Bitmap sample : samples) {
            if (sample == null || sample.isRecycled()) {
                continue;
            }

            float[] embedding =
                    engine.extractEmbedding(sample);

            if (embedding == null || embedding.length != 512) {
                continue;
            }

            for (int i = 0; i < galleryEmbedding.length; i++) {
                galleryEmbedding[i] += embedding[i];
            }

            validCount++;
        }

        if (validCount == 0) {
            throw new IllegalStateException(
                    "没有成功生成任何 ReID 特征");
        }

        for (int i = 0; i < galleryEmbedding.length; i++) {
            galleryEmbedding[i] /= validCount;
        }

        normalizeEmbedding(galleryEmbedding);

        Timber.i(
                "Gallery embedding created. Samples=%d, dimensions=%d",
                validCount,
                galleryEmbedding.length);

        return galleryEmbedding;
    }
    private void normalizeEmbedding(float[] embedding) {
        double squaredSum = 0.0;

        for (float value : embedding) {
            squaredSum += value * value;
        }

        double length = Math.sqrt(squaredSum);

        if (length < 1e-12) {
            throw new IllegalStateException(
                    "ReID Gallery 特征长度为零");
        }

        for (int i = 0; i < embedding.length; i++) {
            embedding[i] =
                    (float) (embedding[i] / length);
        }
    }
    private void showTargetReviewDialog() {
        if (!isAdded() || binding == null) {
            return;
        }

        if (targetReviewDialog != null
                && targetReviewDialog.isShowing()) {
            return;
        }

        View dialogView =
                LayoutInflater.from(requireContext())
                        .inflate(
                                R.layout.dialog_target_review,
                                null,
                                false);

        TextView sampleCountText =
                dialogView.findViewById(
                        R.id.reviewSampleCount);

        TextView personIdText =
                dialogView.findViewById(
                        R.id.reviewPersonId);

        LinearLayout sampleContainer =
                dialogView.findViewById(
                        R.id.reviewSampleContainer);

        View confirmButton =
                dialogView.findViewById(
                        R.id.confirmPersonButton);

        View rerecordButton =
                dialogView.findViewById(
                        R.id.rerecordButton);

        List<Bitmap> samples =
                targetGalleryViewModel.getSamplesSnapshot();

        String personId =
                targetGalleryViewModel.getCurrentPersonId();

        sampleCountText.setText(
                "身份图片："
                        + samples.size()
                        + " 张\n"
                        + "距离样本："
                        + getRegistrationPoseSampleCount()
                        + " 组");

        personIdText.setText(
                "人物 ID：" + personId);

        int imageWidth = dpToPx(90);
        int imageHeight = dpToPx(180);
        int imageMargin = dpToPx(8);

        for (Bitmap sample : samples) {
            if (sample == null || sample.isRecycled()) {
                continue;
            }

            ImageView imageView =
                    new ImageView(requireContext());

            LinearLayout.LayoutParams layoutParams =
                    new LinearLayout.LayoutParams(
                            imageWidth,
                            imageHeight);

            layoutParams.setMargins(
                    0,
                    0,
                    imageMargin,
                    0);

            imageView.setLayoutParams(layoutParams);
            imageView.setScaleType(
                    ImageView.ScaleType.CENTER_CROP);

            imageView.setImageBitmap(sample);

            sampleContainer.addView(imageView);
        }

        targetReviewDialog =
                new AlertDialog.Builder(requireContext())
                        .setView(dialogView)
                        .setCancelable(false)
                        .create();

        confirmButton.setOnClickListener(
                view -> {
                    confirmButton.setEnabled(false);
                    rerecordButton.setEnabled(false);

                    sampleCountText.setText(
                            "正在生成 ReID 身份特征，请稍候……");

                    runInBackground(
                            () -> {
                                try {
                                    List<Bitmap> currentSamples =
                                            targetGalleryViewModel.getSamplesSnapshot();

                                    float[] galleryEmbedding =
                                            buildGalleryEmbedding(currentSamples);
                                    float calculatedBaseline =
                                            calculateRegistrationBaseline();

                                    if (!Float.isFinite(calculatedBaseline)
                                            || calculatedBaseline <= 0.0f) {

                                        throw new IllegalStateException(
                                                "跟随距离标定失败，请重新注册");
                                    }
                                    targetGalleryViewModel.setGalleryEmbedding(
                                            galleryEmbedding);

                                    targetGalleryViewModel.setBaselineTorsoScale(
                                            calculatedBaseline);

                                    /*
                                     * Fragment 中保留一份缓存，
                                     * 实际来源以 ViewModel 为准。
                                     */
                                    baselineTorsoScale =
                                            targetGalleryViewModel
                                                    .getBaselineTorsoScale();

                                    targetGalleryViewModel
                                            .confirmCurrentPerson();

                                    if (!targetGalleryViewModel
                                            .hasCompleteRegistration()) {

                                        throw new IllegalStateException(
                                                "身份或跟随距离注册不完整");
                                    }

                                    if (!isAdded()) {
                                        return;
                                    }

                                    requireActivity()
                                            .runOnUiThread(
                                                    () -> {
                                                        if (binding == null) {
                                                            return;
                                                        }

                                                        if (targetReviewDialog != null) {
                                                            targetReviewDialog.dismiss();
                                                            targetReviewDialog = null;
                                                        }

                                                        updateTargetUi(TargetState.SEARCHING);
                                                        resetRuntimeDistance();
                                                        sendCommandPacket("AUTO");
                                                        setNetworkEnabled(true);

                                                        Toast.makeText(
                                                                        requireContext(),
                                                                        String.format(
                                                                                Locale.US,
                                                                                "注册成功，跟随尺度：%.4f",
                                                                                baselineTorsoScale),
                                                                        Toast.LENGTH_LONG)
                                                                .show();

                                                    });

                                } catch (Exception exception) {
                                    Timber.e(
                                            exception,
                                            "Failed to build ReID gallery embedding");

                                    if (!isAdded()) {
                                        return;
                                    }

                                    requireActivity()
                                            .runOnUiThread(
                                                    () -> {
                                                        sampleCountText.setText(
                                                                "ReID 特征生成失败："
                                                                        + exception.getMessage());

                                                        confirmButton.setEnabled(true);
                                                        rerecordButton.setEnabled(true);
                                                    });
                                }
                            });
                });

        rerecordButton.setOnClickListener(
                view -> {
                    targetReviewDialog.dismiss();
                    targetReviewDialog = null;

                    startTargetRegistration();
                });

        targetReviewDialog.show();
    }

    private int dpToPx(int dp) {
        return Math.round(
                dp
                        * getResources()
                        .getDisplayMetrics()
                        .density);
    }
    private void updateCropImageInfo() {
        //    Timber.i("%s x %s",getPreviewSize().getWidth(), getPreviewSize().getHeight());
        //    Timber.i("%s x %s",getMaxAnalyseImageSize().getWidth(),
        //     getMaxAnalyseImageSize().getHeight());
        frameToCropTransform = null;

        sensorOrientation = 90 - ImageUtils.getScreenOrientation(requireActivity());

        final float textSizePx =
                TypedValue.applyDimension(
                        TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP, getResources().getDisplayMetrics());
        BorderedText borderedText = new BorderedText(textSizePx);
        borderedText.setTypeface(Typeface.MONOSPACE);

        tracker = new MultiBoxTracker(requireContext());
        tracker.setDynamicSpeed(preferencesManager.getDynamicSpeed());

        Timber.i("Camera orientation relative to screen canvas: %d", sensorOrientation);

        recreateNetwork(getModel(), getDevice(), getNumThreads());
        if (detector == null) {
            Timber.e("No network on preview!");
            return;
        }

        binding.trackingOverlay.addCallback(
                canvas -> {
                    if (tracker != null) {
                        tracker.draw(canvas);
                    }
                    //tracker.drawDebug(canvas);
                });
        tracker.setFrameConfiguration(
                getMaxAnalyseImageSize().getWidth(),
                getMaxAnalyseImageSize().getHeight(),
                sensorOrientation);
    }

    protected void onInferenceConfigurationChanged() {
        computingNetwork = false;
        if (croppedBitmap == null) {
            // Defer creation until we're getting camera frames.
            return;
        }
        final Network.Device device = getDevice();
        final Model model = getModel();
        final int numThreads = getNumThreads();
        runInBackground(() -> recreateNetwork(model, device, numThreads));
    }

    private void recreateNetwork(Model model, Network.Device device, int numThreads) {
        resetFpsUi();
        if (model == null) return;
        tracker.clearTrackedObjects();
        if (detector != null) {
            Timber.d("Closing detector.");
            detector.close();
            detector = null;
        }

        try {
            Timber.d("Creating detector (model=%s, device=%s, numThreads=%d)", model, device, numThreads);
            detector = Detector.create(requireActivity(), model, device, numThreads);

            assert detector != null;
            croppedBitmap =
                    Bitmap.createBitmap(
                            detector.getImageSizeX(), detector.getImageSizeY(), Bitmap.Config.ARGB_8888);
            frameToCropTransform =
                    ImageUtils.getTransformationMatrix(
                            getMaxAnalyseImageSize().getWidth(),
                            getMaxAnalyseImageSize().getHeight(),
                            croppedBitmap.getWidth(),
                            croppedBitmap.getHeight(),
                            sensorOrientation,
                            detector.getCropRect(),
                            detector.getMaintainAspect());

            cropToFrameTransform = new Matrix();
            frameToCropTransform.invert(cropToFrameTransform);

            requireActivity()
                    .runOnUiThread(
                            () -> {
                                ArrayAdapter<String> adapter =
                                        new ArrayAdapter<>(
                                                getContext(),
                                                android.R.layout.simple_dropdown_item_1line,
                                                detector.getLabels());
                                binding.classType.setAdapter(adapter);
                                binding.classType.setSelection(
                                        detector.getLabels().indexOf(preferencesManager.getObjectType()));
                                binding.inputResolution.setText(
                                        String.format(
                                                Locale.getDefault(),
                                                "%dx%d",
                                                detector.getImageSizeX(),
                                                detector.getImageSizeY()));
                            });

        } catch (IllegalArgumentException | IOException e) {
            String msg = "Failed to create network.";
            Timber.e(e, msg);
            requireActivity()
                    .runOnUiThread(
                            () ->
                                    Toast.makeText(
                                                    requireContext().getApplicationContext(),
                                                    e.getMessage(),
                                                    Toast.LENGTH_LONG)
                                            .show());
        }
    }

    @Override
    public synchronized void onResume() {
        croppedBitmap = null;
        tracker = null;
        handlerThread = new HandlerThread("inference");
        handlerThread.start();
        handler = new Handler(handlerThread.getLooper());
        binding.bleToggle.setChecked(vehicle.bleConnected());
        super.onResume();
        reloadUdpTargetSender();
        if (voskAgentManager != null) {
            startVoskAgentIfPossible();
        }
    }

    @Override
    public synchronized void onPause() {
        handlerThread.quitSafely();
        try {
            handlerThread.join();
            handlerThread = null;
            handler = null;
        } catch (final InterruptedException e) {
            e.printStackTrace();
        }
        if (voskAgentManager != null) {
            voskAgentManager.stopListening();
        }
        super.onPause();
    }

    @Override
    public void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);

        outState.putString(KEY_TARGET_STATE, targetState.name());
    }

    @Override
    public void onDestroyView() {
        registrationHandler.removeCallbacksAndMessages(null);
        if (binding != null) {
            binding.registrationCountdownText.setVisibility(View.GONE);
        }
        if (targetReviewDialog != null) {
            targetReviewDialog.dismiss();
            targetReviewDialog = null;
        }
        if (voskAgentManager != null) {
            voskAgentManager.shutdown();
            voskAgentManager = null;
        }
        if (udpTargetSender != null) {
            udpTargetSender.close();
            udpTargetSender = null;
        }
        if (poseDistanceEngine != null) {
            poseDistanceEngine.close();
            poseDistanceEngine = null;
        }
        super.onDestroyView();
        binding = null;
    }

    protected synchronized void runInBackground(final Runnable r) {
        if (handler != null) {
            handler.post(r);
        }
    }

    @Override
    protected void processUSBData(String data) {

        Log.d(
                "USB_RX",
                "Received: " + data);

        if (binding != null) {
            binding.controllerContainer.speedInfo.setText(
                    getString(
                            R.string.speedInfo,
                            String.format(
                                    Locale.US,
                                    "%3.0f,%3.0f",
                                    vehicle.getLeftWheelRpm(),
                                    vehicle.getRightWheelRpm())));
        }
    }

    @Override
    protected void processControllerKeyData(String commandType) {
        switch (commandType) {
            case Constants.CMD_DRIVE:
                binding.controllerContainer.controlInfo.setText(
                        String.format(Locale.US, "%.0f,%.0f", vehicle.getLeftSpeed(), vehicle.getRightSpeed()));
                break;

            case Constants.CMD_NETWORK:
                setNetworkEnabledWithAudio(!binding.autoSwitch.isChecked());
                break;
        }
    }

    private void setNetworkEnabledWithAudio(boolean b) {
        setNetworkEnabled(b);

        if (b) audioPlayer.play(voice, "network_enabled.mp3");
        else audioPlayer.playDriveMode(voice, vehicle.getDriveMode());
    }

    private void setNetworkEnabled(boolean b) {
        binding.autoSwitch.setChecked(b);

        binding.controllerContainer.controlMode.setEnabled(!b);
        binding.controllerContainer.driveMode.setEnabled(!b);
        binding.controllerContainer.speedMode.setEnabled(!b);

        binding.controllerContainer.controlMode.setAlpha(b ? 0.5f : 1f);
        binding.controllerContainer.driveMode.setAlpha(b ? 0.5f : 1f);
        binding.controllerContainer.speedMode.setAlpha(b ? 0.5f : 1f);

        resetFpsUi();
        if (!b) handler.postDelayed(() -> vehicle.setControl(0, 0), Math.max(lastProcessingTimeMs, 50));
    }

    @Override
    protected void processFrame(Bitmap bitmap, ImageProxy image) {
        if (tracker == null) updateCropImageInfo();

        ++frameNum;
        if (binding != null && binding.autoSwitch.isChecked()) {
            // If network is busy, return.
            if (computingNetwork) {
                return;
            }

            computingNetwork = true;
            Timber.i("Putting image " + frameNum + " for detection in bg thread.");

            runInBackground(
                    () -> {
                        final Bitmap frameForDetection;

                        if (lensFacing == CameraSelector.LENS_FACING_FRONT) {
                            frameForDetection =
                                    CameraUtils.flipBitmapHorizontal(bitmap);
                        } else {
                            frameForDetection = bitmap;
                        }

                        final Canvas canvas = new Canvas(croppedBitmap);

                        canvas.drawBitmap(
                                frameForDetection,
                                frameToCropTransform,
                                null);

                        if (detector != null) {
                            Timber.i("Running detection on image %s", frameNum);
                            final long startTime = SystemClock.elapsedRealtime();
                            final List<Detector.Recognition> results =
                                    detector.recognizeImage(croppedBitmap, classType);
                            lastProcessingTimeMs = SystemClock.elapsedRealtime() - startTime;

                            if (!results.isEmpty())
                                Timber.i(
                                        "Object: "
                                                + results.get(0).getLocation().centerX()
                                                + ", "
                                                + results.get(0).getLocation().centerY()
                                                + ", "
                                                + results.get(0).getLocation().height()
                                                + ", "
                                                + results.get(0).getLocation().width());

                            cropCopyBitmap = Bitmap.createBitmap(croppedBitmap);
                            final Canvas canvas1 = new Canvas(cropCopyBitmap);
                            final Paint paint = new Paint();
                            paint.setColor(Color.RED);
                            paint.setStyle(Paint.Style.STROKE);
                            paint.setStrokeWidth(2.0f);

                            final List<Detector.Recognition> mappedRecognitions = new LinkedList<>();

                            for (final Detector.Recognition result : results) {
                                final RectF location = result.getLocation();
                                if (location != null && result.getConfidence() >= MINIMUM_CONFIDENCE_TF_OD_API) {
                                    canvas1.drawRect(location, paint);
                                    cropToFrameTransform.mapRect(location);
                                    result.setLocation(location);
                                    mappedRecognitions.add(result);
                                }
                            }
                            captureRegistrationSample(
                                    frameForDetection,
                                    mappedRecognitions);
                            evaluateLiveReIdCandidates(
                                    frameForDetection,
                                    mappedRecognitions);

                            tracker.trackResults(mappedRecognitions, frameNum);
//                            Control target = tracker.updateTarget();
//                            if (mirrorControl) {
//                                handleDriveCommand(target.mirror());
//                            } else {
//                                handleDriveCommand(target);
//                            }
                            tracker.updateTarget();

                            // ReID 尚未接入前，新页面禁止自动驱动车辆
                            //vehicle.setControl(0, 0);
                            binding.trackingOverlay.postInvalidate();
                        }

                        computingNetwork = false;
                    });
            if (lastProcessingTimeMs > 0) {
                if (isBenchmarkMode) {
                    double avgProcessingTimeMs = movingAvgProcessingTimeMs.next(lastProcessingTimeMs);
                    processedFrames += 1;
                    if (processedFrames >= movingAvgSize) updateFpsUi(avgProcessingTimeMs);
                } else updateFpsUi(lastProcessingTimeMs);
            }
        }
    }

    private void updateFpsUi(double processingTimeMs) {
        requireActivity()
                .runOnUiThread(
                        () ->
                                binding.inferenceInfo.setText(
                                        String.format(Locale.US, "%.1f fps", 1000.f / processingTimeMs)));
    }

    private void resetFpsUi() {
        processedFrames = 0;
        movingAvgProcessingTimeMs = new MovingAverage(movingAvgSize);
        requireActivity().runOnUiThread(() -> binding.inferenceInfo.setText(R.string.time_fps));
    }

    protected void handleDriveCommand(Control control) {
        vehicle.setControl(control);
        float left = vehicle.getLeftSpeed();
        float right = vehicle.getRightSpeed();
        requireActivity()
                .runOnUiThread(
                        () ->
                                binding.controllerContainer.controlInfo.setText(
                                        String.format(Locale.US, "%.0f,%.0f", left, right)));
    }

    protected Model getModel() {
        return model;
    }

    @Override
    protected void setModel(Model model) {
        if (this.model != model) {
            Timber.d("Updating  model: %s", model);
            this.model = model;
            preferencesManager.setObjectNavModel(model.name);
            onInferenceConfigurationChanged();
        }
    }

    protected Network.Device getDevice() {
        return device;
    }

    private void setDevice(Network.Device device) {
        if (this.device != device) {
            Timber.d("Updating  device: %s", device);
            this.device = device;
            final boolean threadsEnabled = device == Network.Device.CPU;
            binding.plus.setEnabled(threadsEnabled);
            binding.minus.setEnabled(threadsEnabled);
            binding.threads.setText(threadsEnabled ? String.valueOf(numThreads) : "N/A");
            if (threadsEnabled) binding.threads.setTextColor(Color.BLACK);
            else binding.threads.setTextColor(Color.GRAY);
            preferencesManager.setDevice(device.ordinal());
            onInferenceConfigurationChanged();
        }
    }

    protected int getNumThreads() {
        return numThreads;
    }

    private void setNumThreads(int numThreads) {
        if (this.numThreads != numThreads) {
            Timber.d("Updating  numThreads: %s", numThreads);
            this.numThreads = numThreads;
            preferencesManager.setNumThreads(numThreads);
            onInferenceConfigurationChanged();
        }
    }

    private String[] getModelFiles() {
        return requireActivity().getFilesDir().list((dir1, name) -> name.endsWith(".tflite"));
    }

    private void setSpeedMode(Enums.SpeedMode speedMode) {
        if (speedMode != null) {
            switch (speedMode) {
                case SLOW:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_low);
                    break;
                case NORMAL:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_medium);
                    break;
                case FAST:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_high);
                    break;
            }

            Timber.d("Updating  controlSpeed: %s", speedMode);
            preferencesManager.setSpeedMode(speedMode.getValue());
            vehicle.setSpeedMultiplier(speedMode.getValue());
        }
    }

    private void setControlMode(Enums.ControlMode controlMode) {
        if (controlMode != null) {
            switch (controlMode) {
                case GAMEPAD:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_controller);
                    disconnectPhoneController();
                    break;
                case PHONE:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_phone);
                    if (!PermissionUtils.hasControllerPermissions(requireActivity()))
                        requestPermissionLauncher.launch(Constants.PERMISSIONS_CONTROLLER);
                    else connectPhoneController();
                    break;
                case WEBSERVER:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_server);
                    if (!PermissionUtils.hasControllerPermissions(requireActivity()))
                        requestPermissionLauncher.launch(Constants.PERMISSIONS_CONTROLLER);
                    else connectWebController();
                    break;
            }
            Timber.d("Updating  controlMode: %s", controlMode);
            preferencesManager.setControlMode(controlMode.getValue());
        }
    }

    protected void setDriveMode(Enums.DriveMode driveMode) {
        if (driveMode != null) {
            switch (driveMode) {
                case DUAL:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_dual);
                    break;
                case GAME:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_game);
                    break;
                case JOYSTICK:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_joystick);
                    break;
            }

            Timber.d("Updating  driveMode: %s", driveMode);
            vehicle.setDriveMode(driveMode);
            preferencesManager.setDriveMode(driveMode.getValue());
        }
    }

    private void connectPhoneController() {
        phoneController.connect(requireContext());
        Enums.DriveMode oldDriveMode = currentDriveMode;
        // Currently only dual drive mode supported
        setDriveMode(Enums.DriveMode.DUAL);
        binding.controllerContainer.driveMode.setAlpha(0.5f);
        binding.controllerContainer.driveMode.setEnabled(false);
        preferencesManager.setDriveMode(oldDriveMode.getValue());
    }

    private void connectWebController() {
        phoneController.connectWebServer();
        Enums.DriveMode oldDriveMode = currentDriveMode;
        // Currently only dual drive mode supported
        setDriveMode(Enums.DriveMode.GAME);
        binding.controllerContainer.driveMode.setAlpha(0.5f);
        binding.controllerContainer.driveMode.setEnabled(false);
        preferencesManager.setDriveMode(oldDriveMode.getValue());
    }

    private void disconnectPhoneController() {
        phoneController.disconnect();
        setDriveMode(Enums.DriveMode.getByID(preferencesManager.getDriveMode()));
        binding.controllerContainer.driveMode.setEnabled(true);
        binding.controllerContainer.driveMode.setAlpha(1.0f);
    }

    @Override
    public void onDestroy() {
        if (reIdEngine != null) {
            reIdEngine.close();
            reIdEngine = null;
        }

        super.onDestroy();
    }
}


