package org.openbot.objectNav;

import android.util.Log;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class UdpTargetSender implements AutoCloseable {

    private static final String TAG = "UdpTargetSender";

    private final String host;
    private final int port;

    private final ExecutorService executor =
            Executors.newSingleThreadExecutor();

    private volatile boolean closed = false;

    private DatagramSocket socket;
    private InetAddress address;

    public UdpTargetSender(
            String host,
            int port) {

        this.host = host;
        this.port = port;
    }

    private void ensureSocket()
            throws IOException {

        if (address == null) {
            address =
                    InetAddress.getByName(host);
        }

        if (socket == null
                || socket.isClosed()) {

            socket =
                    new DatagramSocket();
        }
    }

    public void send(String message) {

        if (closed
                || message == null
                || message.isEmpty()) {
            return;
        }

        executor.execute(
                () -> {
                    try {
                        ensureSocket();

                        byte[] data =
                                message.getBytes(
                                        StandardCharsets.UTF_8);

                        DatagramPacket packet =
                                new DatagramPacket(
                                        data,
                                        data.length,
                                        address,
                                        port);

                        socket.send(packet);

                        Log.d(
                                TAG,
                                "UDP sent to "
                                        + host
                                        + ":"
                                        + port
                                        + " -> "
                                        + message.trim());

                    } catch (IOException exception) {

                        if (!closed) {
                            Log.e(
                                    TAG,
                                    "UDP send failed",
                                    exception);
                        }
                    }
                });
    }

    @Override
    public void close() {

        closed = true;

        if (socket != null) {
            socket.close();
            socket = null;
        }

        executor.shutdownNow();
    }
}