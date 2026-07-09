package org.openbot.markerfollow;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class UdpTargetSender {

    private final String targetIp;
    private final int targetPort;
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();

    public UdpTargetSender(String targetIp, int targetPort) {
        this.targetIp = targetIp;
        this.targetPort = targetPort;
    }

    public void send(String message) {
        executorService.execute(() -> {
            try {
                byte[] data = message.getBytes("UTF-8");
                InetAddress address = InetAddress.getByName(targetIp);

                DatagramPacket packet = new DatagramPacket(
                        data,
                        data.length,
                        address,
                        targetPort
                );

                DatagramSocket socket = new DatagramSocket();
                socket.send(packet);
                socket.close();

            } catch (Exception e) {
                e.printStackTrace();
            }
        });
    }

    public void close() {
        executorService.shutdownNow();
    }
}