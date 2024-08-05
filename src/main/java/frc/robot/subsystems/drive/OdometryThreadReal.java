// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.Constants;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.MapleTimeUtils;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThreadReal extends Thread implements OdometryThread {
    private final OdometryThread.OdometryDoubleInput[] odometryDoubleInputs;
    private final BaseStatusSignal[] statusSignals;
    private final Queue<Double> timeStampsQueue;
    private final Lock lock = new ReentrantLock();
    public OdometryThreadReal(OdometryThread.OdometryDoubleInput[] odometryDoubleInputs, BaseStatusSignal[] statusSignals) {
        this.timeStampsQueue = new ArrayBlockingQueue<>(Constants.SwerveDriveChassisConfigs.ODOMETRY_CACHE_CAPACITY);
        this.odometryDoubleInputs = odometryDoubleInputs;
        this.statusSignals = statusSignals;

        setName("OdometryThread");
        setDaemon(true);
    }

    @Override
    public synchronized void start() {
        if (odometryDoubleInputs.length > 0)
            super.start();
    }


    @Override
    public void run() {
        while (true) odometryPeriodic();
    }

    private void odometryPeriodic() {
        refreshSignalsAndBlockThread();

        lock.lock();
        timeStampsQueue.offer(estimateAverageTimeStamps());
        for(OdometryThread.OdometryDoubleInput odometryDoubleInput : odometryDoubleInputs)
            odometryDoubleInput.cacheInputToQueue();
        lock.unlock();
    }

    private void refreshSignalsAndBlockThread() {
        switch (Constants.SwerveDriveChassisConfigs.SWERVE_DRIVE_TYPE) {
            case REV ->
                    MapleTimeUtils.delay(1.0 / Constants.SwerveDriveChassisConfigs.ODOMETRY_FREQUENCY);
            case CTRE_ON_RIO -> {
                MapleTimeUtils.delay(1.0 / Constants.SwerveDriveChassisConfigs.ODOMETRY_FREQUENCY);
                BaseStatusSignal.refreshAll();
            }
            case CTRE_ON_CANIVORE ->
                    BaseStatusSignal.waitForAll(Constants.SwerveDriveChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS, statusSignals);
        }
    }

    private double estimateAverageTimeStamps() {
        double currentTime = MapleTimeUtils.getRealTimeSeconds(), totalLatency = 0;
        for (BaseStatusSignal signal:statusSignals)
            totalLatency += signal.getTimestamp().getLatency();

        if (statusSignals.length == 0)
            return currentTime;
        return currentTime - totalLatency / statusSignals.length;
    }


    @Override
    public void updateInputs(OdometryThreadInputs inputs) {
        inputs.measurementTimeStamps = new double[timeStampsQueue.size()];
        for (int i = 0; i < inputs.measurementTimeStamps.length && !timeStampsQueue.isEmpty(); i++)
            inputs.measurementTimeStamps[i] = timeStampsQueue.poll();
    }

    @Override
    public void lockOdometry() {
        lock.lock();
    }

    @Override
    public void unlockOdometry() {
        lock.unlock();
    }
}
