/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.Manifest;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.Criteria;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.util.Pair;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.helpers.ParameterLoaderNode;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeListener;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.NodeListener;

import org.ros.rosjava_tutorial_native_node.MoveBaseNativeNode;
import org.ros.rosjava_tutorial_native_node.LsmNativeNode;
import org.ros.rosjava_tutorial_native_node.AmclNativeNode;
import org.ros.rosjava_tutorial_native_node.FastLioNativeNode;
import org.ros.rosjava_tutorial_native_node.LivoxRosDriver2NativeNode;
import org.ros.rosjava_tutorial_native_node.LaserLoggerNativeNode;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.CountDownLatch;

public class MainActivity extends RosActivity implements View.OnClickListener {
    private static final String TAG = MainActivity.class.getSimpleName();
    static {
        System.loadLibrary("movebase_jni");
        System.loadLibrary("lsm_jni");
        System.loadLibrary("amcl_jni");
        System.loadLibrary("laser_logger_jni");
        System.loadLibrary("fastlio_jni");
        System.loadLibrary("livox_ros_driver2_jni");
    }

    private static ArrayList<Pair<String, String>> mResourcesToLoad = new ArrayList<Pair<String, String>>() {{
        add(new Pair<>("movebase_params/costmap_common_params_burger.yaml", MoveBaseNativeNode.nodeName + "/local_costmap"));
        add(new Pair<>("movebase_params/costmap_common_params_burger.yaml", MoveBaseNativeNode.nodeName + "/global_costmap"));
        add(new Pair<>("movebase_params/local_costmap_params.yaml", MoveBaseNativeNode.nodeName + "/local_costmap"));
        add(new Pair<>("movebase_params/global_costmap_params.yaml", MoveBaseNativeNode.nodeName + "/global_costmap"));
        add(new Pair<>("movebase_params/dwa_local_planner_params_burger.yaml", MoveBaseNativeNode.nodeName + "/DWAPlannerROS"));
        add(new Pair<>("movebase_params/move_base_params.yaml", MoveBaseNativeNode.nodeName));
//        add(new Pair<>("movebase_params/amcl_params.yaml", AmclNativeNode.nodeName));
        add(new Pair<String, String>("movebase_params/fastlio2_params.yaml", "/"));
        // We use the global namespace for fastlio2.
        add(new Pair<String, String>("movebase_params/livox_ros_driver2_params.yaml", "/"));
        // We use the global namespace for livox ros driver2.
    }};

    private ArrayList<ParameterLoaderNode.Resource> mOpenedResources = new ArrayList<>();


    private NodeMainExecutor nodeMainExecutor = null;
    private URI masterUri;
    private String hostName;
    private MoveBaseNativeNode moveBaseNativeNode;

    private LsmNativeNode lsmNativeNode;
    private AmclNativeNode amclNativeNode;

    private FastLioNativeNode fastlioNativeNode;
    private LivoxRosDriver2NativeNode livoxNativeNode;

    private LaserLoggerNativeNode laserLoggerNativeNode;
    private PathListenerNode pathListenerNode;
    private ParameterLoaderNode mParameterLoaderNode;

    private ModuleStatusIndicator mRosParametersStatusIndicator;
    private ModuleStatusIndicator mRosNavigationStatusIndicator;

    private EditText locationFrameIdView, imuFrameIdView;
    private TextView masterUriTextView;
    Button applyB;
    boolean recording;
    private OnFrameIdChangeListener locationFrameIdListener, imuFrameIdListener;

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        locationFrameIdListener = new OnFrameIdChangeListener() {
            @Override
            public void onFrameIdChanged(String newFrameId) {
                Log.w(TAG, "Default location OnFrameIdChangedListener called");
            }
        };
        imuFrameIdListener = new OnFrameIdChangeListener() {
            @Override
            public void onFrameIdChanged(String newFrameId) {
                Log.w(TAG, "Default IMU OnFrameIdChangedListener called");
            }
        };

        locationFrameIdView = findViewById(R.id.et_location_frame_id);
        imuFrameIdView = findViewById(R.id.et_imu_frame_id);
        masterUriTextView = findViewById(R.id.tv_master_uri);

        SharedPreferences sp = getSharedPreferences("SharedPreferences", MODE_PRIVATE);
        locationFrameIdView.setText(sp.getString("locationFrameId", getString(R.string.default_location_frame_id)));
        imuFrameIdView.setText(sp.getString("imuFrameId", getString(R.string.default_imu_frame_id)));

        applyB = findViewById(R.id.b_apply);
        applyB.setOnClickListener(this);
        mRosParametersStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_config_ok_image));
        mRosNavigationStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_navigation_ok_image));

        Button button = (Button) findViewById(R.id.cancelgoalbutton);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                cancelGoals();
            }
        });

        final Button logbutton = (Button) findViewById(R.id.logbutton);
        logbutton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (recording) {
                    stopLaserLogging();
                    logbutton.setText(R.string.start_logging);
                } else {
                    startLaserLogging();
                    logbutton.setText(R.string.stop_logging);
                }
                recording = !recording;
            }
        });

        // Load raw resources
        for (Pair<String, String> ip : mResourcesToLoad) {
            InputStream assetInStream=null;
            try { // https://stackoverflow.com/questions/1933015/opening-a-file-from-assets-folder-in-android
                assetInStream=getAssets().open(ip.first);
            } catch (IOException e) {
                e.printStackTrace();
            }
            mOpenedResources.add(new ParameterLoaderNode.Resource(assetInStream, ip.second));
        }
//        addRuntimeParameters();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        Log.d(TAG, "init()");

        // Store a reference to the NodeMainExecutor and unblock any processes that were waiting
        // for this to start ROS Nodes
        this.nodeMainExecutor = nodeMainExecutor;
        masterUri = getMasterUri();
        hostName = getRosHostname();

        Log.i(TAG, "Master URI: " + masterUri.toString());
        configureParameterServer();
//        startAmcl();
//        startLsm();
        startLivoxRosDriver2();
        startFastLio();
//        startPathListener();
        startMoveBase();

        final LocationPublisherNode locationPublisherNode = new LocationPublisherNode();
        ImuPublisherNode imuPublisherNode = new ImuPublisherNode();

        MainActivity.this.locationFrameIdListener = locationPublisherNode.getFrameIdListener();
        MainActivity.this.imuFrameIdListener = imuPublisherNode.getFrameIdListener();

        Criteria criteria = new Criteria();
        criteria.setAccuracy(Criteria.ACCURACY_FINE);
        criteria.setPowerRequirement(Criteria.POWER_LOW);
        criteria.setAltitudeRequired(false);
        criteria.setBearingRequired(false);
        criteria.setSpeedRequired(false);
        criteria.setCostAllowed(true);
        final String provider = LocationManager.GPS_PROVIDER;
        String svcName = Context.LOCATION_SERVICE;
        final LocationManager locationManager = (LocationManager) getSystemService(svcName);
        final int t = 500;
        final float distance = 0.1f;

        MainActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                    boolean permissionFineLocation = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
                    boolean permissionCoarseLocation = checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;
                    Log.d(TAG, "PERMISSION 1: " + String.valueOf(permissionFineLocation));
                    Log.d(TAG, "PERMISSION 2: " + String.valueOf(permissionCoarseLocation));
                    if (permissionFineLocation && permissionCoarseLocation) {
                        if (locationManager != null) {
                            Log.d(TAG, "Requesting location");
                            locationManager.requestLocationUpdates(provider, t, distance,
                                    locationPublisherNode.getLocationListener());
                        }
                    } else {
                        // Request permissions
                        requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, PackageManager.GET_PERMISSIONS);
                    }
                } else {
                    locationManager.requestLocationUpdates(provider, t, distance, locationPublisherNode.getLocationListener());
                }
                masterUriTextView.setText("Master URI:" + masterUri.toString());
            }
        });

        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        try {
            Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            sensorManager.registerListener(imuPublisherNode.getAccelerometerListener(), accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        } catch (NullPointerException e) {
            Log.e(TAG, e.toString());
            return;
        }

        SensorManager sensorManager1 = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        try {
            Sensor gyroscope = sensorManager1.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            sensorManager1.registerListener(imuPublisherNode.getGyroscopeListener(), gyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        } catch (NullPointerException e) {
            Log.e(TAG, e.toString());
            return;
        }

        SensorManager sensorManager2 = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        try {
            Sensor orientation = sensorManager2.getDefaultSensor(Sensor.TYPE_ORIENTATION);
            sensorManager2.registerListener(imuPublisherNode.getOrientationListener(), orientation, SensorManager.SENSOR_DELAY_FASTEST);
        } catch (NullPointerException e) {
            Log.e(TAG, e.toString());
            return;
        }

        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.

        // The user can easily use the selected ROS Hostname in the master chooser
        // activity.

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

//        nodeMainExecutor.execute(locationPublisherNode, nodeConfiguration);
//        nodeMainExecutor.execute(imuPublisherNode, nodeConfiguration);

        onClick(null);
    }

    @Override
    public void onClick(View view) {
        Log.i(TAG, "Default IMU OnFrameIdChangedListener called");

        SharedPreferences sp = getSharedPreferences("SharedPreferences", MODE_PRIVATE);
        SharedPreferences.Editor spe = sp.edit();
        String newLocationFrameId = locationFrameIdView.getText().toString();
        if (!newLocationFrameId.isEmpty()) {
            locationFrameIdListener.onFrameIdChanged(newLocationFrameId);
            spe.putString("locationFrameId", newLocationFrameId);
        }
        String newImuFrameId = imuFrameIdView.getText().toString();
        if (!newLocationFrameId.isEmpty()) {
            imuFrameIdListener.onFrameIdChanged(newImuFrameId);
            spe.putString("imuFrameId", newImuFrameId);
        }
        spe.apply();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PackageManager.GET_PERMISSIONS) {
            if (grantResults[0] == PackageManager.PERMISSION_GRANTED
                    && grantResults[1] == PackageManager.PERMISSION_GRANTED) {
                Log.d(TAG, "Permissions granted!");
            } else {
                Log.e(TAG, "Permissions not granted.");
            }
        }
    }

    /**
     * Adds resources to be loaded that depend on the device
     * (they cannot be set at compile time statically).
     */
    private void addRuntimeParameters() {
        String androidApiLevel = "android_api_level: " + Integer.toString(Build.VERSION.SDK_INT);
        Log.i(TAG, "Adding android API level parameter: " + androidApiLevel);
        mOpenedResources.add(new ParameterLoaderNode.Resource(
                new ByteArrayInputStream(androidApiLevel.getBytes()), "/tango"));
    }

    /**
     * Helper method to block the calling thread until the latch is zeroed by some other task.
     * @param latch Latch to wait for.
     * @param latchName Name to be used in log messages for the given latch.
     */
    private void waitForLatchUnlock(CountDownLatch latch, String latchName) {
        try {
            Log.i(TAG, "Waiting for " + latchName + " latch release...");
            latch.await();
            Log.i(TAG,latchName + " latch released!");
        } catch (InterruptedException ie) {
            Log.w(TAG, "Warning: continuing before " + latchName + " latch was released");
        }
    }

    /**
     * Starts {@link ParameterLoaderNode} and waits for it to finish setting parameters.
     */
    private void configureParameterServer() {
        CountDownLatch latch = new CountDownLatch(1);
        startParameterLoaderNode(latch);
        waitForLatchUnlock(latch, "parameter");
    }

    private void startParameterLoaderNode(final CountDownLatch latch) {
        // Create node to load configuration to Parameter Server
        Log.i(TAG, "Setting parameters in Parameter Server");
        mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(ParameterLoaderNode.NODE_NAME);
        mParameterLoaderNode = new ParameterLoaderNode(mOpenedResources);
        nodeMainExecutor.execute(mParameterLoaderNode, nodeConfiguration,
                new ArrayList<NodeListener>() {{
                    add(new DefaultNodeListener() {
                        @Override
                        public void onShutdown(Node node) {
                            latch.countDown();
                            mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                        }

                        @Override
                        public void onError(Node node, Throwable throwable) {
                            Log.e(TAG, "Error loading parameters to ROS parameter server: " + throwable.getMessage(), throwable);
                            mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                        }
                    });
                }});
    }

    private String checkPointCloudPcdMap() {
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath();
        File mapFile = new File(extdir, "maps/map.pcd");
        // we do not copy the pgm here because the pgm from the apk is corrupt and unable to be loaded by the map_server.
        if (!mapFile.exists()) {
            Log.e(TAG, "Error cannot find point cloud map: " + mapFile.getAbsolutePath() +
                    "\n.Push them into the folder with adb push.");
        }
        return mapFile.getAbsolutePath();
    }

    private String checkOccupancyGridMap() {
        // Create the sample map in the app data dir
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath();
        File mapFile = new File(extdir, "maps/map.yaml");
        // we do not copy the pgm here because the pgm from the apk is corrupt and unable to be loaded by the map_server.
        // The pgm file should be put into the /sdcard/Android/data/org.ollide.rosandroid/files/data folder by adb push.
        if (!mapFile.exists()) {
            Log.e(TAG, "Error cannot find occupancy grid map: " + mapFile.getAbsolutePath() +
                    "\n.Push them into the folder with adb push.");
        }
        return mapFile.getAbsolutePath();
    }

    private String createLidarUserConfig(String hostip, String lidarip) {
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath();
        File configFile = new File(extdir, "MID360_config.json");
        String config = LivoxRosDriver2NativeNode.createLidarConfig(hostip, lidarip);
        FileManager.writeToFile(config, configFile);
        return configFile.getAbsolutePath();
    }

    private String copyBurgerUrdf() {
        // Create the sample map in the app data dir
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath() + "/movebase_params";
        if (!new File(extdir).exists()) {
            new File(extdir).mkdir();
        }
        String burgerUrdf = extdir + "/turtlebot3_burger.urdf";
        File burgerUrdfFile = new File(burgerUrdf);
        if (!burgerUrdfFile.exists()) {
            FileManager.copyAssetFile(getAssets(), "movebase_params/turtlebot3_burger.urdf", burgerUrdf);
        }
        return burgerUrdf;
    }

    private String copyClassifierAsset() {
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath() + "/classifiers";
        File f = new File(extdir);
        if (!f.exists()) {
            new File(extdir).mkdir();
            FileManager.copyAssetDir(getAssets(), "classifiers", extdir);
        }
        return extdir;
    }

    private String getUniqueBagPath() {
        String extdir = getExternalFilesDir(
                Environment.getDataDirectory().getAbsolutePath()).getAbsolutePath() + "/rosbags";
        if (!new File(extdir).exists()) {
            new File(extdir).mkdir();
        }
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.getDefault());
        String currentDateandTime = sdf.format(new Date());
        return extdir + "/" + currentDateandTime + ".bag";
    }

    // Create a native movebase node
    private void startMoveBase()
    {
        Log.i(TAG, "Starting native movebase node wrapper...");
        mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(MoveBaseNativeNode.nodeName);

        String burgerUrdf = copyBurgerUrdf();
        String mapYaml = checkOccupancyGridMap();
        String[] extraArgs = new String[2];
        extraArgs[0] = burgerUrdf;
        extraArgs[1] = mapYaml;
        moveBaseNativeNode = new MoveBaseNativeNode(extraArgs);
        nodeMainExecutor.execute(moveBaseNativeNode, nodeConfiguration, new ArrayList<NodeListener>(){{
            add(new DefaultNodeListener() {
                @Override
                public void onStart(ConnectedNode connectedNode) {
                    mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                }

                @Override
                public void onError(Node node, Throwable throwable) {
                    mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                }
            });
        }});
    }


    private void startLsm() {
        Log.i(TAG, "Starting native laser scan matcher node wrapper...");

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(LsmNativeNode.nodeName);

        lsmNativeNode = new LsmNativeNode();
        nodeMainExecutor.execute(lsmNativeNode, nodeConfiguration);
    }

    private void startAmcl() {
        Log.i(TAG, "Starting native amcl node wrapper...");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(AmclNativeNode.nodeName);

        String global_loc_at_start = "false";
        String[] extraArgs = new String[1];
        extraArgs[0] = global_loc_at_start;
        amclNativeNode = new AmclNativeNode(extraArgs);
        nodeMainExecutor.execute(amclNativeNode, nodeConfiguration);
    }

    private void startFastLio() {
        Log.i(TAG, "Starting native fastlio node wrapper...");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(FastLioNativeNode.nodeName);
        String pcdmappath = checkPointCloudPcdMap();
        String[] extraArgs = new String[1];
        extraArgs[0] = pcdmappath;
        fastlioNativeNode = new FastLioNativeNode(extraArgs);
        nodeMainExecutor.execute(fastlioNativeNode, nodeConfiguration);
    }

    private void startLivoxRosDriver2() {
        Log.i(TAG, "Starting native livox ros driver2 node wrapper...");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(LivoxRosDriver2NativeNode.nodeName);
        String hostip = "192.168.217.234";
        String lidarip = "192.168.217.121";
        String userconfigpath = createLidarUserConfig(hostip, lidarip);
        String[] extraArgs = new String[1];
        extraArgs[0] = userconfigpath;
        livoxNativeNode = new LivoxRosDriver2NativeNode(extraArgs);
        nodeMainExecutor.execute(livoxNativeNode, nodeConfiguration);
    }

    private void startPathListener() {
        Log.i(TAG, "Starting path listener node...");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(PathListenerNode.nodeName);

        pathListenerNode = new PathListenerNode();
        nodeMainExecutor.execute(pathListenerNode, nodeConfiguration);
    }

    private void cancelGoals() {
        Log.i(TAG, "Starting cancel goals...");
        moveBaseNativeNode.setCancelGoals();
    }

    private void startLaserLogging() {
        Log.i(TAG, "Starting native laser logging node wrapper...");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName(LaserLoggerNativeNode.nodeName);
        String rosbagPath = getUniqueBagPath();
        String[] extraArgs = new String[1];
        extraArgs[0] = rosbagPath;
        laserLoggerNativeNode = new LaserLoggerNativeNode(extraArgs);
        nodeMainExecutor.execute(laserLoggerNativeNode, nodeConfiguration);
    }

    private void stopLaserLogging() {
        laserLoggerNativeNode.shutdown();
    }

}
