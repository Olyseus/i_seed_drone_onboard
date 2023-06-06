Environment preparation:

- Put some I-Seed objects under the drone. So when camera pointing down
  and making shots will have something to detect. If nothing will be detected
  during forward mission the backward mission with laser measurement will
  not be started

Drone preparation:

- The camera should be set to ZOOM. You can't do it with API. It's an SDK bug
- Zooming should be set to x1.3. The minimum zoom value is x2 when you do it
  with the API. Which is not a minimum focal length
- Smart controller trick to set zoom to x1.3:
  https://olyseusinnovations.slack.com/archives/G01JZSKBA4A/p1669281251318429
- Location in the simulator should be set to latitude "48.90", longitude "-9.40"
  (uninhabited area in the Celtic Sea)
- RTK simulation is not supported and should be disabled

Simulation notes:

- There is no real laser range measurement, it's simulated inside onboard service
- There is no interconnection between onboard service and Android control device
- Mission and inference are run in parallel. So in logs it's not necessary the
  strict order of mission messages and detection messages
- When forward and backward mission finished you have to interrupt execution
  with Ctrl+C

Running tool:

- ``cd`` to some folder where logs will be saved, e.g. ``cd ~/dev``
- Check the drone is connected: ``ls /dev/ttyACM0``
- Check the executable is present: ``ls /home/olyseus/opt/i_seed/i_seed_drone_onboard``
- Check the model is present: ``ls /var/opt/i_seed_drone_onboard/best.engine``
- Run test with sudo (!):

.. code-block:: none

  > cd ~/dev/
  [~/dev]> sudo /home/olyseus/opt/i_seed/i_seed_drone_onboard

Successful log will look similar to:

.. code-block:: none

  [...] Logging to file: /.../i_seed_drone_onboard.log
  [...] Logging to file (debug): /.../i_seed_drone_onboard_debug.log
  [...]-[DjiCore_Init:101) Payload SDK Version : V3.5.0-beta.0-build.1764
  [...]-[DjiSdkVersionAck_Parse:183) Identify aircraft serial number = 1ZNBJ1G00C00HK, Firmware = 3.4.18.29
  [...]-[DjiAccessAdapter_Init:186) Identify aircraft series is Matrice 300 Series
  [...]-[DjiAccessAdapter_Init:206) Identify mount position type is Extension Port Type
  [...]-[DjiAccessAdapter_Init:301) Identity uart0 baudrate is 921600 bps
  [...]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [...]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [...]-[DjiIdentityVerify_UpdatePolicy:470) Updating dji sdk policy file...
  [...]-[DjiIdentityVerify_UpdatePolicy:473) Update dji sdk policy file successfully
  [...]-[DjiCore_Init:169) Identify AircraftType = Matrice 300 RTK, MountPosition = Extension Port, SdkAdapterType = None
  [...]-[DjiCore_ApplicationStart:239) Start dji sdk application
  [...]-[DjiUser_ApplicationStart:261) Application start.
  [...]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen
  [...]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen
  [...] Loading model from file: /var/opt/i_seed_drone_onboard/best.engine
  [...] Expected model size: batch(30) w(768) h(768)
  [...] batch_w(6) batch_h(5)
  [...] crop_w (288) crop_h (24)
  [...] File read in 50ms
  [...] Init CUDA: CPU +240, GPU +0, now: CPU 306, GPU 2856 (MiB)
  [...] Loaded engine size: 47 MB
  [...] deserializeCudaEngine begin: CPU 306 MiB, GPU 2856 MiB
  [...] Init cuBLAS/cuBLASLt: CPU +167, GPU +174, now: CPU 480, GPU 3036 (MiB)
  [...] Init cuDNN: CPU +250, GPU +249, now: CPU 730, GPU 3285 (MiB)
  [...] Init cuBLAS/cuBLASLt: CPU +0, GPU +0, now: CPU 562, GPU 3285 (MiB)
  [...] deserializeCudaEngine end: CPU 562 MiB, GPU 3285 MiB
  [...] Engine loaded in 4314ms
  [...] ExecutionContext creation begin: CPU 562 MiB, GPU 3285 MiB
  [...] Init cuBLAS/cuBLASLt: CPU +168, GPU +0, now: CPU 730, GPU 3285 (MiB)
  [...] Init cuDNN: CPU +0, GPU +0, now: CPU 730, GPU 3285 (MiB)
  [...] ExecutionContext creation end: CPU 730 MiB, GPU 4326 MiB
  [...] Memory reserved in 108ms
  [...] Downloading file list
  [...] Downloading file list: DONE
  [...]   Name: DJI_20230606125130_0001_WIDE.jpg, index: 262163, time:2023-6-6_12:51:30, size: 4.73 MB
  [...]   Name: DJI_20230606130054_0002_WIDE.jpg, index: 262179, time:2023-6-6_13:0:54, size: 4.73 MB
  [...]   Name: DJI_20230606140144_0001_WIDE.jpg, index: 524307, time:2023-6-6_14:1:44, size: 4.94 MB
  [...]   Name: DJI_20230606140152_0002_WIDE.jpg, index: 524323, time:2023-6-6_14:1:52, size: 4.95 MB
  [...]   Name: DJI_20230606140202_0003_WIDE.jpg, index: 524339, time:2023-6-6_14:2:2, size: 4.95 MB
  [...]   Name: DJI_20230606140228_0004_WIDE.jpg, index: 524355, time:2023-6-6_14:2:28, size: 5.63 MB
  [...]   Name: DJI_20230606140306_0005_ZOOM.mp4, index: 524370, time:2023-6-6_14:3:6, size: 2.50 MB
  [...]   Name: DJI_20230606140308_0005_WIDE.mp4, index: 524371, time:2023-6-6_14:3:8, size: 2.27 MB
  [...]   Name: DJI_20230606140308_0005_SCRN.mp4, index: 524372, time:2023-6-6_14:3:8, size: 2.18 MB
  [...]   Name: DJI_20230606142130_0006_ZOOM.mp4, index: 524386, time:2023-6-6_14:21:30, size: 337.63 MB
  [...]   Name: DJI_20230606142130_0006_WIDE.mp4, index: 524387, time:2023-6-6_14:21:30, size: 337.67 MB
  [...]   Name: DJI_20230606142130_0006_SCRN.mp4, index: 524388, time:2023-6-6_14:21:30, size: 336.67 MB
  [...] Number of JPEG files on SD card: 0
  [...] Protocol version: 7
  [...] Command bytes size: 4
  [...] SIMULATOR MODE
  [...] Creating channel 9745
  [...] Send data job started
  [...] Received data job started
  [...] Mission parameters: lat(48.9), lon(-9.4003) ### <<<<<<<<<< STARTING THE MISSION
  [...] Upload mission and start
  [...] Add waypoint lat(48.9), lon(-9.4002), height(15), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.4001), height(15), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.4), height(15), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.399899999999999), height(15), heading(auto)
  [...] Mission start, ID 1155285849
  [...]-[DjiWaypointV2_UploadMission:339) The mission has no action. ### <<<<<<<<<<<<<< JUST A WARNING, IGNORE IT
  [...] Starting state: mission prepared, waypoint #0
  [...] State: enter mission, waypoint #0
  [...] State: execute flying route mission, waypoint #0
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #0
  [...] Simulating laser range: 13.2
  [...] No laser data available
  [...] Request laser range
  [...] Received laser range: 13.2
  [...] Global waypoint #0
  [...] Bad laser range, waypoint altitude tweak ### <<<<<<<<<<<<< CORRECTING DRONE ALTITUDE
  [...] Mission ABORT
  [...] Upload mission and start ### <<<<<<<<<<< RESTART MISSION WITH TWEAKED ALTITUDE
  [...] Add waypoint lat(48.9), lon(-9.4002), height(16.8), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.4001), height(15), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.4), height(15), heading(auto)
  [...] Add waypoint lat(48.9), lon(-9.399899999999999), height(15), heading(auto)
  [...] Mission start, ID 612212096
  [...]-[DjiWaypointV2_UploadMission:339) The mission has no action.
  [...] Starting state: enter mission, waypoint #0
  [...] State: execute flying route mission, waypoint #0
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #0
  [...] Simulating laser range: 15
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15
  [...] Global waypoint #0
  [...] Current gimbal yaw: 68.998046875, expected: 91.26757169450458
  [...] Run gimbal rotation, yaw: 22.269526, roll: 0, pitch: -90
  [...] drone latitude: 48.90000008461573, longitude: -9.400199375678286, altitude: 116.95804595947266
  [...] drone roll: 0.20377481230854397, pitch: -0.12058387716202575, yaw: 91.27285726345632
  [...] gimbal pitch: -89.9000015258789, roll: 0, yaw: 91.19802856445312
  [...] Gimbal/drone yaw diff: 0.07482869900319145
  [...] Shoot photo request ### <<<<<<<<<<<<<<<<<< SHOOT THE PHOTO
  [...] Resume mission
  [...] State: execute flying route mission, waypoint #0
  [...] 1 files to process
  [...] Download file with index 786450 to /var/opt/i_seed_drone_onboard/2023_6_6_14_44_6.jpg ### <<<<< DOWNLOAD CAMERA SHOT TO ONBOARD
  [...] Inference for image /var/opt/i_seed_drone_onboard/2023_6_6_14_44_6.jpg ### <<<<< RUN THE INFERENCE
  [...] Image uint8 read from disk in 354ms
  [...] Image float conversion and memory layout change in 271ms
  [...] Input data pushed to GPU in 85ms
  [...] State: execute flying route mission, waypoint #1
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #1
  [...] Inference done in 1810ms
  [...] Output data fetched from GPU in 27ms
  [...] Bounding boxes analyzed in 26ms
  [...] x: 4319.2734, y: 290.05927, 56.25% ### <<<<<<<<<<< DETECTED OBJECTS
  [...] Simulating laser range: 15.2
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15.2
  [...] Global waypoint #1
  [...] Current gimbal yaw: 91.19804382324219, expected: 90.01082329121708
  [...] Run gimbal rotation, yaw: -1.1872206, roll: 0, pitch: 0
  [...] drone latitude: 48.90000004147726, longitude: -9.400087324344502, altitude: 115.03295135498047
  [...] drone roll: 0.19998462875433384, pitch: -0.27400771671191, yaw: 90.00788552485503
  [...] gimbal pitch: -89.9000015258789, roll: 0, yaw: 90.0980224609375
  [...] Gimbal/drone yaw diff: 0.09013693608247308
  [...] Shoot photo request
  [...] Resume mission
  [...] State: execute flying route mission, waypoint #1
  [...] 1 files to process
  [...] Download file with index 786466 to /var/opt/i_seed_drone_onboard/2023_6_6_14_44_16.jpg
  [...] Inference for image /var/opt/i_seed_drone_onboard/2023_6_6_14_44_16.jpg
  [...] Image uint8 read from disk in 367ms
  [...] Image float conversion and memory layout change in 223ms
  [...] Input data pushed to GPU in 84ms
  [...] State: execute flying route mission, waypoint #2
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #2
  [...] Inference done in 1752ms
  [...] Output data fetched from GPU in 18ms
  [...] Bounding boxes analyzed in 26ms
  [...] x: 914.8246, y: 77.24172, 46.98% (ignored)
  [...] x: 2572.8508, y: 1313.3638, 60.36%
  [...] Simulating laser range: 14.9
  [...] Laser data is outdated
  [...] Received laser range: 14.9
  [...] Global waypoint #2
  [...] drone latitude: 48.9000000139583, longitude: -9.399986372169224, altitude: 115.09182739257812
  [...] drone roll: 0.2034840804795925, pitch: 0.9204651067917371, yaw: 89.99984960555815
  [...] gimbal pitch: -90, roll: 0, yaw: 90.09803009033203
  [...] Gimbal/drone yaw diff: 0.0981804847738772
  [...] Shoot photo request
  [...] Request laser range
  [...] Resume mission
  [...] State: enter mission after ending pause, waypoint #2
  [...] 1 files to process
  [...] Download file with index 786482 to /var/opt/i_seed_drone_onboard/2023_6_6_14_44_22.jpg
  [...] State: execute flying route mission, waypoint #2
  [...] Inference for image /var/opt/i_seed_drone_onboard/2023_6_6_14_44_22.jpg
  [...] Image uint8 read from disk in 366ms
  [...] Image float conversion and memory layout change in 226ms
  [...] Input data pushed to GPU in 86ms
  [...] Inference done in 1735ms
  [...] Output data fetched from GPU in 17ms
  [...] Bounding boxes analyzed in 26ms
  [...] x: 347.0324, y: 1519.5667, 28.75% (ignored)
  [...] x: 1689.3019, y: 1530.7014, 26.27% (ignored)
  [...] State: execute flying route mission, waypoint #3
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #3
  [...] Simulating laser range: 17.8
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 17.8
  [...] Global waypoint #3
  [...] Bad laser range, waypoint altitude tweak
  [...] Mission ABORT
  [...] Upload mission and start
  [...] Add waypoint lat(48.9), lon(-9.399899999999999), height(12.2), heading(auto)
  [...] Mission start, ID 869673021
  [...]-[DjiWaypointV2_UploadMission:339) The mission has no action.
  [...] Starting state: enter mission, waypoint #0
  [...] State: execute flying route mission, waypoint #0
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #0
  [...] Simulating laser range: 15
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15
  [...] Global waypoint #3
  [...] Current gimbal yaw: 90.09803009033203, expected: -2.573398696386011
  [...] Run gimbal rotation, yaw: -92.67143, roll: 0, pitch: -0.099998474
  [...] drone latitude: 48.899999850315545, longitude: -9.399900425771001, altitude: 112.20651245117188
  [...] drone roll: 0.21356835492805726, pitch: -0.13409798679202156, yaw: -2.4208703615084164
  [...] gimbal pitch: -89.9000015258789, roll: 0, yaw: -2.401970386505127
  [...] Gimbal/drone yaw diff: 0.018899975003289438
  [...] Shoot photo request
  [...] Resume mission
  [...] State: execute flying route mission, waypoint #0
  [...] 1 files to process
  [...] Download file with index 786498 to /var/opt/i_seed_drone_onboard/2023_6_6_14_44_48.jpg
  [...] Inference for image /var/opt/i_seed_drone_onboard/2023_6_6_14_44_48.jpg
  [...] State: execute flying route mission, waypoint #1
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #1
  [...] Image uint8 read from disk in 363ms
  [...] Image float conversion and memory layout change in 226ms
  [...] Input data pushed to GPU in 87ms
  [...] Simulating laser range: 15
  [...] Laser data is outdated
  [...] Request laser range
  [...] Inference done in 1892ms
  [...] Output data fetched from GPU in 17ms
  [...] Bounding boxes analyzed in 26ms
  [...] x: 2421.5225, y: 1599.2177, 41.72% (ignored)
  [...] x: 4132.9336, y: 3742.4365, 37.59% (ignored)
  [...] x: 4522.453, y: 3296.7275, 32.06% (ignored)
  [...] x: 4630.4067, y: 3803.7188, 55.54%
  [...] x: 4566.7456, y: 3668.3633, 47.71% (ignored)
  [...] Received laser range: 15
  [...] Abort mission on a last fake waypoint
  [...] Mission ABORT
  [...] Upload mission and start ### <<<<<<<<<< BACKWARD MISSION START
  [...] Add waypoint lat(48.9), lon(-9.399899999999999), height(12.2), heading(-2.420870)
  [...] Add waypoint lat(48.9), lon(-9.4001), height(15), heading(90.007889)
  [...] Add waypoint lat(48.9), lon(-9.4002), height(16.8), heading(91.272858)
  [...] Mission start, ID 627085210
  [...]-[DjiWaypointV2_UploadMission:339) The mission has no action.
  [...] Starting state: enter mission, waypoint #0
  [...] State: execute flying route mission, waypoint #0
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #0
  [...] Simulating laser range: 15.2
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15.2
  [...] Global waypoint #3
  [...] drone latitude: 48.899999890635755, longitude: -9.399900045011167, altitude: 112.1378402709961
  [...] drone roll: 0.20816132286568914, pitch: -0.17565318385131048, yaw: -0.8769595185508131
  [...] gimbal pitch: -89.9000015258789, roll: 0, yaw: -2.401970148086548
  [...] Gimbal rotate to pixel 4630.4067, 3803.7188
  [...] Run gimbal rotation, yaw: 133.90045, roll: 0, pitch: 29.911827
  [...] drone latitude: 48.8999998790748, longitude: -9.39990004671186, altitude: 112.12994384765625
  [...] drone roll: 0.20016667404211225, pitch: -0.13508057624808895, yaw: -0.9415939403212961
  [...] gimbal pitch: -60.099998474121094, roll: 0, yaw: 131.3980255126953
  ### <<<<<<<<<<<<<<<< HERE ARE VARIOUS ECEF COORDINATES (I-SEED, DRONE GPS, DOWN, HEADING)
  [...] 4144487.6221871953 -686108.07653581 4783340.018965709 255 0 0
  [...] 4144487.2569809523 -686108.033570168 4783340.205455263 0 0 255
  [...] 4144486.6084326454 -686107.9262048501 4783339.451891872 255 255 0
  [...] Simulating laser range: 14.8
  [...] 4144487.5746901655 -686108.0405021395 4783339.963368905 255 0 0
  [...] 4144487.2109368653 -686107.9872966747 4783340.150057722 0 0 255
  [...] 4144486.5623885584 -686107.8799313611 4783339.396494331 255 255 0
  [...] 4144483.329937532 -686101.6298462301 4783326.900126852 255 165 0
  [...] 4144483.2903549355 -686101.4836747702 4783326.955019342 0 255 0
  [...] Resume mission
  [...] State: enter mission after ending pause, waypoint #0
  [...] Received laser range: 14.8
  [...] State: execute flying route mission, waypoint #0
  [...] State: execute flying route mission, waypoint #1
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #1
  [...] Simulating laser range: 15.1
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15.1
  [...] Global waypoint #1
  [...] drone latitude: 48.900000280036295, longitude: -9.400112173705525, altitude: 115.16313171386719
  [...] drone roll: 0.20691746722489643, pitch: 0.06466529660287489, yaw: 89.95095304230645
  [...] gimbal pitch: -60, roll: 0, yaw: 131.39804077148438
  [...] Gimbal rotate to pixel 2572.8508, 1313.3638
  [...] Run gimbal rotation, yaw: -43.186333, roll: 0, pitch: -22.476067
  [...] drone latitude: 48.900000270355946, longitude: -9.400111726820606, altitude: 115.16475677490234
  [...] drone roll: 0.20023240095936187, pitch: -0.03993938766380802, yaw: 89.97992542357235
  [...] gimbal pitch: -82.5, roll: 0, yaw: 88.29805755615234
  [...] 4144487.2013599467 -686121.8966830054 4783342.162841773 255 0 0
  [...] 4144487.200299208 -686121.4915523872 4783342.08623853 0 0 255
  [...] 4144486.551751256 -686121.3841849546 4783341.332675138 255 255 0
  [...] Simulating laser range: 15.5
  [...] 4144486.9756966447 -686123.672853646 4783342.27889805 255 0 0
  [...] 4144486.9756171852 -686123.2674670242 4783342.203653911 0 0 255
  [...] 4144486.32706928 -686123.1600993155 4783341.4500905145 255 255 0
  [...] 4144477.5032756734 -686119.7130181497 4783330.994254306 255 165 0
  [...] 4144477.8017960046 -686118.0231513116 4783330.978107549 0 255 0
  [...] Resume mission
  [...] State: execute flying route mission, waypoint #1
  [...] Received laser range: 15.5
  [...] State: execute flying route mission, waypoint #2
  [...] RUN ACTION FOR WAYPOINT
  [...] Pause mission
  [...] State: pause state, waypoint #2
  [...] Simulating laser range: 15
  [...] Laser data is outdated
  [...] Request laser range
  [...] Received laser range: 15
  [...] Global waypoint #0
  [...] drone latitude: 48.90000006671716, longitude: -9.400213720539538, altitude: 116.97367858886719
  [...] drone roll: 0.1876770455582056, pitch: -0.9386895963111621, yaw: 91.26491219023157
  [...] gimbal pitch: -82.4000015258789, roll: 0, yaw: 88.29804229736328
  [...] Gimbal rotate to pixel 4319.2734, 290.05927
  [...] Run gimbal rotation, yaw: 49.209328, roll: 0, pitch: 18.993534
  [...] drone latitude: 48.90000007430443, longitude: -9.400212497095795, altitude: 116.97802734375
  [...] drone roll: 0.20576199195700345, pitch: 0.05645886641445666, yaw: 91.26861697031433
  [...] gimbal pitch: -63.5, roll: 0, yaw: 137.498046875
  [...] 4144487.104476791 -686130.2080134823 4783343.616676302 255 0 0
  [...] 4144487.1106970543 -686129.8039049936 4783343.535081955 0 0 255
  [...] 4144486.4621493123 -686129.6965362926 4783342.781518561 255 255 0
  [...] Simulating laser range: 15
  [...] 4144486.961156074 -686131.1594370035 4783343.630979715 255 0 0
  [...] 4144486.968209128 -686130.7551518296 4783343.550333871 0 0 255
  [...] 4144486.319661411 -686130.64778298 4783342.796770478 255 255 0
  [...] 4144482.627577942 -686125.4386267465 4783330.212662338 255 165 0
  [...] 4144482.7837846586 -686124.5262437251 4783330.208222109 0 255 0
  [...] Resume mission
  [...] State: enter mission after ending pause, waypoint #2
  [...] Received laser range: 15
  [...] State: execute flying route mission, waypoint #2
  [...] Finish event received
  [...] Updated state: exit mission
  [...] Execute MISSION_FINISHED command ### <<< You can interrupt demo now

Copying file to local machine's desktop (no bounding boxes):

.. code-block:: none

  > scp jetson:/var/opt/i_seed_drone_onboard/2023_6_6_14_44_48.jpg ~/Desktop/

Clean-up:

.. code-block:: none

  [~/dev]> sudo rm -rf ./i_seed_drone_onboard_debug.log ./i_seed_drone_onboard.log ./Logs/
