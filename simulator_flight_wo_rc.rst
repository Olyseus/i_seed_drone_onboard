Simulator flight without RC
---------------------------

CMake options:

- `I_SEED_DRONE_ONBOARD_INTERCONNECTION=OFF`
- `I_SEED_DRONE_ONBOARD_SIMULATOR=ON`

CMake configure log:

.. code-block:: none

  ...
  Configuration: Simulator flight without RC
  ...

Build:

.. code-block:: none

  > cd _builds/Release-jetson_gcc/
  > cmake --build .

Starting (with sudo!):

.. code-block:: none

  > sudo ./i_seed_drone_onboard

Logs
----

Logs from Payload SDK:

.. code-block:: none

  [0.013][core]-[Info]-[DjiCore_Init:101) Payload SDK Version : V3.5.0-beta.0-build.1764
  [1.839][linker]-[Error]-[DjiProtocol_sdkUnpack:228) protocol frame crc16 error:0x0000 0x1F09
  [1.970][utils]-[Info]-[DjiSdkVersionAck_Parse:183) Identify aircraft serial number = 1ZNBJ1G00C00HK, Firmware = 3.4.18.29
  [1.972][adapter]-[Info]-[DjiAccessAdapter_Init:186) Identify aircraft series is Matrice 300 Series
  [1.972][adapter]-[Info]-[DjiAccessAdapter_Init:206) Identify mount position type is Extension Port Type
  [2.017][adapter]-[Info]-[DjiAccessAdapter_Init:301) Identity uart0 baudrate is 921600 bps
  [2.017][adapter]-[Info]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [2.063][adapter]-[Info]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [3.203][core]-[Info]-[DjiIdentityVerify_UpdatePolicy:470) Updating dji sdk policy file...
  [4.203][core]-[Info]-[DjiIdentityVerify_UpdatePolicy:473) Update dji sdk policy file successfully
  [4.218][core]-[Info]-[DjiCore_Init:169) Identify AircraftType = Matrice 300 RTK, MountPosition = Extension Port, SdkAdapterType = None
  [4.262][core]-[Info]-[DjiCore_ApplicationStart:239) Start dji sdk application
  [4.262][user]-[Info]-[DjiUser_ApplicationStart:347) Application start.
  [4.989][infor]-[Info]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen
  [4.994][infor]-[Info]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen

Model load:

.. code-block:: none

  [info] Loading model from file: /var/opt/i_seed_drone_onboard/best.engine
  [info] Expected model size: batch(30) w(768) h(768)
  [info] batch_w(6) batch_h(5)
  [info] crop_w (288) crop_h (24)
  [info] File read in 48ms
  ...
  [info] Engine loaded in 4357ms
  ...
  [info] Memory reserved in 106ms

MOP simulation:

.. code-block:: none

  [info] Protocol version: 17
  [info] Packet size: 5
  [info] SIMULATOR MODE
  [info] Creating channel 9745
  [info] Binding channel 9745
  [info] Send data job started
  [info] Received data job started
  [info] simulate send: build_mission_size
  [info] simulate send: build_mission
  [info] simulate send: input_polygon_size
  [info] simulate send: input_polygon_packet

Simulating input mission polygon:

.. code-block:: none

  [info] Input polygon:
  [info]   (48.89991, -9.40039)
  [info]   (48.89991, -9.40021)
  [info]   (48.90009, -9.40021)
  [info]   (48.90009, -9.40039)
  [info] Mission polygon:
  [info]   -6.5986469853800624 -10.008702008717362
  [info]   6.598646985699617 -10.008702008881958
  [info]   6.598623291252753 10.008702007736709
  [info]   -6.598623290923215 10.008702007795268
  ...
  [info] Mission path:
  [info]   (48.89994659410535, -9.400315993092455)
  [info]   (48.900019782247355, -9.400315993115806)
  [info]   (48.90009297038841, -9.400315993139156)
  [info] simulate send: mission_start_size
  [info] simulate send: mission_start
  [info] Init mission with waypoints:
  [info]   (48.89994659410535, -9.400315993092455)
  [info]   (48.900019782247355, -9.400315993115806)
  [info]   (48.90009297038841, -9.400315993139156)
  [info] simulate send: event_id_message_size
  [info] simulate send: event_id_message_packet

Starting mission:

.. code-block:: none

  [info] Upload mission and start
  [info] Add waypoint lat(48.89994659410535), lon(-9.400315993092455), height(15), heading(auto)
  [info] Add waypoint lat(48.900019782247355), lon(-9.400315993115806), height(15), heading(auto)
  [info] Add waypoint lat(48.90009297038841), lon(-9.400315993139156), height(15), heading(auto)
  [info] Mission start, ID 1124648162
  [info] Starting state: mission prepared, waypoint #0
  [info] State: enter mission, waypoint #0
  [info] State: execute flying route mission, waypoint #0

Waypoint reached, making a photo:

.. code-block:: none

  [info] Simulating laser range: 15
  [info] Global waypoint #0
  [info] Current gimbal yaw: 146.6694, expected: 1.7333654
  [info] Run gimbal rotation, yaw: -144.93604, roll: 0, pitch: -0.099998474
  [info] drone latitude: 48.8999466982544, longitude: -9.400315259836793, altitude: 116.96731
  [info] drone roll: 0.18599038, pitch: -0.13077438, yaw: 1.6374434
  [info] gimbal pitch: -89.9, roll: 0, yaw: 1.7693973
  [info] Gimbal/drone yaw diff: 0.13195383548736572
  [info] Shoot photo request

Download file and run inference:

.. code-block:: none

  [info] 1 files to process
  [info] Download file with index 7077906 to /var/opt/i_seed_drone_onboard/2023_11_30_13_11_58.jpg
  [info] Inference for image /var/opt/i_seed_drone_onboard/2023_11_30_13_11_58.jpg
  [info] Image uint8 read from disk in 387ms
  [info] Image float conversion and memory layout change in 284ms
  [info] Input data pushed to GPU in 94ms
  [info] Inference done in 1820ms
  [info] Output data fetched from GPU in 34ms
  [info] Bounding boxes analyzed in 15ms
  [info] x: 647.64465, y: 567.44574, 35.30% (ignored)
  [info] x: 480.96536, y: 117.12923, 52.59%
  [info] x: 665.8585, y: 495.732, 28.57% (ignored)
  [info] x: 1073.6953, y: 506.79623, 34.07% (ignored)
  [info] x: 581.7221, y: 847.8331, 33.21% (ignored)
  [info] x: 402.36877, y: 950.10144, 47.63% (ignored)
  [info] x: 855.79755, y: 1051.1072, 42.53% (ignored)
  [info] x: 512.3196, y: 1177.7269, 50.49%
  [info] x: 1597.854, y: 1454.6038, 25.62% (ignored)
  [info] x: 900.7838, y: 3310.0015, 25.05% (ignored)
  [info] x: 971.52075, y: 3352.2473, 26.45% (ignored)
  [info] x: 1701.634, y: 3161.634, 31.17% (ignored)
  [info] x: 1772.088, y: 3203.903, 29.62% (ignored)
  [info] x: 1260.0502, y: 3295.2087, 42.47% (ignored)
  [info] x: 2437.4556, y: 3185.6045, 38.59% (ignored)
  [info] x: 3384.1282, y: 3721.7207, 46.11% (ignored)

Forward mission is finished, waiting for all the inference to process:

.. code-block:: none

  [info] Wait for inference to finish...
  [info] Wait for inference to finish...
  [info] Wait for inference to finish...

Inference finished, starting backward mission:

.. code-block:: none

  [info] Start backward mission
  [info] Upload mission and start
  [info] Add waypoint lat(48.90009297038841), lon(-9.400315993139156), height(12.200001), heading(4.611417)
  [info] Add waypoint lat(48.900019782247355), lon(-9.400315993115806), height(15), heading(0.003098)
  [info] Add waypoint lat(48.89994659410535), lon(-9.400315993092455), height(16.8), heading(1.637443)
  [info] Mission start, ID 718540239
  [info] Starting state: enter mission, waypoint #0
  [info] State: execute flying route mission, waypoint #0

Reach waypoint, rotate gimbal, distance to the I-Seed object:

.. code-block:: none

  [info] Global waypoint #2
  [info] drone latitude: 48.90009296217657, longitude: -9.400315991699186, altitude: 112.14314
  [info] drone roll: 0.19237866, pitch: -0.15208273, yaw: 5.670135
  [info] gimbal pitch: -89.9, roll: 0, yaw: 4.369397
  [info] Gimbal rotate to pixel 696.5996, 458.12128
  [info] Run gimbal rotation, yaw: -50.604973, roll: 0, pitch: 26.655918
  [info] drone latitude: 48.90009295587983, longitude: -9.400315991699186, altitude: 112.13642
  [info] drone roll: 0.1986443, pitch: -0.1325045, yaw: 5.6519165
  [info] gimbal pitch: -63.3, roll: 0, yaw: -46.130604
  [info] 4144474.9366887677 -686136.8939054642 4783346.807963757 255 0 0
  [info] 4144474.3572377153 -686137.1034183804 4783346.941987365 0 0 255
  [info] 4144473.7086913968 -686136.9960485534 4783346.188422907 255 255 0
  [info] Simulating laser range: 14.8 <<<<<<< shoot laser, distance to the I-Seed object
  [info] 4144474.902474804 -686136.8545271821 4783346.7727670465 255 0 0
  [info] 4144474.3204661063 -686137.0544281228 4783346.910342488 0 0 255
  [info] 4144473.671919788 -686136.9470583012 4783346.156778029 255 255 0
  [info] 4144461.5532191377 -686139.8493327874 4783339.965854141 255 165 0  <<<<< I-Seed object, orange
  [info] 4144461.5871029757 -686140.0254216924 4783339.911602813 0 255 0

Mission finished, since there is no user control, the service is stopped automatically:

.. code-block:: none

  [info] State: enter mission after ending pause, waypoint #2
  [info] State: execute flying route mission, waypoint #2
  [info] Finish event received
  [info] Updated state: exit mission
  [info] MISSION FINISHED
  [critical] Exception: Simulator mission complete
  [critical] User control job exit
  [critical] Inference job exit
