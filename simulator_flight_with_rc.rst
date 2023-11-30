Simulator flight with RC
------------------------

CMake options:

- `I_SEED_DRONE_ONBOARD_INTERCONNECTION=ON`
- `I_SEED_DRONE_ONBOARD_SIMULATOR=ON`

CMake configure log:

.. code-block:: none

  ...
  Configuration: Simulator flight with connected RC
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

  [0.017][core]-[Info]-[DjiCore_Init:101) Payload SDK Version : V3.5.0-beta.0-build.1764
  [1.912][utils]-[Info]-[DjiSdkVersionAck_Parse:183) Identify aircraft serial number = 1ZNBJ1G00C00HK, Firmware = 3.4.18.29
  [1.916][adapter]-[Info]-[DjiAccessAdapter_Init:186) Identify aircraft series is Matrice 300 Series
  [1.916][adapter]-[Info]-[DjiAccessAdapter_Init:206) Identify mount position type is Extension Port Type
  [1.954][adapter]-[Info]-[DjiAccessAdapter_Init:301) Identity uart0 baudrate is 921600 bps
  [1.954][adapter]-[Info]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [1.996][adapter]-[Info]-[DjiPayloadNegotiate_Init:144) No need negotiate device info
  [3.138][core]-[Info]-[DjiIdentityVerify_UpdatePolicy:470) Updating dji sdk policy file...
  [4.138][core]-[Info]-[DjiIdentityVerify_UpdatePolicy:473) Update dji sdk policy file successfully
  [4.151][core]-[Info]-[DjiCore_Init:169) Identify AircraftType = Matrice 300 RTK, MountPosition = Extension Port, SdkAdapterType = None
  [4.177][core]-[Info]-[DjiCore_ApplicationStart:239) Start dji sdk application
  [4.177][user]-[Info]-[DjiUser_ApplicationStart:347) Application start.
  [4.880][infor]-[Info]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen
  [4.900][infor]-[Info]-[DjiAircraftInfo_NotifyMobileAppInfoHandle:589) Set mobile app info, language is English, screen type is Big Screen

Model load:

.. code-block:: none

  [info] Loading model from file: /var/opt/i_seed_drone_onboard/best.engine
  [info] Expected model size: batch(30) w(768) h(768)
  [info] batch_w(6) batch_h(5)
  [info] crop_w (288) crop_h (24)
  [info] File read in 199ms
  ...
  [info] Engine loaded in 6489ms
  ...
  [info] Memory reserved in 103ms

Create MOP channel and wait for the connection:

.. code-block:: none

  [info] Protocol version: 17
  [info] Packet size: 5
  [info] SIMULATOR MODE
  [info] Creating channel 9745
  [info] Binding channel 9745
  [info] Accept channel (create out)

Android control app connected:

.. code-block:: none

  [info] Received data job started
  [info] Send data job started
  [info] PONG command sent

Polygon input from user:

.. code-block:: none

  [info] Input polygon:
  [info]   (48.900319963108345, -9.399967789649963)
  [info]   (48.89981920994735, -9.400344975292683)
  [info]   (48.89990582826053, -9.399409219622612)
  [info] Mission polygon:
  [info]   -4.432889037944617 33.914158638585484
  [info]   -32.087593455271474 -21.773408873212304
  [info]   36.520482493181596 -12.140749769624634

Mission path based on polygon:

.. code-block:: none

  [info] Mission path:
  [info]   (48.9002396226164, -9.400021095360465)
  [info]   (48.90018498521597, -9.399947234405275)
  [info]   (48.90013034776773, -9.399873373611094)
  [info]   (48.90007571027166, -9.399799512977927)
  [info]   (48.90002107272776, -9.399725652505767)
  [info]   (48.899966435136044, -9.399651792194623)
  [info]   (48.89991179749649, -9.399577932044483)
  [info]   (48.8999015079767, -9.39976228990093)
  [info]   (48.89995614549827, -9.399836150236851)
  [info]   (48.900010782972025, -9.39991001073378)
  [info]   (48.90006542039794, -9.399983871391719)
  [info]   (48.900120057776036, -9.400057732210671)
  [info]   (48.900174695106315, -9.400131593190633)
  [info]   (48.90005513022949, -9.400168229729822)
  [info]   (48.90000049292155, -9.400094368886101)
  [info]   (48.899945855565775, -9.40002050820339)
  [info]   (48.89989121816219, -9.39994664768169)
  [info]   (48.899836580710755, -9.399872787321)
  [info]   (48.899826290719474, -9.400057144840291)
  [info]   (48.89988092805293, -9.400131005386763)
  [info]   (48.899935565338545, -9.400204866094242)
  [info]   (48.89987063764894, -9.400315363016144)
  [info]   (48.899816000433475, -9.400241502283894)

Starting mission:

.. code-block:: none

  [info] Upload mission and start
  [info] Mission start, ID 1143140351
  [info] Starting state: mission prepared, waypoint #0
  [info] State: enter mission, waypoint #0
  [info] State: execute flying route mission, waypoint #0

Waypoint reached, making a photo:

.. code-block:: none

  [info] Global waypoint #0
  [info] Current gimbal yaw: 144.18846, expected: 134.40599
  [info] Run gimbal rotation, yaw: -9.782471, roll: 0, pitch: -0.30000305
  [info] drone latitude: 48.90024140340233, longitude: -9.400020895544454, altitude: 115.09652
  [info] drone roll: 0.18499354, pitch: -0.09315652, yaw: 134.4108
  [info] gimbal pitch: -90, roll: 0, yaw: 134.48846
  [info] Gimbal/drone yaw diff: 0.077667236328125
  [info] Shoot photo request

Download file and run inference:

.. code-block:: none

  [info] 1 files to process
  [info] Download file with index 7340210 to /var/opt/i_seed_drone_onboard/2023_11_30_13_44_4.jpg
  [info] Inference for image /var/opt/i_seed_drone_onboard/2023_11_30_13_44_4.jpg
  [info] Image uint8 read from disk in 435ms
  [info] Image float conversion and memory layout change in 223ms
  [info] Input data pushed to GPU in 85ms
  [info] Inference done in 1830ms
  [info] Output data fetched from GPU in 22ms
  [info] Bounding boxes analyzed in 16ms
  [info] x: 376.82428, y: 3091.608, 41.64% (ignored)
  [info] x: 2125.427, y: 2638.5825, 26.81% (ignored)
  [info] x: 1998.457, y: 2834.838, 42.86% (ignored)
  [info] x: 2792.003, y: 2853.2373, 26.81% (ignored)
  [info] x: 3199.6213, y: 3014.6736, 53.92%
  [info] x: 4442.485, y: 3059.7637, 25.26% (ignored)
  [info] x: 4597.024, y: 3820.5874, 34.61% (ignored)

Forward mission is finished, waiting for all the inference to process:

.. code-block:: none

  [info] Wait for inference to finish...
  [info] Wait for inference to finish...
  [info] Wait for inference to finish...

Inference finished, starting backward mission:

.. code-block:: none

  [info] Start backward mission
  [info] Upload mission and start
  [info] Add waypoint lat(48.899816000433475), lon(-9.400241502283894), height(15), heading(138.521805)
  [info] Add waypoint lat(48.89987063764894), lon(-9.400315363016144), height(15), heading(-131.714752)
  [info] Add waypoint lat(48.899935565338545), lon(-9.400204866094242), height(15), heading(-41.719097)
  [info] Add waypoint lat(48.89988092805293), lon(-9.400131005386763), height(15), heading(-41.702759)
  [info] Add waypoint lat(48.899826290719474), lon(-9.400057144840291), height(15), heading(-94.865227)
  [info] Add waypoint lat(48.899836580710755), lon(-9.399872787321), height(15), heading(138.466614)
  [info] Add waypoint lat(48.900174695106315), lon(-9.400131593190633), height(15), heading(-41.682186)
  [info] Add waypoint lat(48.900120057776036), lon(-9.400057732210671), height(15), heading(-41.627243)
  [info] Add waypoint lat(48.90006542039794), lon(-9.399983871391719), height(15), heading(-41.627640)
  [info] Mission start, ID 1313187649
  [info] Starting state: enter mission, waypoint #0
  [info] State: execute flying route mission, waypoint #0

Reach waypoint, rotate gimbal, distance to the I-Seed object:

.. code-block:: none

  [info] Global waypoint #22
  [info] drone latitude: 48.89981639646487, longitude: -9.40024174477663, altitude: 115.105865
  [info] drone roll: 0.2558278, pitch: -0.1078126, yaw: 142.47864
  [info] gimbal pitch: -90, roll: 0, yaw: 138.38844
  [info] Gimbal rotate to pixel 1614.9684, 1387.5408
  [info] Run gimbal rotation, yaw: -56.246536, roll: 0, pitch: 13.245018
  [info] drone latitude: 48.89981644108181, longitude: -9.400241756296996, altitude: 115.09593
  [info] drone roll: 0.20695455, pitch: -0.13972406, yaw: 142.66435
  [info] gimbal pitch: -76.8, roll: 0, yaw: 82.18845
  [info] 4144501.2684849207 -686135.0265767183 4783328.274535572 255 0 0
  [info] 4144501.288519578 -686134.4585019374 4783328.001568231 0 0 255
  [info] 4144500.639969424 -686134.3511324497 4783327.248007025 255 255 0
  [info] 4144500.572598611 -686135.5873259451 4783328.787881096 255 0 0
  [info] 4144500.6223078337 -686135.0342929859 4783328.489041587 0 0 255
  [info] 4144499.9737577867 -686134.9269234104 4783327.735480301 255 255 0
  [info] 4144491.3221501047 -686130.034338309 4783317.835050814 255 165 0  <<<<< I-Seed object, orange
  [info] 4144491.7687684167 -686129.4883242454 4783317.5284678815 0 255 0

Current mission finished, user can start new one:

.. code-block:: none

  [info] State: execute flying route mission, waypoint #8
  [info] State: end of waypoint mission, waypoint #8
  [info] Finish event received
  [info] Updated state: exit mission
  [info] MISSION FINISHED
