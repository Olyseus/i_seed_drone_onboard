Drone preparation:

- The camera should be set to ZOOM. You can't do it with API. It's an SDK bug
- Zooming should be set to x1.3. The minimum zoom value is x2 when you do it
  with the API. Which is not a minimum focal length
- Smart controller trick to set zoom to x1.3:
  https://olyseusinnovations.slack.com/archives/G01JZSKBA4A/p1669281251318429

Running tool:

- ``cd`` to some folder where logs will be saved, e.g. ``cd ~/dev``
- Check the drone is connected: ``ls /dev/ttyACM0``
- Check the executable is present: ``ls /home/olyseus/opt/i_seed/i_seed_drone_camera_test``
- Check the model is present: ``ls /var/opt/i_seed_drone_onboard/best.engine``
- Run test with sudo (!):

.. code-block:: none

  > cd ~/dev/
  [~/dev]> sudo /home/olyseus/opt/i_seed/i_seed_drone_camera_test

Successful log will look similar to:

.. code-block:: none

  [...] Logging to file: /home/olyseus/dev/i_seed_drone_camera.log
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
  [...] Loading model from file: /var/opt/i_seed_drone_onboard/best.engine ### <<<<<<<<<<< PATH TO MODEL
  [...] Expected model size: batch(30) w(768) h(768)
  [...] batch_w(6) batch_h(5)
  [...] crop_w (288) crop_h (24)
  [...] File read in 48ms
  [...] Init CUDA: CPU +240, GPU +0, now: CPU 306, GPU 4289 (MiB)
  [...] Loaded engine size: 47 MB
  [...] deserializeCudaEngine begin: CPU 306 MiB, GPU 4289 MiB
  [...] Using cublas a tactic source
  [...] Init cuBLAS/cuBLASLt: CPU +167, GPU +170, now: CPU 480, GPU 4464 (MiB)
  [...] Using cuDNN as a tactic source
  [...] Init cuDNN: CPU +250, GPU +250, now: CPU 730, GPU 4714 (MiB)
  [...] Init cuBLAS/cuBLASLt: CPU +0, GPU +0, now: CPU 562, GPU 4714 (MiB)
  [...] Deserialization required 2911436 microseconds.
  [...] deserializeCudaEngine end: CPU 562 MiB, GPU 4714 MiB
  [...] Engine loaded in 4198ms
  [...] ExecutionContext creation begin: CPU 562 MiB, GPU 4714 MiB
  [...] Using cublas a tactic source
  [...] Init cuBLAS/cuBLASLt: CPU +168, GPU +2, now: CPU 730, GPU 4716 (MiB)
  [...] Using cuDNN as a tactic source
  [...] Init cuDNN: CPU +0, GPU +0, now: CPU 730, GPU 4716 (MiB)
  [...] Total per-runner device memory is 45088256
  [...] Total per-runner host memory is 144736
  [...] Allocated activation device memory of size 1751778304
  [...] ExecutionContext creation end: CPU 731 MiB, GPU 5694 MiB
  [...] Memory reserved in 99ms
  [...] Downloading file list
  [...] Downloading file list: DONE
  [...]   Name: DJI_20230606125130_0001_WIDE.jpg, index: 262163, time:2023-6-6_12:51:30, size: 4.73 MB ### <<<<< OLD SDCARD FILE
  [...] Number of JPEG files on SD card: 0
  [...] Shoot photo ### <<<<<<<<<<<<<<<<<<<< START SHOOTING
  [...] Shoot photo request
  [...] focal length min: 317, max: 55620, step: 10
  [...] Focal length: 317
  [...] Call DjiCameraManager_StartShootPhoto
  [...] DjiCameraManager_StartShootPhoto OK
  [...] Shoot photo: DONE
  [...] Check SD card
  [...] Downloading file list
  [...] Downloading file list: DONE
  [...]   Name: DJI_20230606125130_0001_WIDE.jpg, index: 262163, time:2023-6-6_12:51:30, size: 4.73 MB
  [...] No files to process
  [...] Check SD card
  [...] Downloading file list
  [...] Downloading file list: DONE
  [...]   Name: DJI_20230606125130_0001_WIDE.jpg, index: 262163, time:2023-6-6_12:51:30, size: 4.73 MB
  [...]   Name: DJI_20230606130054_0002_ZOOM.jpg, index: 262178, time:2023-6-6_13:0:54, size: 6.64 MB ### <<<<<<<< NEW SDCARD FILE
  [...]   Name: DJI_20230606130054_0002_WIDE.jpg, index: 262179, time:2023-6-6_13:0:54, size: 4.73 MB
  [...] 1 files to process
  [...] Download file with index 262178 to /var/opt/i_seed_drone_onboard/2023_6_6_13_0_54.jpg ### <<<<<<< PHOTO COPIED TO ONBOARD
  [...] Inference for image /var/opt/i_seed_drone_onboard/2023_6_6_13_0_54.jpg
  [...] Image uint8 read from disk in 348ms
  [...] Image float conversion and memory layout change in 271ms
  [...] Input data pushed to GPU in 88ms
  [...] Inference done in 1842ms
  [...] Output data fetched from GPU in 37ms
  [...] Bounding boxes analyzed in 25ms
  [...] x: 1435.2429, y: 696.1149, 41.76% (ignored)
  [...] x: 3368.4302, y: 65.0607, 43.13% (ignored)
  [...] x: 1992.0355, y: 1527.3975, 79.73%
  [...] x: 2097.2378, y: 1796.1035, 30.14% (ignored)
  [...] x: 2037.392, y: 1823.3967, 66.79%
  [...] x: 3270.7603, y: 1893.8331, 31.89% (ignored)
  [...] x: 4428.219, y: 2224.6626, 49.10% (ignored)
  [...] Save bounding boxes to image: /var/opt/i_seed_drone_onboard/bbox_2023_6_6_13_0_54.jpg ### <<<<<<<< IMAGE WITH BBOXES
  [...] Deinit camera
  [...] Deinit camera: DONE
  [...] Init cuBLAS/cuBLASLt: CPU +0, GPU +0, now: CPU 518, GPU 5899 (MiB)

Copying file to local machine's desktop:

.. code-block:: none

  > scp jetson:/var/opt/i_seed_drone_onboard/bbox_2023_6_6_13_0_54.jpg ~/Desktop/

Clean-up:

.. code-block:: none

  [~/dev]> sudo rm -rf ./i_seed_drone_camera.log ./Logs/
