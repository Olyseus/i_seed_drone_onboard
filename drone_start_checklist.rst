Slack
-----

List is pinned to ``i-seed`` channel in Slack:
- https://olyseusinnovations.slack.com/archives/G01JZSKBA4A/p1674701000467569

Updates
-------

It's better to check updates for:

- Matrice 300 RTK drone firmware
- H20 camera firmware
- Smart controller firmware
- DJI Pilot App updates

Make sure not to update to firmware which only support RC PLUS:
- https://enterprise.dji.com/matrice-300/downloads

Camera is ZOOM
--------------

Run to test H20 camera settings:

.. code-block:: none

  [_builds]> sudo ./i_seed_drone_onboard_focal_length

- The camera should be set to ZOOM (check it's not WIDE).
  You can't do it with API. It's an SDK bug
  (https://sdk-forum.dji.net/hc/en-us/requests/73828)

- Zooming should be set to x1.3. The minimum zoom value is x2 when you do it
  with the API. Which is not a minimum focal length
  (https://sdk-forum.dji.net/hc/en-us/requests/73839)

- Smart controller trick to set zoom to x1.3 (you may need second RC):
  https://olyseusinnovations.slack.com/archives/G01JZSKBA4A/p1669281251318429

Simulator
---------

.. note::

  SIMULATOR ONLY

Simulator is on and location is correct:

.. note::

  [_builds]> sudo ./i_seed_drone_onboard_quaternion

Location in the simulator should be set to latitude 48.90, longitude -9.40
(uninhabited area in the Celtic Sea). There is no API call to check if the
drone is running in the simulator. So the onboard service is checking for a
particular coordinate:
- https://sdk-forum.dji.net/hc/en-us/requests/75219

RTK simulation is not supported and should be disabled in RC:
- https://sdk-forum.dji.net/hc/en-us/requests/75454

SDCard
------

Check SDCard can be accessed:

.. note::

  [_builds]> sudo ./i_seed_drone_sdcard_list

Optionally clean-up SDCard:

.. note::

  [_builds]> sudo ./i_seed_drone_sdcard_clean

RTK
---

.. note::

  NOT A SIMULATOR

- A high-precision mobile station should be ON, and RTK should be enabled for
  a high-quality drone coordinate and heading angle

- RTK altitude type should be set to WGS-84 ellipsoidal height. There is no way
  to check it with API (https://sdk-forum.dji.net/hc/en-us/requests/82680)

Smart Controller
----------------

- Disable DJI Pilot App: select Pilot 2 on the settings app page and then go
  to the detailed page to turn it off
  (https://sdk-forum.dji.net/hc/en-us/requests/87678)

.. seealso::

  https://sdk-forum.dji.net/hc/en-us/requests/79714

- RC should be set to "Channel A" (in Mobile SDK v5 there is no API to check
  the mode: https://sdk-forum.dji.net/hc/en-us/requests/84402)

- Second RC must be OFF (interconnection is not working when second RC is ON)
  (FIXME: remove?) https://sdk-forum.dji.net/hc/en-us/requests/87678
