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
- There is no interconnection between onboard service and Android control device (simulator without RC configuration)
- Mission and inference are run in parallel. So in logs it's not necessary the
  strict order of mission messages and detection messages

Copying file to local machine's desktop (no bounding boxes):

.. code-block:: none

  > scp jetson:/var/opt/i_seed_drone_onboard/2023_6_6_14_44_48.jpg ~/Desktop/

Clean-up:

.. code-block:: none

  > sudo rm -rf ./i_seed_drone_onboard_debug.log ./i_seed_drone_onboard.log ./Logs/
