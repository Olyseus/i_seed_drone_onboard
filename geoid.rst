Install dataset:

.. code-block:: none

  > sudo apt -y install geographiclib-tools
  > sudo geographiclib-get-geoids egm96-5
  > ls /usr/share/GeographicLib/geoids/egm96-5.pgm

Test dataset:

.. code-block:: none

  > ./i_seed_geoid --lat 7.248622 --lon 77.952026 --height 0.0
  ... Altitude above WGS84 reference ellipsoid: -102.09053230365642 (m)

  > ./i_seed_geoid --lat 63.883630938556905 --lon -15.855183623338124 --height 0.0
  ... Altitude above WGS84 reference ellipsoid: 64.34177153733532 (m)

Online calculator:
- https://geographiclib.sourceforge.io/cgi-bin/GeoidEval
