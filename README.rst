Install service:

.. code-block:: none

  [i_seed_drone_onboard]> sudo ln -s "`pwd`/i_seed_drone_onboard.service" /etc/systemd/system/
  [i_seed_drone_onboard]> sudo systemctl enable i_seed_drone_onboard

Service
-------

Start service:

.. code-block:: none

  > sudo systemctl start i_seed_drone_onboard

Stop service:

.. code-block:: none

  > sudo systemctl stop i_seed_drone_onboard

Logs
----

Check logs:

.. code-block:: none

  > sudo journalctl -u i_seed_drone_onboard -f

Or

.. code-block:: none

  > tail -f ~/.i_seed_drone_onboard/i_seed_drone_onboard.log

Removing logs:

.. code-block:: none

  > sudo rm -rf /var/log/journal/ /run/log/journal/ ~/.i_seed_drone_onboard/i_seed_drone_onboard.log

Used files
----------

- Model for inference: ``/var/opt/i_seed_drone_onboard/best.engine``
- Images downloaded from SDCard saved to ``/var/opt/i_seed_drone_onboard``

DEB
---

Content of ``*.deb`` file:

.. code-block:: none

  > dpkg --contents i_seed_drone_onboard-0.0.3-Linux.deb

Install ``*.deb`` package:

.. code-block:: none

  > sudo apt install -y ./i_seed_drone_onboard-0.0.3-Linux.deb

Remove installed package:

.. code-block:: none

  > sudo apt remove -y i_seed_drone_onboard

Check dependencies of the installed executable:

.. code-block:: none

  > lddtree /usr/bin/i_seed_drone_onboard

UART configuration
------------------

Direct config:

.. code-block:: none

  #define LINUX_UART_DEV1    "/dev/ttyS0"
  #define LINUX_UART_DEV2    "/dev/ttyACM0"

USB-serial config:

.. code-block:: none

  #define LINUX_UART_DEV1    "/dev/ttyUSB0"
  #define LINUX_UART_DEV2    "/dev/ttyACM0"

- ``third_party/psdk/manifold2/hal/hal_uart.h``
- ``samples/sample_c/platform/linux/manifold2/hal/hal_uart.h`` (OSDK sample)
