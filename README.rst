Used files
----------

- Model for inference: ``/var/opt/i_seed_drone_onboard/best.engine``
- Images downloaded from SDCard saved to ``/var/opt/i_seed_drone_onboard``

UART configuration
------------------

Latest recommended config:

.. code-block:: none

  #define LINUX_UART_DEV1    "/dev/ttyTHS2"
  #define LINUX_UART_DEV2    "/dev/ttyACM0"

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
