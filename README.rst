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
