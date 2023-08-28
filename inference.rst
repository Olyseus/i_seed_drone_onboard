virtualenv (Ubuntu x64)
-----------------------

Create and activate ``virtualenv`` with Python 3.7 (see tests https://github.com/Olyseus/i_seed_drone_onboard/issues/3):

.. code-block:: none

  > conda create --name yolo_v5 python=3.7
  > conda activate yolo_v5

Install dependencies

.. code-block:: none

  (yolo_v5)> pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/v6.1/requirements.txt

Protobuf pip package from ``yolo_v5`` virtualenv needs to be downgraded:

.. code-block:: none

  > pip uninstall protobuf
  > pip install protobuf==3.20

ONNX (Ubuntu x64)
-----------------

Convert ``best.pt`` (variation YOLOv5s) to ONNX format (yolov5, git tag v6.2):

.. code-block:: none

  (yolo_v5)> cd yolov5
  (yolo_v5)[yolov5]> python ./export.py --weights ~/Downloads/1st_dataset_weights/1st_dataset_weights/weights/best.pt --include onnx --img-size 768 768 --device 0 --batch-size 30

.. code-block:: none

  > ls -lah ~/Downloads/1st_dataset_weights/1st_dataset_weights/weights/best.onnx
  ... 28M ... ~/Downloads/1st_dataset_weights/1st_dataset_weights/weights/best.onnx

Tensor input parameters image size 768x768 and batch 30 are best:

- https://github.com/Olyseus/i_seed_drone_onboard/issues/4#issuecomment-1217364250
- https://github.com/Olyseus/i_seed_drone_onboard/blob/6155b3816d81f75c6b1126bd1c1ab37db6cb017b/inference.h#L28-L29

TensorRT (Ubuntu arm64)
-----------------------

Convert ONNX weights to TensorRT format:

.. code-block:: none

  > ./i_seed_convert --onnx best.onnx --engine best.engine

Use TensorRT weights in inference:

.. code-block:: none

  > ./i_seed_inference --model best.engine --image ~/Downloads/2023_3_24_10_36_22.jpg

Onboard service model location: ``/var/opt/i_seed_drone_onboard/best.engine``
Onboard service images from mission location: ``/var/opt/i_seed_drone_onboard``

Run inference and save bounding boxes to image:

.. code-block:: none

  > ./i_seed_inference --model best.engine --image ~/Downloads/2023_3_24_10_36_22.jpg --bbimage ~/Downloads/out.jpg

Python script to visualize the bounding box:
- https://gist.github.com/ruslo/8cd75a146878f8dd2284e3948c205e6a
