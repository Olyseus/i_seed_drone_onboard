Installing TensorRT libraries and headers on Ubuntu x64:

```
> cd ~/opt/
[opt]> gsutil -m cp gs://olyseus_bucket/dependencies/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3.tar.gz .
[opt]> tar xfv TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3.tar.gz
[opt]> export LD_LIBRARY_PATH=~/opt/TensorRT-8.4.0.6/lib
```

You can use `trtexec` tool to verify that installed stuff is working. Load ONNX format model (do not use `~/`!):

```
> ./TensorRT-8.4.0.6/bin/trtexec --onnx=$HOME/Downloads/1st_dataset_weights/1st_dataset_weights/weights/best.onnx
```

Load TensorRT format model:

```
> ./TensorRT-8.4.0.6/bin/trtexec --loadEngine=$HOME/Downloads/1st_dataset_weights/1st_dataset_weights/weights/best.engine
```
