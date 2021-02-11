.. _deploy:

Deploy and run long experiments
===============================

Your algorithm is ready to be used. The project can be build correctly using CMake. Your main program seems to do what you want but your computer is too slow to solve such a difficult problem in acceptable time.
The solution is to deploy SDMS on a server. To this purpose, we provide a Dockerfile to easily deploy SDMS on a server and execute the code.

Procedure
---------

1. Copy the code to the server
2. On the server, go to ``/path/to/sdms``
3. Build the image yourself

.. code-block:: bash

  $ docker build --rm -t sdms:v1.0 .

4. Instanciate a container and run your experiment

.. code-block:: bash

  $ docker run -d sdms:v1.0 SMDStudio solve [ARG...]


.. warning:: The default ``Dockerfile`` build an image containing PyTorch for CPU. You can pass ``LIBTORCH_URL=<path/to/libtorch-xxxxx.zip`` argument to specify a different configuration of PyTorch and use, for instance, pytorch for GPU 10.2.
