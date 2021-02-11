.. _install:

Installation
============

Quick install
-------------
In order to execute ``install.sh`` file, you may need to change permissions using ``chmod +x install.sh``.


.. code-block:: bash

  $ git clone https://gitlab.inria.fr/chroma1/plasma/sdms.git
  $ cd sdms
  $ ./install.sh


Step by step installation
--------------------------
**Install SDMS dependencies**

.. code-block:: bash

  $ sudo apt-get install clang libeigen3-dev libboost-all-dev

**Install pytorch**

Download the last version of PyTorch C++ for cxx11 ABI according to your machine requirements (https://pytorch.org/get-started/locally/) and unzip the downloaded file into `/opt/` directory.


.. code-block:: bash

  $ wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-xxxxxxx.zip -O libtorch.zip
  $ unzip libtorch.zip -d /opt

**Build and install SDM'Studio**

.. code-block:: bash

  $ mkdir -p build && cd build
  $ cmake .. -DCMAKE_PREFIX_PATH=/opt/libtorch
  $ make install


Docker Image
--------------

**Using pre-built images**

You can also pull a pre-built docker image from Docker Hub and run with docker

.. code-block:: bash

  $ docker run --rm -ti blavad/sdms:latest


**Building the image yourself**

The ``Dockerfile`` is supplied to build images including the CPU version of PyTorch. You can pass ``LIBTORCH_URL=<path/to/libtorch-xxxxx.zip`` argument to specify which configuration of PyTorch is to be used.

.. code-block:: bash

  $ docker build --rm -t sdms:v1.0 .
  $ docker run --rm -ti sdms:v1.0
