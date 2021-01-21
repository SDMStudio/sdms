.. _example:

======================
Command Line Interface
======================

SDM'Studio CLI
===============

By installing SDM'Studio, you are installing four different things (binaries, docs, headers, libraries). By default, the installation folder is ``/usr/local``.

Binaries
-----------

**Binaries** are store in ``/usr/local/bin/``. The main program is called ``SDMStudio``. It can be used for almost everything. 
To see how to use it, just type ``SDMStudio --help`` and you will get an help of this kind.

.. code-block:: bash

    Usage : SDMStudio COMMAND

    The best solver for sequential decision making problems.

    Commands:
      algorithms	Display all available algorithms.
      help			Show this help message.
      solve			Solve a sequential decision making problem using specified algorithm.
      test			Test a policy.
      version		Show the version.
      worlds		Display all available worlds.

    Run 'SDMStudio COMMAND --help' for more information on a command.

The program SDMStudio is simply make alias to other programs. Invoking ``SDMStudio solve`` is the same as invoking ``sdms-solve``. 
To verify that, you can try to run:

.. code-block:: bash

    $ SDMStudio solve --help

and 

.. code-block:: bash

    $ sdms-solve --help

This will display exacly the same thing.

Documentation
--------------
The **documentation** is store in ``/usr/local/share/``.

Librairies
-----------

**Libraries** are store in ``/usr/local/lib/``. 


Headers
-----------

**Header** files are store in ``/usr/local/include/``.

