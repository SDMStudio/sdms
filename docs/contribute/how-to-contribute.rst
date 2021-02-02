How to contribute
==================

There different ways to contribute to SDMS. In the most cases, contributions will consist in implementing new algorithms based on data structures already availables.
However, there are lot af ways to contribute to SDMS. For instance, we may want to use already existing algorithms with new representation for its components (value function, occupancy states, problem). 
All these components can have many different representations and this is why, contributions can also take the form of giving acces to new ways to represent these values.

Contribute with your new algorithm
-----------------------------------

You have a new algorithm that can solve at least one class of decision making problem and you want to test it and make it available on SDMS platform ? You are in the right place.
To contribute to SDMS with your newly imagine algorithm you need to answer some questions:

*1. Does your algorithm new in terms of classical algorithm steps ?*

- your algorithm does not follow similar steps as algorithms returned by command ``SDMStudio algorithms`` 

*2. Does your algorithm new in term type of problems that it solves ?*

- the type of problem is not in the list returned by command ``SDMStudio worlds`` 

*3. Does your algorithm new in term of data structures involved ?*


.. code-block:: bash

    $ SDMStudio solve --help

**other**