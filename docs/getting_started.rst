###############
Getting started
###############


************
Introduction
************

The :code:`pymeasure` package uses an object oriented approach for communicating with instruments, which provides a more intuitive interface through the encapsulation of low level SCPI commands. That way you can focus on solving the measurement problems at hand, instead of re-inventing how to communicate with them. 

Instruments with VISA (GPIB, Serial, etc) are supported through the `PyVISA package`_ under the hood. `Prologix GPIB`_ adapters are also supported.

.. _PyVISA package: http://pyvisa.readthedocs.org/en/master/
.. _Prologix GPIB: http://prologix.biz/

Before using PyMeasure, you should be acquainted with `basic Python programming for the sciences`_ and understand the concept of objects.

.. _basic Python programming for the sciences: https://scipy-lectures.github.io/


****************
Instrument ready
****************

In the :code:`pymeasure` package you will find a number of instruments already defined. Their definitions are organized based on the manufacturer name of the instrument. For example the class that defines the Keithley 2400 SourceMeter can be imported by calling:

.. code-block:: python

  from pymeasure.instruments.keithley import Keithley2400


If the instrument you are planning to use is not already in the repository, follow the instructions on :doc:`adding an instrument <contribute/adding_instruments>`.

******************
Graphical displays
******************

Graphical displays can be easily developed to manage the execution of measurement procedures.