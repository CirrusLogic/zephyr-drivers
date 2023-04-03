.. _charger_api:

Chargers (Experimental API Stub Doc)
#######################################

The charger subsystem exposes an API to uniformly access battery charger devices. Currently,
only reading data is supported.

Note: This API is currently experimental and this doc will be significantly changed as new features
are added to the API.

Basic Operation
***************

Properties
==========

Fundamentally, a property is a configurable setting, state, or quantity that a charger device can
measure.

Chargers typically support multiple properties, such as temperature readings of the battery-pack
or present-time current/voltage.

Properties are fetched using a client allocated array of :c:struct:`charger_get_property`.  This
array is then populated by values as according to its `property_type` field.

Caching
=======

The Charger API explicitly provides no caching for its clients.


.. _charger_api_reference:

API Reference
*************

.. doxygengroup:: charger_interface
