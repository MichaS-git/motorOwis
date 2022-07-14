# motorOwis
# UNDER CONSTRUCTION
EPICS motor drivers for the following [Owis](https://www.owis.eu/en/products/article/PS-10-32-SM/0) controller: PS 10

motorOwis is a submodule of [motor](https://github.com/epics-modules/motor).  When motorOwis is built in the ``motor/modules`` directory, no manual configuration is needed.

motorOwis can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorOwis contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
