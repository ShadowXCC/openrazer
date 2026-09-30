# SPDX-License-Identifier: GPL-2.0-or-later

"""
Headset settings: sidetone and haptics (e.g. Razer Kraken V3 Pro)
"""
from openrazer_daemon.dbus_services import endpoint


@endpoint('razer.device.audio', 'getSidetone', out_sig='i')
def get_sidetone(self):
    """
    Get the sidetone level

    The driver only knows it once it has been set (the headset has no way to read it back).

    :return: 0 (off) to 100, or -1 if unknown
    :rtype: int
    """
    self.logger.debug("DBus call get_sidetone")

    driver_path = self.get_driver_path('sidetone')

    with open(driver_path, 'r') as driver_file:
        value = int(driver_file.read().strip())

    # The driver reports 0 (off) or the headset's level 0-4 as 1-5
    if value <= 0:
        return value
    return value * 20


@endpoint('razer.device.audio', 'setSidetone', in_sig='y')
def set_sidetone(self, sidetone):
    """
    Set the sidetone level

    :param sidetone: 0 (off) to 100. The headset has 5 levels: 1-20, 21-40, 41-60, 61-80, 81-100.
    :type sidetone: int
    """
    self.logger.debug("DBus call set_sidetone")

    driver_path = self.get_driver_path('sidetone')

    sidetone = max(0, min(100, int(sidetone)))
    value = 0 if sidetone == 0 else (sidetone - 1) // 20 + 1

    with open(driver_path, 'w') as driver_file:
        driver_file.write(str(value))


@endpoint('razer.device.haptics', 'getHapticIntensity', out_sig='i')
def get_haptic_intensity(self):
    """
    Get the haptic feedback intensity

    Known once set, or once changed with the headset's button.

    :return: 0 (off), 1 (low), 2 (medium), 3 (high), or -1 if unknown
    :rtype: int
    """
    self.logger.debug("DBus call get_haptic_intensity")

    driver_path = self.get_driver_path('haptic_intensity')

    with open(driver_path, 'r') as driver_file:
        return int(driver_file.read().strip())


@endpoint('razer.device.haptics', 'setHapticIntensity', in_sig='y')
def set_haptic_intensity(self, intensity):
    """
    Set the haptic feedback intensity

    :param intensity: 0 (off), 1 (low), 2 (medium), 3 (high)
    :type intensity: int
    """
    self.logger.debug("DBus call set_haptic_intensity")

    driver_path = self.get_driver_path('haptic_intensity')

    intensity = max(0, min(3, int(intensity)))

    with open(driver_path, 'w') as driver_file:
        driver_file.write(str(intensity))
