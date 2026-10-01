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


@endpoint('razer.device.power', 'getHapticChargingOverride', out_sig='b')
def get_haptic_charging_override(self):
    """
    Get whether haptics are being kept on while the headset charges

    :return: True when the driver is holding the headset's haptic gate clear
    :rtype: bool
    """
    self.logger.debug("DBus call get_haptic_charging_override")

    driver_path = self.get_driver_path('haptic_charging_override')

    with open(driver_path, 'r') as driver_file:
        return bool(int(driver_file.read().strip()))


@endpoint('razer.device.power', 'setHapticChargingOverride', in_sig='b')
def set_haptic_charging_override(self, enable):
    """
    Keep haptics working while the headset charges

    The headset's firmware disables them whenever the charge cable carries power. This clears
    that, and the driver re-applies it whenever the firmware sets it again. Razer disables them
    on purpose: the earcup gets warm, so don't leave this on unattended.

    :param enable: True to keep haptics on while charging
    :type enable: bool
    """
    self.logger.debug("DBus call set_haptic_charging_override")

    driver_path = self.get_driver_path('haptic_charging_override')

    with open(driver_path, 'w') as driver_file:
        driver_file.write('1' if enable else '0')


@endpoint('razer.device.power', 'getLightingChargingOverride', out_sig='b')
def get_lighting_charging_override(self):
    """
    Get whether the earcup lighting is being kept on while the headset charges

    :return: True when the driver is holding the headset's lighting gate clear
    :rtype: bool
    """
    self.logger.debug("DBus call get_lighting_charging_override")

    driver_path = self.get_driver_path('lighting_charging_override')

    with open(driver_path, 'r') as driver_file:
        return bool(int(driver_file.read().strip()))


@endpoint('razer.device.power', 'setLightingChargingOverride', in_sig='b')
def set_lighting_charging_override(self, enable):
    """
    Keep the earcup lighting on while the headset charges

    The same gate as the haptics one, for the Chroma LEDs.

    :param enable: True to keep the earcup lighting on while charging
    :type enable: bool
    """
    self.logger.debug("DBus call set_lighting_charging_override")

    driver_path = self.get_driver_path('lighting_charging_override')

    with open(driver_path, 'w') as driver_file:
        driver_file.write('1' if enable else '0')
