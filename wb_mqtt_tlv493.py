#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import logging
import os
import signal
import struct
import sys
import threading
from ast import literal_eval
from typing import Tuple

from smbus import SMBus
from wb_common.mqtt_client import MQTTClient

CONFIG = {"poll_interval_s": 0.5, "driver_name": "wb-mqtt-tlv493"}

EXIT_INVALIDARGUMENT = 2
EXIT_NOTCONFIGURED = 6
EXIT_NOTRUNNING = 7

DEFAULT_CONFIG_PATH = "/var/lib/wb-mqtt-tlv493/conf.d/wb-mqtt-tlv493.conf"
MQTT_CLEANUP_TIMEOUT_S = 1


class TLV493:
    """
    A copy-paste from https://github.com/adafruit/Adafruit_CircuitPython_TLV493D/blob/main/adafruit_tlv493d.py
    adopted to generic python (instead of adafruit's internals)
    """

    read_masks = {
        "BX1": (0, 0xFF, 0),
        "BX2": (4, 0xF0, 4),
        "BY1": (1, 0xFF, 0),
        "BY2": (4, 0x0F, 0),
        "BZ1": (2, 0xFF, 0),
        "BZ2": (5, 0x0F, 0),
        "TEMP1": (3, 0xF0, 4),
        "TEMP2": (6, 0xFF, 0),
        "FRAMECOUNTER": (3, 0x0C, 2),
        "CHANNEL": (3, 0x03, 0),
        "POWERDOWNFLAG": (5, 0x10, 4),
        "RES1": (7, 0x18, 3),
        "RES2": (8, 0xFF, 0),
        "RES3": (9, 0x1F, 0),
    }

    write_masks = {
        "PARITY": (1, 0x80, 7),
        "ADDR": (1, 0x60, 5),
        "INT": (1, 0x04, 2),
        "FAST": (1, 0x02, 1),
        "LOWPOWER": (1, 0x01, 0),
        "TEMP_DISABLE": (3, 0x80, 7),
        "LP_PERIOD": (3, 0x40, 6),
        "POWERDOWN": (3, 0x20, 5),
        "RES1": (1, 0x18, 3),
        "RES2": (2, 0xFF, 0),
        "RES3": (3, 0x1F, 0),
    }

    def __init__(self, bus, addr=0x5E, addr_reg: int = 0):
        self.read_buffer = [0] * 10
        self.write_buffer = [0] * 4
        self.bus = bus
        self.addr = addr
        self.addr_reg = addr_reg

        # read in data from sensor, including data that must be set on a write
        self._setup_write_buffer()

        # write correct i2c address
        self._set_write_key("ADDR", addr_reg)

        # setup MASTERCONTROLLEDMODE which takes a measurement for every read
        self._set_write_key("PARITY", 1)
        self._set_write_key("FAST", 1)
        self._set_write_key("LOWPOWER", 1)
        self._write_i2c()

    def _read_i2c(self):
        self.read_buffer = self.bus.read_i2c_block_data(self.addr, 0, len(self.read_buffer))

    def _write_i2c(self) -> None:
        """
        @Magistrdev's heuristic
        """
        val = self.write_buffer[1]
        self.bus.write_i2c_block_data(self.addr, 0, self.write_buffer[:-1])
        self.bus.write_i2c_block_data(self.addr, 0, [val])

    def _setup_write_buffer(self) -> None:
        self._read_i2c()
        for key in ["RES1", "RES2", "RES3"]:
            write_value = self._get_read_key(key)
            self._set_write_key(key, write_value)

    def _get_read_key(self, key: str) -> int:
        read_byte_num, read_mask, read_shift = self.read_masks[key]
        raw_read_value = self.read_buffer[read_byte_num]
        write_value = (raw_read_value & read_mask) >> read_shift
        return write_value

    def _set_write_key(self, key: str, value: int) -> None:
        write_byte_num, write_mask, write_shift = self.write_masks[key]
        current_write_byte = self.write_buffer[write_byte_num]
        current_write_byte &= ~write_mask
        current_write_byte |= value << write_shift
        self.write_buffer[write_byte_num] = current_write_byte

    @property
    def magnetic(self) -> Tuple[float, float, float]:
        """
        Return the processed magnetometer sensor values.
        """
        return self.read_magnetic()

    def read_magnetic(self) -> Tuple[float, float, float]:
        """
        Read the processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        self._read_i2c()  # update read registers
        x_top = self._get_read_key("BX1")
        x_bot = (self._get_read_key("BX2") << 4) & 0xFF
        y_top = self._get_read_key("BY1")
        y_bot = (self._get_read_key("BY2") << 4) & 0xFF
        z_top = self._get_read_key("BZ1")
        z_bot = (self._get_read_key("BZ2") << 4) & 0xFF

        return (
            self._unpack_and_scale(x_top, x_bot),
            self._unpack_and_scale(y_top, y_bot),
            self._unpack_and_scale(z_top, z_bot),
        )

    @staticmethod
    def _unpack_and_scale(top: int, bottom: int) -> float:
        binval = struct.unpack_from(">h", bytearray([top, bottom]))[0]
        binval = binval >> 4
        return binval * 98.0


class VirtualDevice:
    DEVICE_META = {
        "driver": CONFIG["driver_name"],
        "title": {"en": "Magnetic field sensor", "ru": "Датчик напряженности магнитного поля"},
    }

    CONTROL_META = {
        "type": "value",
        "order": 10,
        "readonly": True,
        "units": "%",
        "title": {
            "en": "Magnetic field strength",
            "ru": "Напряженность поля",
        },
    }

    def __init__(self, mqtt_client, bus_number, on_auth_error):
        self._was_disconnected = True
        self._on_auth_error = on_auth_error
        self._val = "0"
        self._err = "r"

        self.base_topic = f"/devices/tlv493_{bus_number}"
        self.control_topic = f"/devices/tlv493_{bus_number}/controls/field_strength_percent"
        self.mqtt_client = mqtt_client
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect

    def _on_connect(self, _client, _userdata, _flags, result_code):
        if result_code != 0:
            logging.error("Mosquitto connection refused, result code %s", result_code)
            if result_code in (4, 5):
                self._on_auth_error()
            return
        logging.info("Mosquitto was connected")
        if self._was_disconnected:
            self.create()
            self.publish_value(self._val)
            self.publish_error(self._err, force=True)
            self._was_disconnected = False

    def _on_disconnect(self, _client, _userdata, _result_code):
        self._was_disconnected = True
        logging.warning("Mosquitto was disconnected")

    def _publish_meta(self, device_meta, control_meta):
        self.mqtt_client.publish(f"{self.base_topic}/meta", device_meta, retain=True)
        self.mqtt_client.publish(f"{self.control_topic}/meta", control_meta, retain=True)

    def create(self):
        self._publish_meta(json.dumps(self.DEVICE_META), json.dumps(self.CONTROL_META))

    def delete(self):
        publish_results = [
            self.mqtt_client.publish(self.control_topic, "", qos=1, retain=True),
            self.mqtt_client.publish(f"{self.control_topic}/meta/error", "", qos=1, retain=True),
            self.mqtt_client.publish(f"{self.control_topic}/meta", "", qos=1, retain=True),
            self.mqtt_client.publish(f"{self.base_topic}/meta", "", qos=1, retain=True),
        ]
        success = True
        for result in publish_results:
            if result.rc != 0:
                success = False
                continue
            try:
                result.wait_for_publish(timeout=MQTT_CLEANUP_TIMEOUT_S)
                success = result.is_published() and success
            except RuntimeError:
                success = False
        return success

    def publish_value(self, val):
        self._val = val
        self.mqtt_client.publish(self.control_topic, str(val), retain=True)

    def publish_error(self, val="r", force=False):
        val = "r" if val else ""
        if val == self._err and not force:
            return
        self._err = val
        self.mqtt_client.publish(f"{self.control_topic}/meta/error", val, retain=True)


class ConfigValidationError(Exception):
    pass


class TLV493Driver:
    MAX_MEASUREMENTS_BOUND_UT = 130000  # datasheet says +- 130mT for tlv493

    def __init__(self, config_fname):
        if not os.path.exists(config_fname):
            logging.info("No config file at %s found! Treating service as not running", config_fname)
            sys.exit(EXIT_NOTRUNNING)

        try:
            self.bus_num = self.get_valid_bus_number(config_fname)  # filled by wb-hwconf-manager
        except ConfigValidationError as error:
            logging.error("Invalid config file %s: %s", config_fname, error)
            sys.exit(EXIT_NOTCONFIGURED)

        self.bus = SMBus(self.bus_num)

        self.stop_event = threading.Event()
        self.exit_code = EXIT_NOTRUNNING

        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self.handle_stop)

        self.mqtt_client = MQTTClient(CONFIG["driver_name"])
        self.virtual_device = VirtualDevice(self.mqtt_client, self.bus_num, self.handle_auth_error)
        self.mqtt_client.will_set(f"{self.virtual_device.control_topic}/meta/error", "r", retain=True)
        self.mqtt_client.start()

    def handle_stop(self, _signum, _frame):
        self.exit_code = EXIT_NOTRUNNING
        self.stop_event.set()

    def handle_auth_error(self):
        self.exit_code = EXIT_INVALIDARGUMENT
        self.stop_event.set()

    def search_i2c_device(self):
        """
        TLV493 has different i2c addr, depending on SDA voltage => we have only 1 device on bus
        """
        for addr in range(128):
            try:
                self.bus.read_i2c_block_data(addr, 0, 10)
                logging.info("Found alive device at %x", addr)
                return addr
            except OSError:
                pass
        raise RuntimeError("No devices found")

    def run(self):
        virtual_device = self.virtual_device
        logging.info("Initting device")
        while not self.stop_event.is_set():
            try:
                addr = self.search_i2c_device()
                sens = TLV493(self.bus, addr)
            except (OSError, RuntimeError):
                virtual_device.publish_error()
                self.stop_event.wait(CONFIG["poll_interval_s"])
                continue

            virtual_device.publish_error(None)
            logging.info("Start polling device via i2c w %.2fs period", CONFIG["poll_interval_s"])

            while not self.stop_event.is_set():
                try:
                    x, y, z = sens.magnetic
                    # cb8ea449 want just percents of magnetic field strength
                    result_ut = max(map(abs, [x, y, z]))
                    result = f"{result_ut / self.MAX_MEASUREMENTS_BOUND_UT * 100.0:.2f}"
                    virtual_device.publish_value(result)
                except OSError:
                    logging.exception("Failed data read. Will reinit device")
                    virtual_device.publish_error()
                    break
                self.stop_event.wait(CONFIG["poll_interval_s"])

        if self.exit_code == EXIT_INVALIDARGUMENT:
            self.mqtt_client.stop()
            return self.exit_code

        if not virtual_device.delete():
            logging.error("Failed to clear retained MQTT topics: broker is unavailable")
        self.mqtt_client.stop()
        return self.exit_code

    def get_valid_bus_number(self, config_fname):
        try:
            with open(config_fname, encoding="utf-8") as conffile:
                config_dict = literal_eval(conffile.read())
                if not isinstance(config_dict, dict):
                    raise TypeError("top-level config value must be a mapping")
                CONFIG.update(config_dict)
        except (SyntaxError, TypeError, ValueError) as error:
            raise ConfigValidationError(str(error)) from error

        bus_num = CONFIG.get("bus_num", None)
        if bus_num is None:
            raise ConfigValidationError(f"Bus number is not specified in {config_fname}")
        if isinstance(bus_num, bool) or not isinstance(bus_num, int) or bus_num < 0:
            raise ConfigValidationError("Bus number must be a non-negative integer")

        poll_interval = CONFIG.get("poll_interval_s")
        if (
            isinstance(poll_interval, bool)
            or not isinstance(poll_interval, (int, float))
            or poll_interval <= 0
        ):
            raise ConfigValidationError("Poll interval must be a positive number")

        try:
            SMBus(bus_num)
        except (OSError, TypeError) as error:
            raise ConfigValidationError(str(error)) from error
        return bus_num


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s %(message)s", level=logging.INFO)
    parser = argparse.ArgumentParser(description="Wiren Board TLV493 magnetic field sensor driver")
    parser.add_argument("legacy_config", nargs="?", help=argparse.SUPPRESS)
    parser.add_argument("-c", "--config", help="Path to configuration file")
    arguments = parser.parse_args()
    if arguments.config and arguments.legacy_config:
        parser.error("configuration path must be specified only once")
    sys.exit(TLV493Driver(arguments.config or arguments.legacy_config or DEFAULT_CONFIG_PATH).run())
