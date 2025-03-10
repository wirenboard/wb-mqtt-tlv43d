import json
import logging
import struct
import sys
import time
from ast import literal_eval
from typing import Tuple

from smbus import SMBus
from wb_common.mqtt_client import MQTTClient

CONFIG = {
    "poll_interval_s": 0.5,
    "driver_name": "wb-mqtt-tlv493"
}

EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_NOTCONFIGURED = 6
EXIT_NOTRUNNING = 7


class TLV493:
    """A copy-paste from https://github.com/adafruit/Adafruit_CircuitPython_TLV493D/blob/main/adafruit_tlv493d.py
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

    def __init__(self, bus, addr = 0x5e, addr_reg: int = 0):
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
        """@Magistrdev's heuristic
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
        """The processed magnetometer sensor values.
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
        "title": {
            "en": "Magnetic field sensor",
            "ru": "Датчик напряженности магнитного поля"
        }
    }

    CONTROL_META = {
        "type": "value",
        "order": 10,
        "readonly": True,
        "title": {
            "en": "Magnetic field strength",
            "ru": "Напряженность поля",
        }
    }

    def __init__(self, mqtt_client, bus_number):
        self.base_topic = f"/devices/tlv493_{bus_number}"
        self.control_topic = f"/devices/tlv493_{bus_number}/controls/field_strength"
        self.mqtt_client = mqtt_client
        self.create()

    def _publish_meta(self, device_meta, control_meta):
        self.mqtt_client.publish(f"{self.base_topic}/meta", device_meta, retain=True)
        self.mqtt_client.publish(f"{self.control_topic}/meta", control_meta, retain=True)

    def create(self):
        self._publish_meta(json.dumps(self.DEVICE_META), json.dumps(self.CONTROL_META))
        self.publish_error()

    def delete(self):
        self._publish_meta(None, None)

    def publish_value(self, val):
        self.mqtt_client.publish(self.control_topic, str(val), retain=True)

    def publish_error(self, val="r"):
        val = "r" if val else ""
        self.mqtt_client.publish(f"{self.control_topic}/meta/error", val, retain=True)


def search_i2c_device(bus):
    for device in range(128):
        try:
            bus.read_i2c_block_data(device, 0, 10)
            logging.info("Found alive device at %x", device)
            return device
        except Exception:
            pass
    raise RuntimeError("No devices found")


def update_config(config_fname):
    try:
        with open(config_fname, encoding="utf-8") as conffile:
            config_dict = literal_eval(conffile.read())
            CONFIG.update(config_dict)
            return EXIT_SUCCESS
    except IOError:
        logging.info("No config file at %s found! Treating service as not running", config_fname)
        return EXIT_NOTRUNNING
    except (SyntaxError, ValueError):
        logging.exception("Error in config file %s", config_fname)
        return EXIT_NOTCONFIGURED


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s %(message)s", level=logging.DEBUG)

    ec = update_config("/etc/wb-mqtt-tlv493.conf")
    if ec != EXIT_SUCCESS:
        sys.exit(ec)

    mqtt_client = MQTTClient(CONFIG["driver_name"])
    mqtt_client.start()

    bus_num = CONFIG.get("bus_num", None)
    if bus_num is None:
        logging.warning("Bus number is not specified")
        sys.exit(EXIT_NOTCONFIGURED)
    bus = SMBus(bus_num)

    try:
        virtual_device = VirtualDevice(mqtt_client, bus_num)
        addr = search_i2c_device(bus)
        sens = TLV493(bus, addr)
        virtual_device.publish_error(None)
        while True:
            x,y,z = sens.magnetic
            result_ut = max(map(abs, [x, y, z]))
            result = "%.2f" % (result_ut / 130000.0 * 100.0)
            virtual_device.publish_value(result)
            time.sleep(CONFIG["poll_interval_s"])
    except Exception as e:
        if not isinstance(e, SystemExit):
            logging.exception("Unhandled exception:")
            ec = EXIT_FAILURE
        virtual_device.publish_error()
    finally:
        virtual_device.delete()
        mqtt_client.stop()
        sys.exit(ec)
