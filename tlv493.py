from smbus2 import SMBus
import struct
import paho.mqtt.client as mqtt
import time

class TLV493:
    TLV493D_DEFAULT_ADDRESS = 0x5E
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

    def __init__(self,bus,addr = TLV493D_DEFAULT_ADDRESS, addr_reg: int = 0):
        self.read_buffer = [0] * 10
        self.write_buffer = [0] * 4
        self.bus = bus
        self.addr = addr
        self.addr_reg = addr_reg
        self.init_sens()

    def _unpack_and_scale(self,top: int, bottom: int) -> float:
        binval = struct.unpack_from(">h", bytearray([top, bottom]))[0]
        binval = binval >> 4
        return binval * 98.0
    
    def _setup_write_buffer(self) -> None:
            self.read_i2c()
            for key in ["RES1", "RES2", "RES3"]:
                write_value = self._get_read_key(key)
                self._set_write_key(key, write_value)

    def _set_write_key(self, key: str, value: int) -> None:
        write_byte_num, write_mask, write_shift = self.write_masks[key]
        current_write_byte = self.write_buffer[write_byte_num]
        current_write_byte &= ~write_mask
        current_write_byte |= value << write_shift
        self.write_buffer[write_byte_num] = current_write_byte

    def _get_read_key(self, key: str) -> int:
        read_byte_num, read_mask, read_shift = self.read_masks[key]
        raw_read_value = self.read_buffer[read_byte_num]
        write_value = (raw_read_value & read_mask) >> read_shift
        return write_value

    def read_i2c(self):
        try:
            self.read_buffer = self.bus.read_i2c_block_data(self.addr,0,10)
        except:
            time.sleep(2) # какая ошибка?
            self.init_sens()
            pass
    # def write_i2c(self):
    #     try:
    #         self.bus.write_i2c_block_data(self.addr,0x00,self.write_buffer[:-1])
    #     except:
    #         pass
    def _get_read_key(self, key: str) -> int:
            read_byte_num, read_mask, read_shift = self.read_masks[key]
            raw_read_value = self.read_buffer[read_byte_num]
            write_value = (raw_read_value & read_mask) >> read_shift
            return write_value
    def magnetic(self):
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        self.read_i2c()  # update read registers
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
    def init_sens(self):
        self._setup_write_buffer()
        self._set_write_key("ADDR", self.addr_reg)
        self._set_write_key("PARITY", 1)
        self._set_write_key("FAST", 1)
        self._set_write_key("LOWPOWER", 1)
        # self.write_i2c()
        try:
            self.bus.write_i2c_block_data(self.addr,0x00,self.write_buffer[:-1])
            self.bus.write_i2c_block_data(self.addr, 0x00, [5]) # падает на 4 регистре; эксперименты
        except:
            pass


MQTT_BROKER = "localhost"  # Для работы на самом WB
MQTT_TOPIC = "/devices/tlv493d/controls/temperature"
MQTT_CLIENT_ID = "my_python_script"

# Топики для параметров
TOPICS = {
    "x": "/devices/tlv493d/controls/X",
    "y": "/devices/tlv493d/controls/Y",
    "z": "/devices/tlv493d/controls/Z",
    "temperature": "/devices/tlv493d/controls/Temperature"
}

client = mqtt.Client(MQTT_CLIENT_ID)
client.connect(MQTT_BROKER, 1883, 60)

    
if __name__ == "__main__":
    bus = SMBus(3)
    sens = TLV493(bus,TLV493.TLV493D_DEFAULT_ADDRESS)
    # sens = TLV493(bus,0x35)
    fc = 0
    while True:
        x,y,z = sens.magnetic()
        temperature = -255
        print("X - %6.0f µT\t Y - %6.0f µT\t Z - %6.0f µT\t temp - %0.01f\t "%(x,y,z,0))
        for key, topic in TOPICS.items():
                    value = locals()[key]
                    client.publish(topic, value)
        time.sleep(0.5)
