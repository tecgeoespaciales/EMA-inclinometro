from machine import I2C
import time
import math

class QMC5883L:
    
    __gain__ = {
        '0.88': (0 << 5, 0.73),
        '1.3':  (1 << 5, 0.92),
        '1.9':  (2 << 5, 1.22),
        '2.5':  (3 << 5, 1.52),
        '4.0':  (4 << 5, 2.27),
        '4.7':  (5 << 5, 2.56),
        '5.6':  (6 << 5, 3.03),
        '8.1':  (7 << 5, 4.35)
    }
    
    def __init__(self, i2c, address=0x0D,declination=(0, 0),gauss='1.3'):
        self.i2c = i2c
        self.address = address
        reg_value, self.gain = self.__gain__[gauss]
        self.init_sensor()
        
        # Convert declination (tuple of degrees and minutes) to radians.
        self.declination = (declination[0] + declination[1] / 60) * math.pi / 180
        
    def init_sensor(self):
        # Control Register 2: Soft reset
        self.i2c.writeto_mem(self.address, 0x0A, bytes([0x00]))
        time.sleep_ms(10)
        # Set continuous measurement mode, 200Hz, 8G range, 16-bit output
        self.i2c.writeto_mem(self.address, 0x09, bytes([0x1D]))  # 0b00011101
        # Set roll pointer
        self.i2c.writeto_mem(self.address, 0x0B, bytes([0x01]))
        self.i2c.writeto_mem(self.address, 0x20, bytes([0x40]))
        self.i2c.writeto_mem(self.address, 0x21, bytes([0x01]))

    def read(self):
        gain=self.gain
        data = self.i2c.readfrom_mem(self.address, 0x00, 6)
        
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]

        
        x = x - (1 << 16) if x & (1 << 15) else x
        y = y - (1 << 16) if y & (1 << 15) else y
        z = z - (1 << 16) if z & (1 << 15) else z

        x = round(x * gain, 4)
        y = round(y * gain, 4)
        z = round(z * gain, 4)
        return round(x,4), round(y,4),round(z,4)
    
    
    def heading(self, x, y):
        heading_rad = math.atan2(y, x)
        heading_rad += self.declination

        # Correct reverse heading.
        if heading_rad < 0:
            heading_rad += 2 * math.pi

        # Compensate for wrapping.
        elif heading_rad > 2 * math.pi:
            heading_rad -= 2 * math.pi

        # Convert from radians to degrees.
        heading = heading_rad * 180 / math.pi
        degrees = math.floor(heading)
        minutes = round((heading - degrees) * 60)
        return degrees, minutes

    def format_result(self, x, y, z):
        degrees, minutes = self.heading(x, y)
        return 'X: {:.4f}, Y: {:.4f}, Z: {:.4f}, Heading: {}° {}′ '.format(x, y, z, degrees, minutes)

