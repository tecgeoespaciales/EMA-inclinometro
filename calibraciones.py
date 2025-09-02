import json
from MPU6050 import MPU6050
from hmc5883l import HMC5883L
from qmc5883l import QMC5883L 
from machine import I2C, Pin
import time

# Inicializar I2C (ajustar los pines a tu placa)
i2c = I2C(scl=Pin(22), sda=Pin(21))  # GPIO5 = D1, GPIO4 = D2 (ESP8266)





TCA9548A_ADDR = 0x70
sensores={}
def seleccionar_canal(canal):
    if 0 <= canal <= 7:
        i2c.writeto(TCA9548A_ADDR, bytes([1 << canal]))
        time.sleep(0.05)  # Pequeña pausa para estabilizar el bus
seleccionar_canal(0)
address=30
for canal in [0, 1]:
    seleccionar_canal(canal)
    time.sleep(0.1)  # Da tiempo al bus I2C a cambiar de canal

    mpu = MPU6050(i2c)
    mpu.activate_low_pass()
    mpu.set_gyro_range(0x08)
    mpu.set_accel_range(0x10)
    if 13 in i2c.scan():
        hmc = QMC5883L(i2c)
    if 30 in i2c.scan():
       hmc = HMC5883L(i2c)
    

    sensores[canal] = {
        'mpu': mpu,
        'hmc': hmc
    }


print(sensores)

# Calibración del MPU6050
def calibrate_mpu6050(mpu,samples=100):

    print("Calibrando MPU6050...")
    acc_bias = {'x': 0, 'y': 0, 'z': 0}
    gyro_bias = {'x': 0, 'y': 0, 'z': 0}

    for _ in range(samples):
        acc = mpu.read_accel_data()
        gyro = mpu.read_gyro_data()
        for axis in acc_bias:
            
            acc_bias[axis] += acc[axis]
            if axis == 'z':
                acc_bias[axis]-=10
            gyro_bias[axis] += gyro[axis]
        time.sleep(0.01)
    acc_bias ={axis: round(acc_bias[axis] / samples,2) for axis in acc_bias} 
    gyro_bias = {axis: round(gyro_bias[axis] / samples,2) for axis in gyro_bias}

    return {"accel_bias": acc_bias, "gyro_bias": gyro_bias}

# Calibración del HMC5883L
def calibrate_hmc5883l(hmc,samples=200):
    print("Calibrando HMC5883L...")
    min_values = [float('inf')] * 3
    max_values = [float('-inf')] * 3

    for _ in range(samples):
        mag = hmc.read()
        for i in range(3):
            min_values[i] = min(min_values[i], mag[i])
            max_values[i] = max(max_values[i], mag[i])
        time.sleep(0.05)

    offset = [(max_values[i] + min_values[i]) / 2 for i in range(3)]
    offset = [round(x, 2) for x in offset]

    return {"mag_offset": offset}

# Ejecutar calibración

calibs={}

for canal in [0,1]:
    seleccionar_canal(canal)

#if True:
    #canal=0
    try:
        # Inicializa sensores tras seleccionar el canal
        mpu = sensores[canal]['mpu']
        hmc = sensores[canal]['hmc']



        mpu_data = calibrate_mpu6050(mpu)
        hmc_data = calibrate_hmc5883l(hmc)
        
        

        # Guardar en archivo JSON
        calibration_data = {
            "mpu6050": mpu_data,
            "hmc5883l": hmc_data
        }
        
        calibs[canal]=calibration_data
        
    except Exception as e:
            print(f"[CANAL {canal}] ❌ Error leyendo sensores: {e}")

with open("calibration.json", "w") as f:
    f.write(json.dumps(calibs))

print("Calibración guardada en calibration.json")