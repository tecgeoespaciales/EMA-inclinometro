from machine import Pin, I2C
from time import sleep
from MPU6050 import MPU6050
from hmc5883l import HMC5883L
from qmc5883l import QMC5883L
from math import atan,atan2,cos,sin,pi

# Configura I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=10000)

TCA9548A_ADDR = 0x70
sensores={}

kalman_pitch=0
kalman_roll=0
kalman_yaw=0

kalman_uncertainty_roll=0
kalman_uncertainty_pitch=0
kalman_uncertainty_yaw=0


gyro_uncertainty_roll=4
gyro_uncertainty_pitch=4
gyro_uncertainty_yaw=4

acc_uncertainty_roll=3
acc_uncertainty_pitch=3
magna_uncertainty_yaw=4

timeoffset=0.005
sampleTime=0.05-timeoffset


i2c.writeto(TCA9548A_ADDR,b'\x00')

def seleccionar_canal(canal):
    if 0 <= canal <= 7:
        i2c.writeto(TCA9548A_ADDR, bytes([1 << canal]))
        #sleep(0.05)  # Pequeña pausa para estabilizar el bus



for canal in [0, 1]:
    seleccionar_canal(canal)
    sleep(0.1)  # Da tiempo al bus I2C a cambiar de canal

    mpu = MPU6050(i2c)
    if 13 in i2c.scan():
        hmc = QMC5883L(i2c)
    if 30 in i2c.scan():
       hmc = HMC5883L(i2c)
    

    sensores[canal] = {
        'mpu': mpu,
        'hmc': hmc
    }


'''
import json

def read_calibration_file(filename="calibration.json"):
    try:
        with open(filename, "r") as f:
            data = json.load(f)
            return data
    except Exception as e:
        print("Error leyendo archivo:", e)
        return None

# Ejemplo de uso
cal = read_calibration_file()
if cal:
    print("Calibración MPU6050:")
    print("  Acelerómetro:", cal["mpu6050"]["accel_bias"])
    print("  Giroscopio:", cal["mpu6050"]["gyro_bias"])
    print("Calibración HMC5883L:")
    print("  Offset magnético:", cal["hmc5883l"]["mag_offset"])
else:
    print("No se pudo leer la calibración.")'''


def kalman_1d(KalmanState,KalmanUncertainty,rateInput,rateUncertainty,noiseInput,noiseUncertainty,sampleTime):
    KalmanState=KalmanState+sampleTime*rateInput
    KalmanUncertainty=KalmanUncertainty+(sampleTime*rateUncertainty)**2
    KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+noiseUncertainty**2)
    KalmanState=KalmanState+KalmanGain*(noiseInput-KalmanState)
    KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty
    
    return KalmanState,KalmanUncertainty



count=0

while True:
    for canal in [0,1]:
        seleccionar_canal(canal)
    
    #if True:
        #canal=0
        try:
            # Inicializa sensores tras seleccionar el canal
            mpu = sensores[canal]['mpu']
            hmc = sensores[canal]['hmc']

            acc = mpu.read_accel_data()
            gyro = mpu.read_gyro_data()
            mag = hmc.read()
            roll=atan2(acc['y'],(acc['x']**2+acc['z']**2)**0.5)*180/pi
            pitch=atan2(acc['x'],(acc['y']**2+acc['z']**2)**0.5)*180/pi
            heading=hmc.heading(mag[0],mag[1])
            
            kalman_roll,kalman_uncertainty_roll=kalman_1d(kalman_roll,kalman_uncertainty_roll,gyro['x'],gyro_uncertainty_roll,roll,acc_uncertainty_roll,sampleTime)
            kalman_pitch,kalman_uncertainty_pitch=kalman_1d(kalman_pitch,kalman_uncertainty_pitch,gyro['y'],gyro_uncertainty_pitch,pitch,acc_uncertainty_pitch,sampleTime)
            kalman_yaw,kalman_uncertainty_yaw=kalman_1d(kalman_yaw,kalman_uncertainty_yaw,gyro['z'],gyro_uncertainty_yaw,heading[0],magna_uncertainty_yaw,sampleTime)
                                    
            if count>=int(1/sampleTime):
                count=0
                print(f"[CANAL {canal}]")
                print(f"  Aceleración: X={acc['x']:.2f}g Y={acc['y']:.2f}g Z={acc['z']:.2f}g")
                print(f"  Giroscopio : X={gyro['x']:.2f}°/s Y={gyro['y']:.2f}°/s Z={gyro['z']:.2f}°/s")
                print("  Magnetom.  : {}".format(heading))
                print("Ángulos: {},{},{}".format(kalman_roll,kalman_pitch,kalman_yaw))
                print("-" * 50)

            count+=1

        except Exception as e:
            print(f"[CANAL {canal}] ❌ Error leyendo sensores: {e}")

        sleep(sampleTime)