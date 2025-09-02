from machine import Pin, I2C
from time import sleep,ticks_us
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

comp_roll=0
comp_pitch=0
comp_yaw=0

kalman_uncertainty_roll=0
kalman_uncertainty_pitch=0
kalman_uncertainty_yaw=0


gyro_uncertainty_roll=4
gyro_uncertainty_pitch=4
gyro_uncertainty_yaw=4

acc_uncertainty_roll=3
acc_uncertainty_pitch=3
magna_uncertainty_yaw=1

sampleTime=0.05
timeoffset=0.05


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
    
    for elem in cal:
    
        print("Calibración MPU6050:")
        print("  Acelerómetro:", cal[elem]["mpu6050"]["accel_bias"])
        print("  Giroscopio:", cal[elem]["mpu6050"]["gyro_bias"])
else:
    print("No se pudo leer la calibración.")


def kalman_1d(KalmanState,KalmanUncertainty,rateInput,rateUncertainty,noiseInput,noiseUncertainty,sampleTime):
    KalmanStateval=KalmanState+sampleTime*rateInput
    KalmanUncertaintyval=KalmanUncertainty+(sampleTime*rateUncertainty)**2
    KalmanGain=KalmanUncertaintyval*1/(1*KalmanUncertaintyval+noiseUncertainty**2)
    KalmanStateval=KalmanStateval+KalmanGain*(noiseInput-KalmanStateval)
    KalmanUncertaintyval=(1-KalmanGain)*KalmanUncertaintyval
    
    return KalmanStateval,KalmanUncertaintyval

def complementary_1d(theta,measurement,rate,sampleTime,alpha):
    
    gain=alpha/100    
    theta=gain*(theta+rate*sampleTime)+(1-gain/100)*measurement
    
    return theta
    
    


count=0
start=0
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
            
            for elem in acc:
                acc[elem]=acc[elem]-cal[str(canal)]["mpu6050"]["accel_bias"][elem]
                gyro[elem]=gyro[elem]-cal[str(canal)]["mpu6050"]["gyro_bias"][elem]
                
                
            roll=atan(acc['y']/(acc['x']**2+acc['z']**2)**0.5)*180/pi
            pitch=-atan(acc['x']/(acc['y']**2+acc['z']**2)**0.5)*180/pi
            heading=hmc.heading(mag[0],mag[1])
            end=ticks_us() 
            if start!= 0:
                kalman_roll,kalman_uncertainty_roll=kalman_1d(kalman_roll,kalman_uncertainty_roll,gyro['x'],gyro_uncertainty_roll,roll,acc_uncertainty_roll,(end-start)/1000000)
                kalman_pitch,kalman_uncertainty_pitch=kalman_1d(kalman_pitch,kalman_uncertainty_pitch,gyro['y'],gyro_uncertainty_pitch,pitch,acc_uncertainty_pitch,(end-start)/1000000)
                kalman_yaw,kalman_uncertainty_yaw=kalman_1d(kalman_yaw,kalman_uncertainty_yaw,gyro['z'],gyro_uncertainty_yaw,heading[0],magna_uncertainty_yaw,(end-start)/1000000)
                comp_roll=complementary_1d(comp_roll,roll,gyro['x'],(end-start)/1000000,95)
                comp_pitch=complementary_1d(comp_pitch,pitch,gyro['y'],(end-start)/1000000,95)
                comp_yaw=complementary_1d(comp_yaw,heading[0],gyro['z'],(end-start)/1000000,95)
            start=ticks_us()                      
            if count>=int(1/sampleTime):
                count=0
                #print(f"[CANAL {canal}]")
                #print(f"  Aceleración: X={acc['x']:.2f}g Y={acc['y']:.2f}g Z={acc['z']:.2f}g")
                #print(f"  Giroscopio : X={gyro['x']:.2f}°/s Y={gyro['y']:.2f}°/s Z={gyro['z']:.2f}°/s")
                #print("  Magnetom.  : {}".format(heading))
                #print("Ángulos: {},{},{}".format(roll,pitch,heading))
                #print("Ángulos comp: {},{},{}".format(comp_roll,comp_pitch,comp_yaw))
            

                #print("-" * 50)
                
                
            print("{},{},{},{}".format(canal,round(kalman_roll,2),round(kalman_pitch,2),round(kalman_yaw,2)))
            count+=1
            sleep(sampleTime)
            sleep(1)
        except Exception as e:
            print(f"[CANAL {canal}] ❌ Error leyendo sensores: {e}")

        
        