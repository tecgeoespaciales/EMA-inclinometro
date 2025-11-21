from machine import Pin, I2C
from time import sleep,ticks_us,ticks_ms
from MPU6050 import MPU6050
from hmc5883l import HMC5883L
from qmc5883l import QMC5883L
from math import atan,atan2,cos,sin,pi

# Configura I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=10000)

print(i2c.scan())

TCA9548A_ADDR = 0x70
sensores={}

kalman_pitch0=0
kalman_roll0=0
kalman_yaw0=0
kalman_pitch1=0
kalman_roll1=0
kalman_yaw1=0

comp_roll0=0
comp_pitch0=0
comp_yaw0=0
comp_roll1=0
comp_pitch1=0
comp_yaw1=0

kalman_uncertainty_roll=10
kalman_uncertainty_pitch=10
kalman_uncertainty_yaw=10


gyro_uncertainty_roll=0.5
gyro_uncertainty_pitch=0.5
gyro_uncertainty_yaw=0.5

acc_uncertainty_roll=1.5
acc_uncertainty_pitch=1.5
magna_uncertainty_yaw=1

sampleTime=0.005
timeoffset=0.05


i2c.writeto(TCA9548A_ADDR,b'\x00')

roll0=0
pitch0=0
roll1=0
pitch1=0

def seleccionar_canal(canal):
    if 0 <= canal <= 7:
        i2c.writeto(TCA9548A_ADDR, bytes([1 << canal]))
        #sleep(0.05)  # Pequeña pausa para estabilizar el bus



for canal in [0, 1]:
    seleccionar_canal(canal)
    sleep(0.1)  # Da tiempo al bus I2C a cambiar de canal

    mpu = MPU6050(i2c)
    mpu.activate_low_pass()
    mpu.set_gyro_range(0x08)
    mpu.set_accel_range(0x10)

    sensores[canal] = {
        'mpu': mpu,
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

def complementary_1d(theta,measurement,rate,sampleTime,alphapercent):
    
    alpha=alphapercent/100    
    theta=alpha*(theta+rate*sampleTime)+(1-alpha)*measurement
    
    return theta
    

deltatime=0


start = ticks_us()
count=0

while True:


#if True:
    #canal=0
    try:
        # Inicializa sensores tras seleccionar el canal
            
        acc=[0,0]
        gyro=[0,0]
        for canal in [0,1]:
            mpu = sensores[canal]['mpu']

            acc[canal] = mpu.read_accel_data()
            gyro[canal] = mpu.read_gyro_data()

    
            seleccionar_canal(canal)
            for elem in acc[canal]:
                acc[canal][elem]=acc[canal][elem]-cal[str(canal)]["mpu6050"]["accel_bias"][elem]
                gyro[canal][elem]=gyro[canal][elem]-cal[str(canal)]["mpu6050"]["gyro_bias"][elem]
            
        
        pitch0=atan(acc[0]['z']/(acc[0]['x']**2+acc[0]['y']**2)**0.5)*180/pi
        roll0=-atan(acc[0]['x']/(acc[0]['z']**2+acc[0]['y']**2)**0.5)*180/pi
        pitch1=atan(acc[1]['z']/(acc[1]['x']**2+acc[1]['y']**2)**0.5)*180/pi
        roll1=-atan(acc[1]['x']/(acc[1]['z']**2+acc[1]['y']**2)**0.5)*180/pi   
        end=ticks_us()
        
        deltatime=(end-start)/1000000
        
        

            
            
        
        
        #kalman_roll,kalman_uncertainty_roll=kalman_1d(kalman_roll,kalman_uncertainty_roll,-gyro['z'],gyro_uncertainty_roll,roll,acc_uncertainty_roll,deltatime)
        #kalman_pitch,kalman_uncertainty_pitch=kalman_1d(kalman_pitch,kalman_uncertainty_pitch,gyro['x'],gyro_uncertainty_pitch,pitch,acc_uncertainty_pitch,deltatime)
        comp_roll0=complementary_1d(comp_roll0,roll1,gyro[0]['x'],deltatime,80)
        comp_pitch0=complementary_1d(comp_pitch0,pitch1,-gyro[0]['z'],deltatime,80)
        comp_roll1=complementary_1d(comp_roll1,roll1,gyro[1]['x'],deltatime,80)
        comp_pitch1=complementary_1d(comp_pitch1,pitch1,-gyro[1]['z'],deltatime,80)        
        start=ticks_us()
        #if count>=int(1/sampleTime):
        #   count=0
            #print(f"[CANAL {canal}]")
            #print(f"  Aceleración: X={acc['x']:.2f}g Y={acc['y']:.2f}g Z={acc['z']:.2f}g")
            #print(f"  Giroscopio : X={gyro['x']:.2f}°/s Y={gyro['y']:.2f}°/s Z={gyro['z']:.2f}°/s")
            #print("  Magnetom.  : {}".format(heading))
            #print("Ángulos: {},{},{}".format(roll,pitch,heading))
            #print("Ángulos comp: {},{},{}".format(comp_roll,comp_pitch,comp_yaw))
        

            #print("-" * 50)
            
        if (count>=10):
            count=0
        print("{},{},{},{}\n{},{},{},{}".format(0,round(comp_roll0,2),round(comp_pitch0,2),round(kalman_yaw0,2),1,round(comp_roll1,2),round(comp_pitch1,2),round(kalman_yaw1,2)))
        count+=1
        sleep(sampleTime)
    except Exception as e:
        print(f"[CANAL {canal}] ❌ Error leyendo sensores: {e}")

    
        
