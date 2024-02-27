#include<stdio.h>
#include<stdint.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c-dev.h>

#define MPU6050_ADDR        0x68       // MPU6050 I2C address
#define MPU6050_PWR_MGMT    0x6B       //MPU6050 power management register 

//mpu6050 address to get accelerometer values
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

//mpu6050 address to get gyro values
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

//address of gyroscope configuration register
#define GYRO_CONFIG         0x1B
//address of accelerometer configuration register
#define ACCEL_CONFIG        0x1C


// Different full scale ranges for acc and gyro
#define ACC_FS_SENSITIVITY_0			    16384
#define ACC_FS_SENSITIVITY_1		        8192
#define ACC_FS_SENSITIVITY_2		        4096
#define ACC_FS_SENSITIVITY_3		        2048

#define GYR_FS_SENSITIVITY_0			    131
#define GYR_FS_SENSITIVITY_1			    65.5
#define GYR_FS_SENSITIVITY_2		        32.8
#define GYR_FS_SENSITIVITY_3	            16.4


//global declarations of file_descriptor and switch case variable
int fd;
int choice;


//to configure the registers of MPU6050
int MPU6050_write(uint8_t add,uint8_t data){
    char buf[2];
    buf[0]=add;
    buf[1]=data;
    if(write(fd,buf,sizeof(buf))<0){
        printf("error in the write\n");
        return -1;
    }
    return 0;
}

//user defined function to initialize the mpu6050 
void MPU6050_init()
{
    //to wake the mpu6050 from the sleep mode
    MPU6050_write(MPU6050_PWR_MGMT,0x00);
    usleep(100000);  //sleep for 0.1 sec

    //to configure the mpu6050_accelerometer and mpu6050_gyroscope fullsclae sensitivity   
    printf("enter your choice:\n");
    printf("1.sensitivity 0\n2.sensitivity 1\n3.sensitivity 2\n4.sensitivity 3\n");
    scanf("%d",&choice);
    switch(choice){
        case 1:
            MPU6050_write(ACCEL_CONFIG, 0x00); // For mpu6050_accelerometer sensitivity 0 (±2 g)
            usleep(100000);
            MPU6050_write(GYRO_CONFIG, 0x00); // For mpu6050_gyro sensitivity 0 (±250 º/s)
            usleep(100000);
            break;

        case 2:
            MPU6050_write(ACCEL_CONFIG, 0x08); // For mpu6050_accelerometer sensitivity 1 (±4 g)
            usleep(100000);
            MPU6050_write(GYRO_CONFIG, 0x08); // For mpu6050_gyro sensitivity 1 (±500 º/s)
            usleep(100000);
            break;

        case 3:
            MPU6050_write(ACCEL_CONFIG, 0x10); // For mpu6050_accelerometer sensitivity 2 (±8 g)
            usleep(100000);
            MPU6050_write(GYRO_CONFIG, 0x10); // For mpu6050_gyro sensitivity 2 (±1000 º/s)
            usleep(100000);
            break;

        case 4:
            MPU6050_write(ACCEL_CONFIG, 0x18); // For mpu6050_accelerometer sensitivity 3 (±16 g)
            usleep(100000);
            MPU6050_write(GYRO_CONFIG, 0x18); // For mpu6050_gyro sensitivity 3 (±2000 º/s)
            usleep(100000);
            break;
            
        default:
            printf("enter the correct choice\n");
            break;
    }
}

//MPU6050_read_accel function will retrive the data from the registers of accelerometer
void MPU6050_read_accel(uint8_t baseadd, int16_t *accdata){
    
    char buf;
    buf=baseadd;
    if(write(fd,&buf,1)<0){
        printf("error in the write\n");
    }
    
    int8_t rawdata[6];
    if(read(fd,rawdata,6)<0){
        printf("error in reading the raw data\n");
        return ;
    }

    accdata[0]= (rawdata[0]<<8 | rawdata[1]);
    accdata[1]= (rawdata[2]<<8 | rawdata[3]);
    accdata[2]= (rawdata[4]<<8 | rawdata[5]);
}

//MPU6050_read_accel function will retrive the data from the registers of the gyroscope
void MPU6050_read_gyro(uint8_t baseadd, int16_t *gyrodata){
    
    char buf;
    buf=baseadd;
    if(write(fd,&buf,1)<0){
        printf("error in the write\n");
    }
    
    int8_t rawdata[6];
    if(read(fd,rawdata,6)<0){
        printf("error in reading the raw data\n");
        return ;
    }

    gyrodata[0]= (rawdata[0]<<8 | rawdata[1]);
    gyrodata[1]= (rawdata[2]<<8 | rawdata[3]);
    gyrodata[2]= (rawdata[4]<<8 | rawdata[5]);
}

int main(){
    //to store the of accelerometer and gyro 
    int16_t accdata[3], gyrodata[3];
    double accx,accy,accz,gyrox,gyroy,gyroz;

    //to open the node which is associated with the mpu6050 to communicate
    fd=open("/dev/i2c-2", O_RDWR);
    if(fd<0){
        printf("error in opening the file\n");
        return -1;
    }

    //to set mpu6050 as i2c slave 
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0){
        printf("error in ioctl setting slave\n");
        return -1;
    }

    MPU6050_init();  //initializing the mpu6050

/* 
   --------------------------------------TO FETCH RAW DATA---------------------------------------
   
    //infinite loop to fetch the raw data from the 
    while (1)
    {
        MPU6050_read_accel(ACCEL_XOUT_H, accdata);
        MPU6050_read_gyro(GYRO_XOUT_H, gyrodata);

        printf("Accelerometer: X=%d, Y=%d, Z=%d\n", accdata[0], accdata[1], accdata[2]);
        printf("Gyroscope: X=%d, Y=%d, Z=%d\n", gyrodata[0], gyrodata[1], gyrodata[2]);

        usleep(500000); //sleep for 0.5 sec
        }
*/

   switch(choice){
   
        case 1:
            while(1){
                MPU6050_read_accel(ACCEL_XOUT_H, accdata);
                MPU6050_read_gyro(GYRO_XOUT_H, gyrodata);

                /*Convert acc raw values in to 'g' values*/
                accx = (double) accdata[0]/ACC_FS_SENSITIVITY_0;
                accy = (double) accdata[1]/ACC_FS_SENSITIVITY_0;
                accz = (double) accdata[2]/ACC_FS_SENSITIVITY_0;

                /* Convert gyro raw values in to  "°/s" (deg/seconds) */
                gyrox = (double) gyrodata[0]/GYR_FS_SENSITIVITY_0;
                gyroy = (double) gyrodata[1]/GYR_FS_SENSITIVITY_0;
                gyroz = (double) gyrodata[2]/GYR_FS_SENSITIVITY_0;

                /* print the 'g' and '°/s' values */
                printf("Acc(g)=> X:%.3f Y:%.3f Z:%.3f\n",accx,accy,accz);
                printf("gyro(dps)=> X:%.3f Y:%.3f Z:%.3f\n",gyrox,gyroy,gyroz);
                
                usleep(500000); //sleep for 0.5 sec
            }
            break;
            
        case 2:
            while(1){
                MPU6050_read_accel(ACCEL_XOUT_H, accdata);
                MPU6050_read_gyro(GYRO_XOUT_H, gyrodata);

                /*Convert acc raw values in to 'g' values*/
                accx = (double) accdata[0]/ACC_FS_SENSITIVITY_1;
                accy = (double) accdata[1]/ACC_FS_SENSITIVITY_1;
                accz = (double) accdata[2]/ACC_FS_SENSITIVITY_1;

                /* Convert gyro raw values in to  "°/s" (deg/seconds) */
                gyrox = (double) gyrodata[0]/GYR_FS_SENSITIVITY_1;
                gyroy = (double) gyrodata[1]/GYR_FS_SENSITIVITY_1;
                gyroz = (double) gyrodata[2]/GYR_FS_SENSITIVITY_1;

                /* print the 'g' and '°/s' values */
                printf("Acc(g)=> X:%.3f Y:%.3f Z:%.3f\n",accx,accy,accz);
                printf("gyro(dps)=> X:%.3f Y:%.3f Z:%.3f\n",gyrox,gyroy,gyroz);
                
                usleep(500000); //sleep for 0.5 sec

            }
            break;
            
        case 3:
            while(1){
                MPU6050_read_accel(ACCEL_XOUT_H, accdata);
                MPU6050_read_gyro(GYRO_XOUT_H, gyrodata);

                /*Convert acc raw values in to 'g' values*/
                accx = (double) accdata[0]/ACC_FS_SENSITIVITY_2;
                accy = (double) accdata[1]/ACC_FS_SENSITIVITY_2;
                accz = (double) accdata[2]/ACC_FS_SENSITIVITY_2;

                /* Convert gyro raw values in to  "°/s" (deg/seconds) */
                gyrox = (double) gyrodata[0]/GYR_FS_SENSITIVITY_2;
                gyroy = (double) gyrodata[1]/GYR_FS_SENSITIVITY_2;
                gyroz = (double) gyrodata[2]/GYR_FS_SENSITIVITY_2;

                /* print the 'g' and '°/s' values */
                printf("Acc(g)=> X:%.3f Y:%.3f Z:%.3f\n",accx,accy,accz);
                printf("gyro(dps)=> X:%.3f Y:%.3f Z:%.3f\n",gyrox,gyroy,gyroz);
                
                usleep(500000); //sleep for 0.5 sec

            }
            break;
            
        case 4:
            while(1){
                MPU6050_read_accel(ACCEL_XOUT_H, accdata);
                MPU6050_read_gyro(GYRO_XOUT_H, gyrodata);

                /*Convert acc raw values in to 'g' values*/
                accx = (double) accdata[0]/ACC_FS_SENSITIVITY_3;
                accy = (double) accdata[1]/ACC_FS_SENSITIVITY_3;
                accz = (double) accdata[2]/ACC_FS_SENSITIVITY_3;

                /* Convert gyro raw values in to  "°/s" (deg/seconds) */
                gyrox = (double) gyrodata[0]/GYR_FS_SENSITIVITY_3;
                gyroy = (double) gyrodata[1]/GYR_FS_SENSITIVITY_3;
                gyroz = (double) gyrodata[2]/GYR_FS_SENSITIVITY_3;

                /* print the 'g' and '°/s' values */
                printf("Acc(g)=> X:%.3f Y:%.3f Z:%.3f\n",accx,accy,accz);
                printf("gyro(dps)=> X:%.3f Y:%.3f Z:%.3f\n",gyrox,gyroy,gyroz);
                
                usleep(500000); //sleep for 0.5 sec

            }
            break;
        default:
            break;
   }
   
return 0;
}
