//=========================================================
//Inverted Pendulum with Kalman Filter
//MPU board:  NUCLEO-F401RE
//Accelerometer + Gyro sensor: BMX055
//Motor driver: TA7291P x 2
//2019/03/01  N. Beppu
//=========================================================
#include "mbed.h"
#include "math.h"

//=========================================================
//Port Setting
DigitalOut led1(LED1);  //LED on the NUCLEO board
I2C i2c(PB_9, PB_8);    // Gyro + ACC (SDA, SCLK)
BusIn encoder_bus(PC_11, PD_2); //Encoder (LSB to MSB)
DigitalOut IN1(PC_8);   //TA7291P IN1
DigitalOut IN2(PC_6);   //TA7291P IN2
PwmOut motor(PC_9);     //TA7291P Vref
DigitalOut led_r(PC_0); //Red LED
DigitalOut led_g(PC_1); //Green LED
DigitalOut led_y(PC_2); //Yellow LED

//=========================================================
//Ticker
Ticker timer1; //for rotary encoder
Ticker timer2; //for Kalman filter (angle)

//=========================================================
//Accelerometer and gyro statistical data
int sample_num = 100;
float meas_interval = 0.01;
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
//Rotary encoder variables
int rotary_encoder_update_rate = 25; //usec
int rotary_encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
float pre_theta2 = 0;

//=========================================================
//Kalman filter (for angle estimation) variables
//Update rate
float theta_update_freq = 400; //Hz
float theta_update_interval = 1.0f/theta_update_freq;
//State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
//Covariance matrix
float P_theta_predict[2][2];
float P_theta[2][2];
//"A" of the state equation
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
//"B" of the state equation
float B_theta[2][1] = {{theta_update_interval}, {0}};
//"C" of the state equation
float C_theta[1][2] = {{1, 0}};

//=========================================================
//Kalman filter (for all system estimation) variables
//State vector
//[[theta1(rad)], [theta1_dot(rad/s)], [theta2(rad)]. [theta2_dot(rad/s)]]
float x_data_predict[4][1];
float x_data[4][1];
//Covariance matrix
float P_x_predict[4][4];
float P_x[4][4];
//"A" of the state equation (update freq = 100 Hz)
float A_x[4][4] = {
{1.00210e+00,1.00070e-02,0.00000e+00,3.86060e-05},
{4.20288e-01,1.00210e+00,0.00000e+00,7.65676e-03},
{-1.15751e-03,-3.87467e-06,1.00000e+00,9.74129e-03},
{-2.29569e-01,-1.15751e-03,0.00000e+00,9.48707e-01}
};
//"B" of the state equation (update freq = 100 Hz)
float B_x[4][1] = {
{-2.70805e-04},
{-5.37090e-02},
{1.81472e-03},
{3.59797e-01}
};
//"C" of the state equation (update freq = 100 Hz)
float C_x[4][4] = {
{1, 0, 0, 0},
{0, 1, 0, 0},
{0, 0, 1, 0},
{0, 0, 0, 1}
};
//measurement noise
float measure_variance_mat[4][4];
//System noise
float voltage_error = 0.01; //volt
float voltage_variance = voltage_error * voltage_error;

//=========================================================
//Motor control variables
float feedback_rate = 0.01; //sec
float motor_value = 0;
int pwm_width = 0;
int motor_direction = 1;
float motor_offset = 0.17; //volt

//=========================================================
//Gain vector for the state feedback 
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};

//=========================================================
// Matrix common functions
//=========================================================
//Matrix addition
void mat_add(float *m1, float *m2, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = m1[i*column + j] + m2[i*column + j];    
        }    
    }
    return;
}

//Matrix subtraction
void mat_sub(float *m1, float *m2, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = m1[i*column + j] - m2[i*column + j];    
        }    
    }
    return;
}

//Matrix multiplication
void mat_mul(float *m1, float *m2, float *sol, int row1, int column1, int row2, int column2)
{
    for(int i=0; i<row1; i++)
    {
        for(int j=0; j<column2; j++)
        {
            sol[i*column2 + j] = 0;
            for(int k=0; k<column1; k++)
            {
                sol[i*column2 + j] += m1[i*column1 + k]*m2[k*column2 + j];    
            }
        }    
    }
    return;
}

//Matrix transposition
void mat_tran(float *m1, float *sol, int row_original, int column_original)
{
    for(int i=0; i<row_original; i++)
    {
        for(int j=0; j<column_original; j++)
        {
            sol[j*row_original + i] = m1[i*column_original + j];    
        }    
    }
    return;
}

//Matrix scalar maltiplication
void mat_mul_const(float *m1,float c, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = c * m1[i*column + j];    
        }    
    }
    return;
}

//Matrix inversion (by Gaussian elimination)
void mat_inv(float *m, float *sol, int column, int row)
{
    //allocate memory for a temporary matrix
    float* temp = (float *)malloc( column*2*row*sizeof(float) );
    
    //make the augmented matrix
    for(int i=0; i<column; i++)
    {
        //copy original matrix
        for(int j=0; j<row; j++)
        {
            temp[i*(2*row) + j] = m[i*row + j];  
        }
        
        //make identity matrix
        for(int j=row; j<row*2; j++)
        {
            if(j-row == i)
            {
                temp[i*(2*row) + j] = 1;
            }    
            else
            {
                temp[i*(2*row) + j] = 0;    
            }
        }
    }

    //Sweep (down)
    for(int i=0; i<column; i++)
    {
        //pivot selection
        float pivot = temp[i*(2*row) + i];
        int pivot_index = i;
        float pivot_temp;
        for(int j=i; j<column;j++)
        {
            if( temp[j*(2*row)+i] > pivot )
            {
                pivot = temp[j*(2*row) + i];
                pivot_index = j;
            }    
        }  
        if(pivot_index != i)
        {
            for(int j=0; j<2*row; j++)
            {
                pivot_temp = temp[ pivot_index * (2*row) + j ];
                temp[pivot_index * (2*row) + j] = temp[i*(2*row) + j];
                temp[i*(2*row) + j] = pivot_temp;    
            }    
        }
        
        //division
        for(int j=0; j<2*row; j++)
        {
            temp[i*(2*row) + j] /= pivot;    
        }
        
        //sweep
        for(int j=i+1; j<column; j++)
        {
            float temp2 = temp[j*(2*row) + i];
            
            //sweep each row
            for(int k=0; k<row*2; k++)
            {
                temp[j*(2*row) + k] -= temp2 * temp[ i*(2*row) + k ];    
            }    
        }
    }
        
    //Sweep (up)
    for(int i=0; i<column-1; i++)
    {
        for(int j=i+1; j<column; j++)
        {
            float pivot = temp[ (column-1-j)*(2*row) + (row-1-i)];   
            for(int k=0; k<2*row; k++)
            {
                temp[(column-1-j)*(2*row)+k] -= pivot * temp[(column-1-i)*(2*row)+k];    
            }
        }    
    }     
    
    //copy result
    for(int i=0; i<column; i++)
    {
        for(int j=0; j<row; j++)
        {
            sol[i*row + j] = temp[i*(2*row) + (j+row)];    
        }    
    }
    free(temp);
    return;
}

//=========================================================
// I2C common functions
//=========================================================
//i2c write function
void i2c_mem_write(int device_address, int mem_address, int mem_data)
{  
    int device_address_temp = device_address<<1;
    device_address_temp = device_address_temp & 0xfe;

    i2c.start();
    i2c.write(device_address_temp);
    i2c.write(mem_address);
    i2c.write(mem_data); 
    i2c.stop();   
    return;
}

//i2c read function
int i2c_mem_read(int device_address, int mem_address)
{   
    int device_address_temp = device_address<<1;
    int device_address_temp_w = device_address_temp & 0xfe; 
    int device_address_temp_r = device_address_temp | 0x01;

    i2c.start();
    i2c.write(device_address_temp_w);
    i2c.write(mem_address);  
    i2c.start();
    i2c.write(device_address_temp_r);    
    int data = i2c.read(0);
    i2c.stop();   
    return data;
}

//=========================================================
// Accelerometer (BMX055)
//=========================================================
//get data
float get_acc_data()
{
    //read ACCD_Y_LSB registor (0x04)
    int y_temp_L = i2c_mem_read(0x19, 0x04);
    y_temp_L = y_temp_L >> 4;
    y_temp_L = y_temp_L & 0x0f;
    
    //read RATE_Y_MSB registor (0x05)
    int y_temp_H = i2c_mem_read(0x19, 0x05);
    
    //calculate Y acceleration
    int y_data = y_temp_L + 16 * y_temp_H;
    if(y_data > 2047)
    {
        y_data = -1 * (4096 - y_data);    
    }
    
    //-----------------------------------------------------    
    //read ACCD_Z_LSB registor (0x06)
    int z_temp_L = i2c_mem_read(0x19, 0x06);
    z_temp_L = z_temp_L >> 4;
    z_temp_L = z_temp_L & 0x0f;
    
    //read RATE_Z_MSB registor (0x07)
    int z_temp_H = i2c_mem_read(0x19, 0x07);
    
    //calculate Z acceleration
    int z_data = z_temp_L + 16 * z_temp_H;
    if(z_data > 2047)
    {
        z_data = -1 * (4096 - z_data);    
    }

    //-----------------------------------------------------  
    //calculate theta
    float theta_deg = atan( float(z_data) / float(y_data) );
    return (float)theta_deg * 57.29578f;   //degree
}

//statistical data of accelerometer
void acc_init()
{             
    //initialize ACC register 0x0F (range)
    //Full scale = +/- 2 G
    i2c_mem_write(0x19, 0x0f, 0x03);
 
    //initialize ACC register 0x10 (band width)
    //Filter bandwidth = 1000 Hz
    i2c_mem_write(0x19, 0x10, 0x0f);

    //get data
    float theta_array[sample_num];
    for(int i=0; i<sample_num; i++)
    {
        theta_array[i] = get_acc_data();    
        wait( meas_interval );
    }
    
    //calculate mean
    theta_mean = 0;
    for(int i=0; i<sample_num; i++)
    {
            theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;
    
    //calculate variance
    float temp;
    theta_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
            temp = theta_array[i] - theta_mean;
            theta_variance += temp*temp;
    }
    theta_variance /= sample_num;
    return;
}

//=========================================================
// Gyroscope (BMX055)
//=========================================================
//get data
float get_gyro_data()
{    
    //read RATE_X_LSB registor (0x02)
    int x_temp_L = i2c_mem_read(0x69, 0x02);
    //read RATE_X_MSB registor
    int x_temp_H = i2c_mem_read(0x69, 0x03);
    
    //calculate X angular ratio
    int x_data = x_temp_L + 256 * x_temp_H;
    if(x_data > 32767)
    {
        x_data = -1 * (65536 - x_data);    
    }
    x_data = -1 * x_data;
    // +1000 (deg/sec) / 2^15 = 0.0305176
    return float(x_data) * 0.0305176f; // deg/sec
}

//statistical data of gyro
void gyro_init()
{             
    //initialize Gyro register 0x0F (range)
    //Full scale = +/- 1000 deg/s
    i2c_mem_write(0x69, 0x0f, 0x01);
     
    //initialize Gyro register 0x10 (band width)
    //Data rate = 1000 Hz, Filter bandwidth = 116 Hz
    i2c_mem_write(0x69, 0x10, 0x02); 
  
    //get data
    float theta_dot_array[sample_num];
    for(int i=0;i<sample_num;i++)
    {
        theta_dot_array[i] = get_gyro_data();    
        wait(meas_interval);
    }
    
    //calculate mean
    theta_dot_mean = 0;
    for(int i=0;i<sample_num;i++)
    {
        theta_dot_mean += theta_dot_array[i];    
    }
    theta_dot_mean /= sample_num;
 
    //calculate variance
    float temp;
    theta_dot_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp*temp;    
    }
    theta_dot_variance /= sample_num;
    return;
}

//=========================================================
//Rotary encoder polling function
//It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
void rotary_encoder_check()
{  
    static int code; 
    //check the movement
    code = ( (code<<2) +  int(encoder_bus) ) & 0xf ;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}

//=========================================================
//Kalman filter for "theta" & "theta_dot_bias" 
//It takes 650 usec. (NUCLEO-F401RE 84MHz, BMX055)
//=========================================================
void update_theta()
{     
    //detach the rotary encoder polling
    timer1.detach();

    //measurement data
    float y = get_acc_data(); //degree
    
    //input data
    float theta_dot_gyro = get_gyro_data(); //degree/sec
      
    //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    float P_CT[2][1] = {};
    float tran_C_theta[2][1] = {};
    mat_tran(C_theta[0], tran_C_theta[0], 1, 2);//C^T
    mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);//P'C^T
    float G_temp1[1][1] = {};
    mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1,2, 2,1);//CP'C^T
    float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);//(W+CP'C^T)^-1
    float G[2][1] = {};
    mat_mul_const(P_CT[0], G_temp2, G[0], 2, 1);//P'C^T(W+CP'C^T)^-1
    
    //theta_data estimation: theta = theta'+G(y-Ctheta')
    float C_theta_theta[1][1] = {};
    mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2, 2, 1);//Ctheta'
    float delta_y = y - C_theta_theta[0][0];//y-Ctheta'
    float delta_theta[2][1] = {};
    mat_mul_const(G[0], delta_y, delta_theta[0], 2, 1);
    // 行列の先頭のアドレスを引数として、行列計算を行う
    mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);
           
    //calculate covariance matrix: P=(I-GC)P'
    float GC[2][2] = {};
    float I2[2][2] = {{1,0},{0,1}};
    mat_mul(G[0], C_theta[0], GC[0], 2, 1, 1, 2);//GC
    float I2_GC[2][2] = {};
    mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
    mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);//(I-GC)P'
      
    //predict the next step data: theta'=Atheta+Bu
    float A_theta_theta[2][1] = {};
    float B_theta_dot[2][1] = {};
    mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);//Atheta
    mat_mul_const(B_theta[0], theta_dot_gyro, B_theta_dot[0], 2, 1);//Bu
    mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2, 1);//Atheta+Bu 
    
    //predict covariance matrix: P'=APA^T + BUB^T
    float AP[2][2] = {};   
    float APAT[2][2] = {};
    float tran_A_theta[2][2] = {};
    mat_tran(A_theta[0], tran_A_theta[0], 2, 2);//A^T 
    mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);//AP
    mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);//APA^T
    float BBT[2][2];
    float tran_B_theta[1][2] = {};
    mat_tran(B_theta[0], tran_B_theta[0], 2, 1);//B^T
    mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);//BB^T
    float BUBT[2][2] = {};
    mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);//BUB^T
    mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);//APA^T+BUB^T
   
    //attach a timer for the rotary encoder (40 kHz)
    timer1.attach_us(&rotary_encoder_check, rotary_encoder_update_rate); 
}


//=========================================================
// Main
//=========================================================
int main() {   

    //-------------------------------------------
    //LED
    //-------------------------------------------
    led1 = 0;
    led_r = 0;
    led_g = 0;
    led_y = 0;
    wait(1);   //wait 1 sec
    led_y = 1; //turns on the yellow LED 

    //-------------------------------------------
    //I2C initialization
    //-------------------------------------------
    i2c.frequency(400000); //400 kHz

    //-------------------------------------------
    //Accelerometer & Gyro initialization
    //-------------------------------------------
    acc_init(); 
    gyro_init();
    
    //-------------------------------------------
    //Rotary encoder initialization
    //-------------------------------------------
    encoder_value = 0;  

    //-------------------------------------------
    //Motor driver intialization
    //-------------------------------------------
    IN1 = 0; //motor stop
    IN2 = 0; //motor stop
    motor.period_us(100);   //10 kHz pulse
    motor.pulsewidth_us(0); //0 to 100

    //-------------------------------------------
    //Kalman filter (angle) initialization
    //-------------------------------------------
    //initial value of theta_data_predict
    theta_data_predict[0][0] = 0;
    theta_data_predict[1][0] = theta_dot_mean;
    
    //initial value of P_theta_predict
    P_theta_predict[0][0] = 1;
    P_theta_predict[0][1] = 0;
    P_theta_predict[1][0] = 0;
    P_theta_predict[1][1] = theta_dot_variance;

    //-------------------------------------------
    //Kalman filter (all system) variables
    //------------------------------------------- 
    //variable for measurement data
    float y[4][1];
    
    //variables for Kalman gain calculation
    float theta1_dot_temp;
    float tran_C_x[4][4];
    float P_CT[4][4];
    float G_temp1[4][4];    
    float G_temp2[4][4];
    float G_temp2_inv[4][4];
    float G[4][4];

    //variables for x_hat estimation
    float C_x_x[4][1];
    float delta_y[4][1];
    float delta_x[4][1];

    //variables for covariance matrix calculation
    float GC[4][4];
    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    float I4_GC[4][4];
    
    //variables for x prediction
    float Vin;
    float A_x_x[4][1];
    float B_x_Vin[4][1];
   
    //variables for covariance prediction
    float tran_A_x[4][4];
    float AP[4][4]; 
    float APAT[4][4];
    float BBT[4][4];
    float tran_B_x[1][4];
    float BUBT[4][4];

    //-------------------------------------------  
    //Kalman filter (all system) initialization
    //-------------------------------------------
    //initial value of x_data_predict
    for(int i=0; i<4; i++)
    {
        x_data_predict[i][0] = 0;
    }
    
    //initial value of P_x_predict
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            P_x_predict[i][j] = 0;    
        } 
    }
    for(int i=0; i<4; i++)
    {
        P_x_predict[i][i] = 1e-4;   
    }
    
    //measurement noise matrix
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            measure_variance_mat[i][j] = 0;    
        } 
    }
    float deg_rad_coeff = (3.14*3.14)/(180*180);
    measure_variance_mat[0][0] = theta_variance * deg_rad_coeff;
    measure_variance_mat[1][1] = theta_dot_variance * deg_rad_coeff;
    float encoder_error = 0.1f*2*3.14f/(4*rotary_encoder_resolution);
    measure_variance_mat[2][2] = encoder_error * encoder_error;
    float encoder_rate_error = encoder_error / feedback_rate;
    measure_variance_mat[3][3] = encoder_rate_error * encoder_rate_error;

    //-------------------------------------------  
    //Timer
    //-------------------------------------------  
    //timer1: rotary encoder polling, 40 kHz
    timer1.attach_us(&rotary_encoder_check, rotary_encoder_update_rate);
    //timer2: Kalman filter (theta & theta_dot), 400 Hz
    timer2.attach(&update_theta, theta_update_interval);

    //-------------------------------------------  
    //initialization done
    //-------------------------------------------  
    led_y = 0;
 
    //===========================================
    //Main loop
    //it takes 700 usec (calculation)
    //===========================================    
    while(1)
    {
        //stop theta update process
        timer2.detach();
        
        //turn off LEDs
        led1 = !led1;
        led_g = 0;
        led_r = 0;
         
        //---------------------------------------
        //Kalman Filter (all system)
        //---------------------------------------
        //measurement data
        y[0][0] = theta_data[0][0] * 3.14f/180;
        theta1_dot_temp = get_gyro_data();
        y[1][0] = ( theta1_dot_temp - theta_data[1][0]) * 3.14f/180;
        y[2][0] = encoder_value * (2*3.14f)/(4*rotary_encoder_resolution);
        y[3][0] = (y[2][0] - pre_theta2)/feedback_rate;    
                
        //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
        mat_tran(C_x[0], tran_C_x[0], 4, 4);//C^T
        mat_mul(P_x_predict[0], tran_C_x[0], P_CT[0], 4, 4, 4, 4);//P'C^T
        mat_mul(C_x[0], P_CT[0], G_temp1[0], 4, 4, 4, 4);//CPC^T
        mat_add(G_temp1[0], measure_variance_mat[0], G_temp2[0], 4, 4);//W+CP'C^T
        mat_inv(G_temp2[0], G_temp2_inv[0], 4, 4);//(W+CP'C^T)^-1
        mat_mul(P_CT[0], G_temp2_inv[0], G[0], 4, 4, 4, 4); //P'C^T(W+CP'C^T)^-1
        
        //x_data estimation: x = x'+G(y-Cx')
        mat_mul(C_x[0], x_data_predict[0], C_x_x[0], 4, 4, 4, 1);//Cx'
        mat_sub(y[0], C_x_x[0], delta_y[0], 4, 1);//y-Cx'
        mat_mul(G[0], delta_y[0], delta_x[0], 4, 4, 4, 1);//G(y-Cx')
        mat_add(x_data_predict[0], delta_x[0], x_data[0], 4, 1);//x'+G(y-Cx')
        
        //calculate covariance matrix: P=(I-GC)P'
        mat_mul(G[0], C_x[0], GC[0], 4, 4, 4, 4);//GC
        mat_sub(I4[0], GC[0], I4_GC[0], 4, 4);//I-GC
        mat_mul(I4_GC[0], P_x_predict[0], P_x[0], 4, 4, 4, 4);//(I-GC)P'
      
        //predict the next step data: x'=Ax+Bu
        Vin = motor_value;
        if(motor_value > 3.3f)
        {
            Vin = 3.3f;
        }
        if(motor_value < -3.3f)
        {
            Vin = -3.3f;    
        }
        mat_mul(A_x[0], x_data[0], A_x_x[0], 4, 4, 4, 1);//Ax_hat
        mat_mul_const(B_x[0], Vin , B_x_Vin[0], 4, 1);//Bu
        mat_add(A_x_x[0], B_x_Vin[0], x_data_predict[0], 4, 1);//Ax+Bu 
        
        //predict covariance matrix: P'=APA^T + BUB^T
        mat_tran(A_x[0], tran_A_x[0], 4, 4);//A^T
        mat_mul(A_x[0], P_x[0], AP[0], 4, 4, 4, 4);//AP
        mat_mul(AP[0], tran_A_x[0], APAT[0], 4, 4, 4, 4);//APA^T
        mat_tran(B_x[0], tran_B_x[0], 4, 1);//B^T
        mat_mul(B_x[0], tran_B_x[0], BBT[0], 4, 1, 1, 4);//BB^T
        mat_mul_const(BBT[0], voltage_variance, BUBT[0], 4, 4);//BUB^T
        mat_add(APAT[0], BUBT[0], P_x_predict[0], 4, 4);//APA^T+BUB^T

        //---------------------------------------
        //Motor control
        //---------------------------------------
        //reset
        motor_value = 0;
        
        //calculate Vin
        for(int i=0; i<4; i++)
        { 
            motor_value += Gain[i] * x_data[i][0];
        }
        
        //offset
        if(motor_value > 0)
        {
            motor_value += motor_offset ;   
        }
        if(motor_value < 0)
        {
            motor_value -= motor_offset;    
        }
        
        //calculate PWM pulse width
        pwm_width = int( motor_value*100.0f/3.3f );
        
        //drive the motor in forward
        if(pwm_width>=0)
        {
            //over voltage
            if(pwm_width>100)
            {
                pwm_width = 100;    
            }         
            //to protect TA7291P
            if(motor_direction == 2)
            {
                IN1 = 0;
                IN2 = 0;
                wait(0.0001); //wait 100 usec    
            }        
            //forward
            IN1 = 1;
            IN2 = 0;
            led_g = 1;
            motor.pulsewidth_us(pwm_width);
            motor_direction = 1;
        }      
        //drive the motor in reverse
        else
        {
            //calculate the absolute value
            pwm_width = -1 * pwm_width;

            //over voltage
            if(pwm_width>100)
            {
                pwm_width = 100;    
            }
            //to protect TA7291P
            if(motor_direction == 1)
            {
                IN1 = 0;
                IN2 = 0;
                wait(0.0001); //wait 100 usec    
            }
            //reverse
            IN1 = 0;
            IN2 = 1;
            led_r = 1;
            motor.pulsewidth_us(pwm_width);
            motor_direction = 2;          
        }
        
        // prepare for the next calculation of theta2_dot
        pre_theta2 = y[2][0];
        // start the angle update process
        timer2.attach(&update_theta, theta_update_interval);
        // wait 
        wait(feedback_rate);    
    }    
    //===========================================
    //Main loop (end)
    //=========================================== 
}
