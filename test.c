/*
    Dataset collection with Crazyflie and AIdeck. This file can save images and drone states to Flash;
    Then, the data can be uploaded to laptop via Jtag.
    Shushuai Li, Mavlab, Tudelft

    Write dataset: make clean all write=1; make image flash
    Turn off the drone and fly it.
    Read dataset: make clean all run io=host write=0

    Attention: Erase the flash with io=host before logging onboard
 */

#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "gaplib/ImgIO.h"
// #include "img_proc.h"
#include "bsp/flash.h"
#include "bsp/flash/hyperflash.h"
#include "img_proc.h"

#define WIDTH    324
#define HEIGHT   244
#define CROP_SIZE 71680 // 224x320
#define BUFF_SIZE (WIDTH*HEIGHT)
PI_L2 unsigned char *buff;
PI_L2 unsigned char *buff_demosaick;
PI_L2 unsigned char value;

// #define UART_RCV_SIZE 28
// #define PACKET_SIZE 24

#define UART_RCV_SIZE 12     // 2 x 4 (float bytes) + 4 = 12 
#define PACKET_SIZE 8        // 2 x 4 (float bytes) = 8

static PI_L2 unsigned char buff_uart_rcv[UART_RCV_SIZE];
static PI_L2 unsigned char buff_packet[PACKET_SIZE];
static PI_L2 unsigned char buff_packet_tmp[PACKET_SIZE];

// For async Uart read
struct pi_device uart;
pi_task_t taskUart;
void uart_rx_cb(void *arg);

static struct pi_device camera;
static struct pi_device flash;

#define READSIZE 888             // 512 Mbit -> 64MB flash; 64000 kb / 72 kb(per img) = 888.88   
uint32_t flash_address = 0;      // 300*(CROP_SIZE+PACKET_SIZE);     // start address in Flash
char image_name[13];

static uint16_t image_number = 0;    
static float previousRoll;
float cf_tick, pos_z;
static bool start_storing = true;

static float decode_packet(uint8_t *src, float * cf_tick, float * pos_z);
static float buff_strip(uint8_t *src, uint8_t *tar);

// check img end
static bool IMG_END;

int test_camera()
{
    printf("Entering main controller\n");
    pi_time_wait_us(1000*5000); // delay 5s, reliable initialization
    // ------ open led
    static struct pi_device gpio_device;
    // pi_pad_set_function(PI_PAD_14_A2_RF_PACTRL2, PI_PAD_14_A2_GPIO_A2_FUNC1);
    pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);
    static int led_val = 1;
    pi_gpio_pin_write(&gpio_device, 2, led_val);

    // ------ reserve buffer space
    buff = pmsis_l2_malloc(BUFF_SIZE);
    if (buff == NULL){ return -1;}
    // buff_demosaick = pmsis_l2_malloc(BUFF_SIZE*3);   // this buffer is used for color image
    // if (buff_demosaick == NULL){ return -1;}
    printf("Initialized buffers\n");

    // ------ open flash
    // pi_pad_set_function(PI_PAD_46_B7_SPIM0_SCK, PI_PAD_46_B7_HYPER_DQ6_FUNC3);
    static struct pi_hyperflash_conf flash_conf;
    pi_hyperflash_conf_init(&flash_conf);
    pi_open_from_conf(&flash, &flash_conf);
    if (pi_flash_open(&flash))
    {
        printf("Error flash open !\n");
        pmsis_exit(-3);
    }
    printf("Open Hyper flash\n");

#if WRITE_DATASET
    // ------ erase flash
    pi_flash_erase_chip(&flash);
    printf("Erase done\n");

    // ------ open uart
    pi_pad_set_function(PI_PAD_46_B7_SPIM0_SCK, PI_PAD_46_B7_SPIM0_SCK_FUNC0);
    // ----- For async Uart read
    struct pi_uart_conf uart_conf;
    pi_uart_conf_init(&uart_conf);
    // conf.enable_tx = 1;
    uart_conf.enable_rx = 1;
    uart_conf.baudrate_bps = 115200;
    pi_open_from_conf(&uart, &uart_conf);
    if (pi_uart_open(&uart))
    {
        printf("Uart open failed !\n");
        pmsis_exit(-1);
    }
    pi_uart_read_async(&uart, buff_uart_rcv, UART_RCV_SIZE, pi_task_callback(&taskUart, (void *) uart_rx_cb, NULL));
    printf("Start receiving !\n");
    
    // ------ open camera
    struct pi_himax_conf cam_conf;
    pi_himax_conf_init(&cam_conf);
    cam_conf.format = PI_CAMERA_QVGA;
    pi_open_from_conf(&camera, &cam_conf);
    if (pi_camera_open(&camera))
    {
        printf("Failed to open camera\n");
        pmsis_exit(-1);
    }
    // pi_camera_control(&camera, PI_CAMERA_CMD_AEG_INIT, 0);
    printf("Open Himax camera\n");
    
    // ------ camera rotation and qvga mode
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    uint8_t set_value=3;
    uint8_t reg_value;
    pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);
    pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);
    if (set_value!=reg_value)
    {
        printf("Failed to rotate camera image\n");
        return -1;
    }
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(&camera, PI_CAMERA_CMD_AEG_INIT, 0);

    set_value=1;
    pi_camera_reg_set(&camera, QVGA_WIN_EN, &set_value);
    pi_camera_reg_get(&camera, QVGA_WIN_EN, &reg_value);
    printf("qvga window enabled %d\n",reg_value);
    //pi_time_wait_us(1000*5000); // delay 5s for setting logging on cfclient
    pi_gpio_pin_write(&gpio_device, 2, 0);
#endif
    while(1)
    {
#if WRITE_DATASET
        pi_time_wait_us(10*1000); // 10ms, otherwise the following if cannot be detected
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_camera_capture(&camera, buff, WIDTH*HEIGHT);
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        for (int i=0; i<PACKET_SIZE; i++)
            {buff_packet_tmp[i]=buff_packet[i];}

        pi_pad_set_function(PI_PAD_46_B7_SPIM0_SCK, PI_PAD_46_B7_SPIM0_SCK_FUNC0);

        // printf("tick: %f, pos_z %f\n", cf_tick, pos_z);

        led_val ^= 1;
        pi_gpio_pin_write(&gpio_device, 2, led_val);    // [LED blink]

        printf("%d\n",start_storing);
        if(start_storing)
        {
            // if(true){
            // Crop from 244x324 to 224x320   [why do this step? slow down the speed]
            int ps=0;
            for(int i=0; i<244; i++){
                for(int j=0; j<324; j++){
                    if ( i<234 && i>9 && j<322 && j>1){
                        buff[ps] = buff[i*324+j];
                        ps++;        			
                    }
                }
            }
            // Uncomment the following code for sampling the calibration images
            // sprintf(image_name, "../../../images/img%05d.ppm", image_number);
            // demosaicking(buff, buff_demosaick, 320, 224, 0);
            // WriteImageToFile(image_name, 320, 224, sizeof(uint32_t), buff_demosaick, RGB888_IO);

            // Saving real-time flight images to flash
            pi_pad_set_function(PI_PAD_46_B7_SPIM0_SCK, PI_PAD_46_B7_HYPER_DQ6_FUNC3); // HW bug
            pi_flash_program(&flash, flash_address, buff, (uint32_t) CROP_SIZE);
            flash_address += CROP_SIZE;
            // saving the uart data sent from CF to flash 
            // (uart data is updated in high-frequency, we assume here the uart data is updated, which is captured as the same time as the image data)
            pi_flash_program(&flash, flash_address, buff_packet_tmp, (uint32_t) PACKET_SIZE);
            flash_address += PACKET_SIZE;

            /* pi_flash_program(&flash, flash_address, buff_demosaick, (uint32_t) BUFF_SIZE3);
            flash_address += BUFF_SIZE3; */
            image_number ++;
            printf("image num: %05d\n", image_number);
        }

        // [interesting bug]: the following start_storing cannot overwirte the one on top  
        // if (image_number>3)
        // {
        //     start_storing = false;
        //     printf("we have 3 images\n");
        //     printf("%d\n",start_storing);
        // }

#else
        for (uint32_t i=0; i<BUFF_SIZE; i++){buff[i] = 0;}                            // can we skip this part?
        pi_flash_read(&flash, flash_address, buff, (uint32_t) CROP_SIZE);
        flash_address += CROP_SIZE;

        // check if reach the end of img sequence
        // if( ((buff[0]==0xFF) && (buff[1]==0xFF)) || (image_number > READSIZE) ){   [Found the problem]: need a better way to check the end of img
        
        // One walk around: check the entire cropped image. 
        // If all pixels are white, indicated as the end of img sequence.
        IMG_END = true;                        // flag 
        for(uint32_t i=0; i<CROP_SIZE; i++){
                if(buff[i] != 0xFF){           // if any pixel is not 0XFF, the image is valide
                    IMG_END = false;
                    printf("detect at buff[i] = %d\n",i);
                    break;                     // break the for loop  
                }
            } 

        if( IMG_END ||(image_number > READSIZE) ){
            // reach the end of img
            printf("Reach the end of images\n");
            //------ write attitude and 3D pos to a txt file
            struct pi_fs_conf conf;
            pi_fs_conf_init(&conf);
            struct pi_device fs;
            conf.type = PI_FS_HOST;
            pi_open_from_conf(&fs, &conf);
            if (pi_fs_mount(&fs))
                return -1;

            void *File = pi_fs_open(&fs, "../../../images/label.txt", PI_FS_FLAGS_WRITE);
            char line_data[100];
            flash_address = CROP_SIZE;
            
            for (int i=0; i<image_number; i++){
                pi_flash_read(&flash, flash_address, buff_packet, (uint32_t) (PACKET_SIZE));
                flash_address += (CROP_SIZE+PACKET_SIZE);
    
                decode_packet(buff_packet, &cf_tick, &pos_z);  // cf_tick, pos_z;
                sprintf(line_data, "img%05d.ppm %f %f \n", i, cf_tick, pos_z);                                    
                int size_line_data = 0;
                while(line_data[size_line_data]!='\n'){size_line_data++;}
                pi_fs_write(File, line_data, size_line_data+1);
            }
            pi_fs_close(File);
            pi_fs_unmount(&fs);

            pi_flash_close(&flash);
            pmsis_exit(0);
        }
        sprintf(image_name, "../../../images/img%05d.ppm", image_number);
        // save gray image //
        WriteImageToFile(image_name, 320, 224, sizeof(uint8_t), buff, GRAY_SCALE_IO);

        // save color image //
        //demosaicking(buff, buff_demosaick, 320, 224, 0);
        //WriteImageToFile(image_name, 320, 224, sizeof(uint32_t), buff_demosaick, RGB888_IO);

        pi_flash_read(&flash, flash_address, buff_packet, (uint32_t) PACKET_SIZE);
        flash_address += PACKET_SIZE;
        image_number ++;
        printf("images read:%d\n", image_number);
#endif        
    }
    pmsis_exit(0);
}

int main(void)
{
    printf("\n\t*** PMSIS Camera -> Hyperflash -> Laptop ***\n\n");
    return pmsis_kickoff((void *) test_camera);
}

static float buff_strip(uint8_t *src, uint8_t *tar)
{
    for (int i = UART_RCV_SIZE-1; i>0; i--)
    {
        if( (src[i]==0xfe) && (src[i-1]!=0xfe) && (src[i-(PACKET_SIZE+1)]==0xfe) )
        {
            for(int j=0; j<PACKET_SIZE; j++){
                tar[j] = src[i-PACKET_SIZE+j];
            }
        }
    }
}


static float decode_packet(uint8_t *src, float * tick, float * pos_z)
{
    union {
        float a;
        unsigned char bytes[4];
    } thing;
    for(int j=0; j<4; j++){
        thing.bytes[j] = src[0+j];
    }
    *tick = thing.a;
    for(int j=0; j<4; j++){
        thing.bytes[j] = src[4+j];
    }
    *pos_z = thing.a;
}


void uart_rx_cb(void *arg)
{
    buff_strip(buff_uart_rcv, buff_packet);

    decode_packet(buff_packet, &cf_tick, &pos_z); 
    if(pos_z > 0.3f){ start_storing = true;}

    // printf("tick: %f, pos_z: %f \n", cf_tick, pos_z);

    pi_uart_read_async(&uart, buff_uart_rcv, UART_RCV_SIZE, pi_task_callback(&taskUart, (void *) uart_rx_cb, NULL));
}

