
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "wav_encoder.h"
#include "wav_decoder.h"
#include "mp3_decoder.h"
#include "board.h"
#include "filter_resample.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_button.h"

#include "raw_stream.h"
#include "esp_audio.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"

#include "rec_eng_helper.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_client.h"

#include "http_stream.h"

#include "periph_button.h"
#include "periph_wifi.h"

#include "input_key_service.h"

#include "vprocTwolf_access.h"

#include "periph_is31fl3216.h"
#include "IS31FL3216.h"

#include <stdio.h>
#include <stdlib.h>
#include "esp_vad.h"
#include "periph_adc_button.h"
#include "audio_sonic.h"
#include "lwip/apps/sntp.h"


#include <sys/param.h>
#include "esp_system.h"
#include "esp_event_loop.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "amrwb_encoder.h"
#include "amrnb_encoder.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"

#include "sdmmc_cmd.h"
#include "display_service.h"
#include "periph_service.h"
#include "led_bar_is31x.h"
#include "led_indicator.h"
#include "unity.h"

#include "esp_heap_caps.h"

static const char *TAG = "example_asr_keywords";
static const char *TAG_sntp = "example_asr_keywords";
#define TEST_FATFS_READER  "/sdcard/wav.c"

 esp_periph_handle_t led_handle ;

static int my_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    static FILE *file;
    if (file == NULL) {
        file = fopen("/sdcard/wav.c", "rb");
        if (!file) {
            printf("Error opening file\n");
            return -1;
        }
    }
    int read_len = fread(buf, 1, len, file);
    if (read_len == 0) {
        fclose(file);
        file = NULL;
        read_len = AEL_IO_DONE;
    }
    return read_len;
}

void test_s (void)
{

static FILE *file;
    if (file == NULL) {
        file = fopen("/sdcard/wav.c", "rb");
        if (!file) {
            printf("Error opening file\n");
            return -1;
        }
    }
     uint8_t *buff = (uint8_t *)malloc(2000);
    memset(buff, 0, 2000);
    int read_len = fread(buff, 1, 960, file);

     int p;
       printf("111111111111111111\n");
    for(p=0;p<960;p++)
    {
        printf("%c",buff[p]);
    }
    
    free(buff);
    buff=NULL;
}




void app_fat(void)
{ 
audio_element_handle_t fatfs_stream_reader;
     ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set);
test_s();
uint8_t* buffer = audio_malloc(1024*1024*2);
   
    memset(buffer, 0, 960);
    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);


     fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);

 audio_element_set_read_cb(fatfs_stream_reader, my_read_cb, NULL);

          raw_stream_read(fatfs_stream_reader, (char *)buffer, 960);
        
             int j;
    printf("333333333333333\n");
    for(j=0;j<960;j++)
    {
        printf("%c",buffer[j]);
    }
    free(buffer);
    buffer=NULL;
   


}




static void periph_is31fl3216_test(void)
{


	
 



    ESP_LOGI(TAG, "Set up peripherals handle");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    TEST_ASSERT_NOT_NULL(set);
    periph_is31fl3216_cfg_t is31fl3216_cfg = {
        .state = IS31FL3216_STATE_ON
    };

    ESP_LOGI(TAG, "Register gpio isr to peripherals");
    esp_periph_handle_t is31fl3216_handle = periph_is31fl3216_init(&is31fl3216_cfg);
    TEST_ASSERT_FALSE(esp_periph_start(set, is31fl3216_handle));

    for (int i = 0; i < BLUE_LED_MAX_NUM; i++) {
        TEST_ASSERT_FALSE(periph_is31fl3216_set_duty(is31fl3216_handle, i, 255));
    }

   TEST_ASSERT_FALSE(periph_is31fl3216_set_light_on_num(is31fl3216_handle, 1, BLUE_LED_MAX_NUM));
    TEST_ASSERT_FALSE(periph_is31fl3216_set_interval(is31fl3216_handle, 100));
    TEST_ASSERT_FALSE(periph_is31fl3216_set_shift_mode(is31fl3216_handle, PERIPH_IS31_SHIFT_MODE_ACC));
    TEST_ASSERT_FALSE(periph_is31fl3216_set_state(is31fl3216_handle, IS31FL3216_STATE_SHIFT));
    ESP_LOGI(TAG, "Start testing for 5 seconds...");

    vTaskDelay(5000 / portTICK_RATE_MS);
// TEST_ASSERT_FALSE(periph_is31fl3216_set_state(is31fl3216_handle, IS31FL3216_STATE_OFF));
// esp_periph_stop(is31fl3216_handle);
    ESP_LOGI(TAG, "Quit test, release all resources");
 //   TEST_ASSERT_FALSE(esp_periph_set_stop_all(set));
  //  TEST_ASSERT_FALSE(esp_periph_set_destroy(set));
}

    
/*
 * When there is a led display chip, such as MSC_V2.1, MSC_V2.2, we use "led_bar" to control the leds on board
 * To run this case, please choose MSC_V2.1 or MSC_V2.2
 */
void leed(void)
{
    esp_periph_handle_t led_handle = led_bar_is31x_init();
    TEST_ASSERT_NOT_NULL(led_handle);

    ESP_LOGI(TAG, "wifi connected");
    TEST_ASSERT_FALSE(led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_CONNECTED, 0));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "wifi setting");
    TEST_ASSERT_FALSE(led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_SETTING, 0));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "wifi connectting");
    TEST_ASSERT_FALSE(led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_CONNECTTING, 0));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "wifi disconnected");
    TEST_ASSERT_FALSE(led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_DISCONNECTED, 0));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "wifi setting finished");
    TEST_ASSERT_FALSE(led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_SETTING_FINISHED, 0));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGW(TAG, "Display service will be destroyed");
    led_bar_is31x_deinit(led_handle);
}

  
 
    
   

//#ifdef test

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#else
#define ESP_IDF_VERSION_VAL(major, minor, patch) 1
#endif

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#include "esp_netif.h"
#else
#include "tcpip_adapter.h"
#endif


#define RING_BUFFER_SIZE (2048)
#define RECORD_TIME_SECONDS (3)

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

#define CONFIG_WIFI_SSID "Archempower"
#define CONFIG_WIFI_PASSWORD "60662155"
#define CONFIG_EXAMPLE_IPV4 1
#define CONFIG_EXAMPLE_IPV4_ADDR "192.168.21.219"
#define CONFIG_EXAMPLE_PORT 3333

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
 char *pcm="/sdcard/hi.pcm";
char *wav="/sdcard/hi.wav";
int pcm_to_wav(const char *pcm, const char*wav);
char buff_2[1024];
char buff_3[1024];
char *copy1=buff_2;
char *copy2=buff_3;
char readBuf[1024];
int l;
int j;
#define PORT CONFIG_EXAMPLE_PORT

 char rx_buffer[10];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

static esp_periph_set_handle_t set;

esp_periph_config_t periph_cfg;
esp_periph_handle_t is31fl3216_periph;
int i=0;
    int led_index = 0;
    char k;
void led_init(void)//初始化灯
{
	
    ESP_LOGI(TAG, "[ 2 ] Initialize IS31fl3216 peripheral");
    periph_is31fl3216_cfg_t is31fl3216_cfg = { 0 };//LED灯珠
    is31fl3216_cfg.state = IS31FL3216_STATE_ON;
    is31fl3216_periph = periph_is31fl3216_init(&is31fl3216_cfg);
    ESP_LOGI(TAG, "[ 3 ] Start peripherals");
  //  esp_periph_start(set, is31fl3216_periph);
  
    ESP_LOGI(TAG, "[ 4 ] Set duty for each LED index");
    for (int i = 0; i < 12; i++) {//开哪些灯0-14
        periph_is31fl3216_set_duty(is31fl3216_periph, i, 255);
    }

   

}
void led_on(void)//控制灯
{

 
    while (1) {
        int blink_pattern = (1UL << led_index) - 1;//开到哪儿
             led_index++;
        
            if (led_index > 14)
        {
           led_index = 0;
           periph_is31fl3216_set_blink_pattern(is31fl3216_periph, blink_pattern);
               
                periph_is31fl3216_state_t led_state = IS31FL3216_STATE_ON;
                 periph_is31fl3216_set_state(is31fl3216_periph, led_state);
               if (led_index==0)break;
        }
      
    }   
}


void led_off(void)//控制关灯
{

    while (1) {
        int blink_pattern = (1UL << led_index) - 1;//开到哪儿
             led_index++;
        
            if (led_index > 14)
        {
           led_index = 0;
           periph_is31fl3216_set_blink_pattern(is31fl3216_periph, blink_pattern);
               
                periph_is31fl3216_state_t led_state = IS31FL3216_STATE_OFF;
                 periph_is31fl3216_set_state(is31fl3216_periph, led_state);
               if (led_index==0)break;
        }
        ESP_LOGI(TAG, "[ 6 ] Destroy peripherals");
       esp_periph_set_destroy(set);
       
      break;
    }   
}


static const char *TAG_tcp = "example";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG_tcp, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG_tcp, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG_tcp, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG_tcp, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG_tcp, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG_tcp, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG_tcp, "Connected to AP");
}









typedef enum {
    WAKE_UP = 1,
} asr_wakenet_event_t;

typedef enum {
    ID0_DAKAIKONGTIAO       = 0,
    ID1_GUANBIKONGTIAO      = 1,
    ID2_ZENGDAFENGSU        = 2,
    ID3_JIANXIOAFENGSU      = 3,
    ID4_SHENGGAOYIDU        = 4,
    ID5_JIANGDIYIDU         = 5,
    ID6_ZHIREMOSHI          = 6,
    ID7_ZHILENGMOSHI        = 7,
    ID8_SONGFENGMOSHI       = 8,
    ID9_JIENENGMOSHI        = 9,
    ID10_GUANBIJIENENGMOSHI = 10,
    ID11_CHUSHIMOSHI        = 11,
    ID12_GUANBICHUSHIMOSHI  = 12,
    ID13_DAKAILANYA         = 13,
    ID14_GUANBILANYA        = 14,
    ID15_BOFANGGEQU         = 15,
    ID16_ZANTINGBOFANG      = 16,
    ID17_DINGSHIYIXIAOSHI   = 17,
    ID18_DAKAIDIANDENG      = 18,
    ID19_GUANBIDIANDENG     = 19,
    ID_MAX,
} asr_multinet_event_t;

static esp_err_t asr_multinet_control(int commit_id);

static esp_audio_handle_t setup_player()
{
    esp_audio_handle_t player = NULL;
    esp_audio_cfg_t cfg = DEFAULT_ESP_AUDIO_CONFIG();
    audio_board_handle_t board_handle = audio_board_init();
    cfg.vol_handle = board_handle->audio_hal;
    cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    cfg.resample_rate = 16000;
#else
    cfg.resample_rate = 48000;
#endif
    player = esp_audio_create(&cfg);
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    fatfs_stream_cfg_t fs_reader = FATFS_STREAM_CFG_DEFAULT();
    fs_reader.type = AUDIO_STREAM_READER;
    raw_stream_cfg_t raw_reader = RAW_STREAM_CFG_DEFAULT();
    raw_reader.type = AUDIO_STREAM_READER;
    esp_audio_input_stream_add(player, raw_stream_init(&raw_reader));
    esp_audio_input_stream_add(player, fatfs_stream_init(&fs_reader));
    mp3_decoder_cfg_t  mp3_dec_cfg  = DEFAULT_MP3_DECODER_CONFIG();
    esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, mp3_decoder_init(&mp3_dec_cfg));
    i2s_stream_cfg_t i2s_writer = I2S_STREAM_CFG_DEFAULT();

#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    i2s_writer.i2s_config.sample_rate = 16000;
#else
    i2s_writer.i2s_config.sample_rate = 48000;
#endif

    i2s_writer.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_writer);
    esp_audio_output_stream_add(player, i2s_stream_writer);
    esp_audio_vol_set(player, 60);

    ESP_LOGI(TAG, "esp_audio instance is:%p\r\n", player);
    return player;
}

 char buff[1024];
        //uint16_t sn;
  
        uint16_t block;


char *filename="/sdcard/hi.wav";
#define BUFFSIZE 1024
#define PACKETSIZE 2048
#define FILE_NAME_MAX_SIZE 512
char buff_s[BUFFSIZE];
int count;


#define PACKET_HEAD          "\x57\x41\x56"
#define PACKET_NUM              "1024"
#define PACKET_TAIL         "\x56\0x41\0x57"
int num=0;
int count;
int block_length=0;



typedef struct tcp_TEST {
  uint16_t sn;
  uint16_t length;
  uint8_t msg_type;
  char* message;
} tcp_TEST;
static tcp_TEST tcp_message;
tcp_TEST *replyMessage = &tcp_message;
uint8_t msg_type;
#define msg_t       0x52  // 'R'

char* message=buff_s;
void tcp_con(void)
{
#define tcp_g
#ifdef tcp_g

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG_tcp, "Unable to create socket: errno %d", errno);
          //  break;
        }
        ESP_LOGI(TAG_tcp, "Socket created");
    
        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG_tcp, "Socket unable to connect: errno %d", errno);
          //  break;
        }
        ESP_LOGI(TAG_tcp, "Successfully connected");

 #endif       
     }



int cou=0;
void tcp_send()
{


#define tcp_g
#ifdef tcp_g

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG_tcp, "Unable to create socket: errno %d", errno);
          //  break;
        }
        ESP_LOGI(TAG_tcp, "Socket created");
    
        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG_tcp, "Socket unable to connect: errno %d", errno);
          //  break;
        }
        ESP_LOGI(TAG_tcp, "Successfully connected");

 #endif   

 // while (1) {

    FILE *fd=fopen(pcm,"rb");
        if(fd==NULL)
        {
            printf("文件 :%s 没有找到!\n",pcm);

        }
 else 
        {//send file imformation
       
                         struct stat buf ;
                        if ( stat( pcm, &buf ) < 0 )
                       {
                         perror( "stat" );
                          return ;
                         }
                         printf("文件大小:%ld\n", buf.st_size );
            bzero(buff_s,BUFFSIZE);
    #ifdef  mem    

            int file_block_length=0;
            int block=0;
            int total=0;
     #else
            uint16_t file_block_length=0;
            uint16_t block=0;
            uint16_t total=0;
    #endif 
            while((file_block_length=fread(buff_s,sizeof(char),BUFFSIZE,fd))>0)//(BUFFSIZE<buf.st_size ?BUFFSIZE:buf.st_size)
            {                   
                                 num+=1;
                                block=file_block_length;
                                total=total+file_block_length;
                printf("发送文件大小:%d,总共发送了:%d\n",block,total);
       





 ++replyMessage->sn;
  replyMessage->msg_type=msg_t;
 
  replyMessage->length = block;
  replyMessage->message = (char *)message;

static uint8_t buff_1[PACKETSIZE];
 uint8_t *p = buff_1;
  strcpy((char *)p, PACKET_HEAD);
  p += strlen(PACKET_HEAD);  
#if defined(ITP_USE_LITTE_ENDIAN)  
  *(p++) = (replyMessage->sn & 0xff);
  *(p++) = (replyMessage->sn >> 8);    
  *(p++) = (replyMessage->length & 0xff);
  *(p++) = (replyMessage->length >> 8);
  *(p++) = (uint8_t)'R';      
 
#else
  *(p++) = (replyMessage->sn >> 8);
  *(p++) = (replyMessage->sn & 0xff);
  *(p++) = (replyMessage->length >> 8);
  *(p++) = (replyMessage->length & 0xff);
  *(p++) = (uint8_t)'R';  

#endif 
  *(p++) = (uint8_t)msg_t;
  memcpy(p, message, BUFFSIZE);
  p += BUFFSIZE;
 if(send(sock,buff_1,file_block_length+9,0)<0)
                {
                    perror("Send");
                    exit(1);
                }
             
             bzero(buff_1,PACKETSIZE); 
  
            }
            fclose(fd);
            printf("传输 %s 端口号为%d的文件 %s已完成! \n",
                        inet_ntop(AF_INET, &CONFIG_EXAMPLE_IPV4_ADDR, buff, sizeof(buff)),  ntohs(CONFIG_EXAMPLE_PORT), pcm );
                
        }
 if (sock != -1) {
           // ESP_LOGE(TAG_tcp, "Shutting down socket and restarting...");
           // shutdown(sock, 0);
            close(sock);
printf("sock ! \n");
replyMessage->sn=0;
 }

  //}
}




void asr_main()
{


esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
set = esp_periph_set_init(&periph_cfg);

  
#if defined CONFIG_ESP_LYRAT_V4_3_BOARD
    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1UL << get_green_led_gpio(),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = 0
    };
    gpio_config(&gpio_conf);
#endif

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Initialize SR wn handle");
    esp_wn_iface_t *wakenet;
    model_coeff_getter_t *model_coeff_getter;
    model_iface_data_t *model_wn_data;
    const esp_mn_iface_t *multinet = &MULTINET_MODEL;

    get_wakenet_iface(&wakenet);
    get_wakenet_coeff(&model_coeff_getter);
    model_wn_data = wakenet->create(model_coeff_getter, DET_MODE_95);
    int wn_num = wakenet->get_word_num(model_wn_data);
    for (int i = 1; i <= wn_num; i++) {
        char *name = wakenet->get_word_name(model_wn_data, i);
        ESP_LOGI(TAG, "keywords: %s (index = %d)", name, i);
    }
    float wn_threshold = wakenet->get_det_threshold(model_wn_data, 1);
    int wn_sample_rate = wakenet->get_samp_rate(model_wn_data);
    int audio_wn_chunksize = wakenet->get_samp_chunksize(model_wn_data);
    ESP_LOGI(TAG, "keywords_num = %d, threshold = %f, sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", wn_num, wn_threshold, wn_sample_rate, audio_wn_chunksize, sizeof(int16_t));

    model_iface_data_t *model_mn_data = multinet->create(&MULTINET_COEFF, 6000);
    int audio_mn_chunksize = multinet->get_samp_chunksize(model_mn_data);
    int mn_num = multinet->get_samp_chunknum(model_mn_data);
    int mn_sample_rate = multinet->get_samp_rate(model_mn_data);
    ESP_LOGI(TAG, "keywords_num = %d , sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", mn_num,  mn_sample_rate, audio_mn_chunksize, sizeof(int16_t));

    int size = audio_wn_chunksize;
    if (audio_mn_chunksize > audio_wn_chunksize) {
        size = audio_mn_chunksize;
        
    }
    int16_t *buffer = (int16_t *)malloc(size * sizeof(short));//size * sizeof(short)

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader, filter, raw_read;
    esp_audio_handle_t player;
    bool enable_wn = true;
    uint32_t mn_count = 0;

    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    //esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    audio_board_sdcard_init(esp_periph_set_init(&periph_cfg));

    ESP_LOGI(TAG, "[ 2.0 ] Create audio pipeline for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[ 2.1 ] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_cfg.type = AUDIO_STREAM_READER;

#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    i2s_cfg.i2s_config.sample_rate = 16000;
    i2s_cfg.i2s_port = 1;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
#else
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
    ESP_LOGI(TAG, "[ 2.2 ] Create filter to resample audio data");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 48000;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 16000;
    rsp_cfg.dest_ch = 1;
    filter = rsp_filter_init(&rsp_cfg);
#endif

    ESP_LOGI(TAG, "[ 2.3 ] Create raw to receive data");
    raw_stream_cfg_t raw_cfg = {
        .out_rb_size = 8 * 1024,
        .type = AUDIO_STREAM_READER,
    };
    raw_read = raw_stream_init(&raw_cfg);

    ESP_LOGI(TAG, "[ 3 ] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, raw_read, "raw");

#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    ESP_LOGI(TAG, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->raw-->[SR]");
    const char *link_tag[2] = {"i2s", "raw"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);
#else
    audio_pipeline_register(pipeline, filter, "filter");
    ESP_LOGI(TAG, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->filter-->raw-->[SR]");
    const char *link_tag[3] = {"i2s", "filter", "raw"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);
#endif

    player = setup_player();

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);
    
   // char *buf_pcm = (char *)malloc(48 * 1024);
   // memset(buf_pcm, 0, 48 * 1024);
    char buf[960] = {0};
  
    while (1) {
   
        raw_stream_read(raw_read, (char *)buffer,size * sizeof(short));//size * sizeof(short)
  /*      memcpy(buf_pcm, buffer, size * sizeof(short));
        buf_pcm+=960;
        cou+=960;
        if(cou>=48*1024)
        {
           printf("+++++++%x",buf_pcm[500]);   
        free(buf_pcm);
        buf_pcm = NULL;
        }
        */
        if (enable_wn) {
            if (wakenet->detect(model_wn_data, (int16_t *)buffer) ==  WAKE_UP) {
              //  esp_audio_sync_play(player, "file://sdcard/111.mp3", 0);
        led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_SETTING_FINISHED, 0);   
        ESP_LOGI(TAG, "开始识别！！！！！！！！！！！！！！！！！！！！\n"); 
        vTaskDelay(500);
        
        char *buff0 = (char *)malloc(48 * 1024);
        if (NULL == buff0) 
        {
            ESP_LOGE(TAG, "Memory allocation failed!");
            return;
        }
        memset(buff0, 0, 48 * 1024);

      
        for(size_t i = 0; i < 6; i++)
        {
            raw_stream_read(raw_read, (char *)buff0 + i * 8 * 1024, 8 * 1024);
        }
    FILE* f1 = fopen("/sdcard/hi.pcm", "wb");
        if(f1==NULL)
        {
            printf("文件没有找到!\n");
        }
      fwrite(buff0, sizeof(char), 48 * 1024, f1);
        fclose(f1); 
        free(buff0);
        buff0 = NULL;
        printf("pcm_okk!\n");
        //pcm_to_wav(pcm,wav);
       tcp_send();
       printf("tcp_okk!\n");
        enable_wn = false;
            }
              
          
        } 
             
        else {
            mn_count++;
            static FILE *file;
            if (file == NULL) {
            file = fopen("/sdcard/hi.pcm", "rb");
            if (!file) {
            printf("Error opening file\n");
                     }
             }
             
             int file_block_length=0;
            int total=0;
            while((file_block_length=fread(buf,sizeof(char),size * sizeof(short),file))>=0)
            {  
                total=total+file_block_length;
                printf("文件大小:%d,总共识别了:%d\n",file_block_length,total);
                int commit_id = multinet->detect(model_mn_data, buf);
                
            if (asr_multinet_control(commit_id) == ESP_OK ) {
            //esp_audio_sync_play(player, "file://sdcard/222.mp3", 0);
                fclose(file);
                file = NULL;
                enable_wn = true;
                mn_count = 0;
                  break;
            }
             bzero(buf,size * sizeof(short));  
            if (file_block_length == 0) {
                fclose(file);
                file = NULL;
                enable_wn = true;
                 mn_count = 0;
                  printf("该次未识别到\n");
                 break;
                  }  
               
            }


            if (mn_count == mn_num) {
                ESP_LOGI(TAG, "stop multinet");
                enable_wn = true;
                mn_count = 0;
            }
        }
    }

    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");

    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    audio_pipeline_unregister(pipeline, raw_read);
    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, filter);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(raw_read);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(filter);

    ESP_LOGI(TAG, "[ 7 ] Destroy model");
    wakenet->destroy(model_wn_data);
    model_wn_data = NULL;
    free(buffer);
    buffer = NULL;
}

static esp_err_t asr_multinet_control(int commit_id)
{
    if (commit_id >=0 && commit_id < ID_MAX) {
        switch (commit_id) {
            case ID0_DAKAIKONGTIAO:
                ESP_LOGI(TAG, "turn on air-condition");
                break;
            case ID1_GUANBIKONGTIAO:
                ESP_LOGI(TAG, "turn off air-condition");
                break;
            case ID2_ZENGDAFENGSU:
                ESP_LOGI(TAG, "increase in speed");
                break;
            case ID3_JIANXIOAFENGSU:
                ESP_LOGI(TAG, "decrease in speed");
                break;
            case ID4_SHENGGAOYIDU:
                ESP_LOGI(TAG, "increase in temperature");
                break;
            case ID5_JIANGDIYIDU:
                ESP_LOGI(TAG, "decrease in temperature");
                break;
            case ID6_ZHIREMOSHI:
                ESP_LOGI(TAG, "hot mode");
                break;
            case ID7_ZHILENGMOSHI:
                ESP_LOGI(TAG, "slow mode");
                break;
            case ID8_SONGFENGMOSHI:
                ESP_LOGI(TAG, "blower mode");
                break;
            case ID9_JIENENGMOSHI:
                ESP_LOGI(TAG, "save power mode");
                break;
            case ID10_GUANBIJIENENGMOSHI:
                ESP_LOGI(TAG, "turn off save power mode");
                break;
            case ID11_CHUSHIMOSHI:
                ESP_LOGI(TAG, "dampness mode");
                break;
            case ID12_GUANBICHUSHIMOSHI:
                ESP_LOGI(TAG, "turn off dampness mode");


                break;
            case ID13_DAKAILANYA:
                ESP_LOGI(TAG, "turn on bt");
                break;
            case ID14_GUANBILANYA:
                ESP_LOGI(TAG, "turn off bt");
                break;
            case ID15_BOFANGGEQU:
                ESP_LOGI(TAG, "turn on");
                break;
            case ID16_ZANTINGBOFANG:
                ESP_LOGI(TAG, "turn off");
                break;
            case ID17_DINGSHIYIXIAOSHI:
                ESP_LOGI(TAG, "timer one hour");
                break;
            case ID18_DAKAIDIANDENG:
                ESP_LOGI(TAG, "turn on lignt");
               led_bar_is31x_pattern(led_handle, DISPLAY_PATTERN_WIFI_CONNECTED, 0);

                break;
            case ID19_GUANBIDIANDENG:
                ESP_LOGI(TAG, "turn off lignt");
                 led_bar_is31x_deinit(led_handle);
                break;
            default:
                ESP_LOGI(TAG, "not supportint mode");
                break;
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}


const char *url;
char str[100];
char str0[]={"/sdcard/r"};

char str1[100]={};

char str2[]={".wav"};
char tmp[64]={"0"};
static void esp_initialize_sntp(void)
{
    ESP_LOGI(TAG_sntp, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
       sntp_setservername(0, "ntp1.aliyun.com");
    sntp_setservername(1, "210.72.145.44");		// 国家授时中心服务器 IP 地址
    sntp_setservername(2, "1.cn.pool.ntp.org");        

    sntp_init();
}

void esp_wait_sntp_sync(void)
{
    char strftime_buf[64];
   

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;

    while (timeinfo.tm_year < (2019 - 1900)) {
        ESP_LOGD(TAG_sntp, "Waiting for system time to be set... (%d)", ++retry);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    // set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG_sntp, "The current date/time in Shanghai is: %s", strftime_buf);
}

#ifdef sonic
void record_sonic()
{
    audio_pipeline_handle_t pipeline_rec = NULL;
    audio_pipeline_handle_t pipeline_play = NULL;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    /**
     * For the Recorder:
     * We will setup I2S and get audio at sample rates 16000Hz, 16-bits, 1 channel.
     * And the audio stream will be encoded with Wav encoder.
     * Then the audio stream will be written to SDCARD.
     */
    ESP_LOGI(TAG, "[1.1] Initialize recorder pipeline");
    pipeline_rec = audio_pipeline_init(&pipeline_cfg);
    pipeline_play = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[1.2] Create audio elements for recorder pipeline");
    audio_element_handle_t i2s_reader_el = create_i2s_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_READER);
    audio_element_handle_t wav_encoder_el = create_wav_encoder();
    audio_element_handle_t fatfs_writer_el = create_fatfs_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_WRITER);

    ESP_LOGI(TAG, "[1.3] Register audio elements to recorder pipeline");
    audio_pipeline_register(pipeline_rec, i2s_reader_el, "i2s_reader");
    audio_pipeline_register(pipeline_rec, wav_encoder_el, "wav_encoder");
    audio_pipeline_register(pipeline_rec, fatfs_writer_el, "file_writer");
    const char *link_rec[3] = {"i2s_reader", "wav_encoder", "file_writer"};
    audio_pipeline_link(pipeline_rec, &link_rec[0], 3);

    /**
     * For the Playback:
     * We will read the recorded file processed by sonic.
     */
    ESP_LOGI(TAG, "[2.2] Create audio elements for playback pipeline");
    audio_element_handle_t fatfs_reader_el = create_fatfs_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_READER);
    audio_element_handle_t wav_decoder_el = create_wav_decoder();
    audio_element_handle_t sonic_el = create_sonic();
    audio_element_handle_t i2s_writer_el = create_i2s_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_WRITER);

    ESP_LOGI(TAG, "[2.3] Register audio elements to playback pipeline");
    audio_pipeline_register(pipeline_play, fatfs_reader_el, "file_reader");
    audio_pipeline_register(pipeline_play, wav_decoder_el, "wav_decoder");
    audio_pipeline_register(pipeline_play, sonic_el, "sonic");
    audio_pipeline_register(pipeline_play, i2s_writer_el, "i2s_writer");
    
    const char *link_play[4] = {"file_reader", "wav_decoder", "sonic", "i2s_writer"};
    audio_pipeline_link(pipeline_play, &link_play[0], 4);

    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
      led_on();
    ESP_LOGW(TAG, "Press [Rec] to start recording");
    bool is_modify_speed = true;
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        if ((int)msg.data == get_input_mode_id()) {
            if ((msg.cmd == PERIPH_BUTTON_LONG_PRESSED)
                || (msg.cmd == PERIPH_BUTTON_PRESSED)) {
                is_modify_speed = !is_modify_speed;
                if (is_modify_speed) {
                    ESP_LOGI(TAG, "The speed of audio file is changed");
                } else {
                    ESP_LOGI(TAG, "The pitch of audio file is changed");
                }
            }
            continue;
        }
        if ((int)msg.data == get_input_rec_id()) {
            if (msg.cmd == PERIPH_BUTTON_PRESSED) {
                i++;
         ESP_LOGI(TAG, "########################=%d",i);
                 // url="/sdcard/rec.wav";
               
            char str[100]={"0"};
	        itoa(i,str1,10);
           // str1[0]=i;
            strcat (str,str0);
            strcat (str,str1);
              strcat (str,tmp);//时间
            strcat (str,str2);
            url=str;
        ESP_LOGI(TAG, "*************************=%s",url);

       //  fun_i();
                //using LOGE to make the log color different
                ESP_LOGE(TAG, "Now recording, release [Rec] to STOP");
                audio_pipeline_stop(pipeline_play);
                audio_pipeline_wait_for_stop(pipeline_play);
                audio_pipeline_terminate(pipeline_play);
                audio_pipeline_reset_ringbuffer(pipeline_play);
                audio_pipeline_reset_elements(pipeline_play);

                /**
                 * Audio Recording Flow:
                 * [codec_chip]-->i2s_stream--->wav_encoder-->fatfs_stream-->[sdcard]
                 */
                ESP_LOGI(TAG, "Setup file path to save recorded audio");
                i2s_stream_set_clk(i2s_reader_el, SAMPLE_RATE, BITS, CHANNEL);
                audio_element_set_uri(fatfs_writer_el,url );
                audio_pipeline_run(pipeline_rec);
            } else if (msg.cmd == PERIPH_BUTTON_RELEASE || msg.cmd == PERIPH_BUTTON_LONG_RELEASE) {
                ESP_LOGI(TAG, "START Playback");
                audio_pipeline_stop(pipeline_rec);
                audio_pipeline_wait_for_stop(pipeline_rec);
                audio_pipeline_terminate(pipeline_rec);
                audio_pipeline_reset_ringbuffer(pipeline_rec);
                audio_pipeline_reset_elements(pipeline_rec);

                /**
                 * Audio Playback Flow:
                 * [sdcard]-->fatfs_stream-->wav_decoder-->sonic-->i2s_stream-->[codec_chip]
                 */
                ESP_LOGI(TAG, "Setup file path to read the wav audio to play");
                i2s_stream_set_clk(i2s_writer_el, SAMPLE_RATE, BITS, CHANNEL);
                audio_element_set_uri(fatfs_reader_el, url);
                if (is_modify_speed) {
                    sonic_set_pitch_and_speed_info(sonic_el, 1.0f, SONIC_SPEED);
                } else {
                    sonic_set_pitch_and_speed_info(sonic_el, SONIC_PITCH, 1.0f);
                }
                audio_pipeline_run(pipeline_play);
            }
        }
    }
led_off();
    ESP_LOGI(TAG, "[ 4 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline_rec);
    audio_pipeline_wait_for_stop(pipeline_rec);
    audio_pipeline_terminate(pipeline_rec);
    audio_pipeline_stop(pipeline_play);
    audio_pipeline_wait_for_stop(pipeline_play);
    audio_pipeline_terminate(pipeline_play);

    audio_pipeline_unregister(pipeline_play, fatfs_reader_el);
    audio_pipeline_unregister(pipeline_play, wav_decoder_el);
    audio_pipeline_unregister(pipeline_play, i2s_writer_el);

    audio_pipeline_unregister(pipeline_rec, i2s_reader_el);
    audio_pipeline_unregister(pipeline_rec, sonic_el);
    audio_pipeline_unregister(pipeline_rec, wav_encoder_el);
    audio_pipeline_unregister(pipeline_rec, fatfs_writer_el);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline_rec);
    audio_pipeline_remove_listener(pipeline_play);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline_rec);
    audio_pipeline_deinit(pipeline_play);

    audio_element_deinit(fatfs_reader_el);
    audio_element_deinit(wav_decoder_el);
    audio_element_deinit(i2s_writer_el);

    audio_element_deinit(i2s_reader_el);
    audio_element_deinit(sonic_el);
    audio_element_deinit(wav_encoder_el);
    audio_element_deinit(fatfs_writer_el);
}
#endif


/*
     PCM文件转WAV文件     
 *     * @param inPcmFilePath  输入PCM文件路径  
 *    * @param outWavFilePath 输出WAV文件路径    
 *  * @param sampleRate   *   采样率，例如44100   
 *   * @param channels       声道数 单声道：1或双声道：2    
 *  * @param bitNum         采样位数，8或16   
 *    //采样字节byte率           
 *  long byteRate = sampleRate * channels * bitNum / 8;    
 *         in = new FileInputStream(inPcmFilePath);        
 *     out = new FileOutputStream(outWavFilePath);         
 *    //PCM文件大小       
 *      long totalAudioLen = in.getChannel().size();      
 *       //总大小，由于不包括RIFF和WAV，所以是44 - 8 = 36，在加上PCM文件大小         
 *    long totalDataLen = totalAudioLen + 36;
 */

/* These types MUST be 16-bit */
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

/* These types MUST be 32-bit */
typedef long			LONG;
typedef unsigned long	DWORD;
// Define WAVE File Header
typedef struct tagHXD_WAVFLIEHEAD
{
char RIFFNAME[4];
DWORD nRIFFLength;//4
char WAVNAME[4];
char FMTNAME[4];
DWORD nFMTLength;//4
WORD nAudioFormat;//2
WORD nChannleNumber;//2
DWORD nSampleRate;//4
DWORD nBytesPerSecond;//4
WORD nBytesPerSample;//2
WORD   nBitsPerSample;//2
char   DATANAME[4];
DWORD  nDataLength;//4
}tagHXD_WAVFLIEHEAD;

//typedef tagHXD_WAVFLIEHEAD HXD_WAVFLIEHEAD;

//char *pcm="/sdcard/16k.pcm";
//char *wav="/sdcard/16k.wav";

int pcm_to_wav(const char *pcm, const char*wav)
{
  
 
// 开始准备WAV的文件头
tagHXD_WAVFLIEHEAD DestionFileHeader;
DestionFileHeader.RIFFNAME[0] = 'R';
DestionFileHeader.RIFFNAME[1] = 'I';
DestionFileHeader.RIFFNAME[2] = 'F';
DestionFileHeader.RIFFNAME[3] = 'F';

DestionFileHeader.WAVNAME[0] = 'W';
DestionFileHeader.WAVNAME[1] = 'A';
DestionFileHeader.WAVNAME[2] = 'V';
DestionFileHeader.WAVNAME[3] = 'E';

DestionFileHeader.FMTNAME[0] = 'f';
DestionFileHeader.FMTNAME[1] = 'm';
DestionFileHeader.FMTNAME[2] = 't';
DestionFileHeader.FMTNAME[3] = 0x20;
DestionFileHeader.nFMTLength =16;  //  表示 FMT 的长度
DestionFileHeader.nAudioFormat = 1; //这个表示a lawPCM

DestionFileHeader.DATANAME[0] = 'd';
DestionFileHeader.DATANAME[1] = 'a';
DestionFileHeader.DATANAME[2] = 't';
DestionFileHeader.DATANAME[3] = 'a';
DestionFileHeader.nBitsPerSample = 16;//2
DestionFileHeader.nBytesPerSample =2; //2   
DestionFileHeader.nSampleRate =16000;   //
DestionFileHeader.nBytesPerSecond = 32000;
DestionFileHeader.nChannleNumber = 1;

// 文件头的基本部分
int nFileLen = 0;
int nSize = sizeof(DestionFileHeader);
printf("nsize=====%d\n",nSize);

    ESP_LOGI(TAG, "Reading file");
   FILE* f = fopen(pcm, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
   struct stat buf ;
       if ( stat(pcm, &buf ) < 0 )
      {
         perror( "stat" );
             return ;
     }
     printf("文件大小:%ld\n", buf.st_size );
int k= buf.st_size;
FILE *fp_d=NULL;
fp_d= fopen(wav, "wb+");
if (fp_d == NULL)
  printf("文件%s没有找到!\n",wav);



int nWrite =fwrite(&DestionFileHeader, 1, nSize, fp_d);
if (nWrite != nSize)
{
  fclose(f);
  fclose(fp_d);
  
}
 
while(1)
{
 
  int nRead = fread(readBuf, 1,1024, f);
  if (nRead >0)
  {
   fwrite(readBuf,1, nRead, fp_d);
  }
  bzero(readBuf,1024);
  nFileLen += nRead;
  printf("文件%d!\n",nFileLen);
if(nFileLen>=k)
{
    break;
}
}

fseek(fp_d, 0L, SEEK_SET);

DestionFileHeader.nRIFFLength = nFileLen - 8 +nSize;//头长
DestionFileHeader.nDataLength = nFileLen;//文件长
nWrite =fwrite(&DestionFileHeader, 1, nSize, fp_d);
if (nWrite != nSize)
{
  fclose(f);
  fclose(fp_d);
 
}

fclose(f);
fclose(fp_d);
printf("over\n");



}

void app_main()
{
//app_fat();
//led_init();
//periph_is31fl3216_test();
//leed();
//esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
//set = esp_periph_set_init(&periph_cfg);
//audio_board_sdcard_init(esp_periph_set_init(&periph_cfg));
led_handle = led_bar_is31x_init();
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();
    asr_main();



}

//#endif