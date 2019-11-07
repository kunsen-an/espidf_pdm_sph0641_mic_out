#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "audio_example_file.h"

#include <driver/dac.h>
#include <driver/rtc_io.h>

#define FREQ_KILO    96

#if FREQ_KILO == (192)
#include "v192k.h"
#elif FREQ_KILO == (96)
#include "v96k.h"
#elif FREQ_KILO == (48) 
#include "v48k.h"
#else   /* 16 */
#include "variatio.h"
#endif


/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/

#define PDM_MIC 1          /* PDM microphone */
#define I2S_MIC (!PDM_MIC) /* I2S microphone */

#define PDM_OUT 1         /* PDM output */
#define DAC_OUT (!PDM_OUT) /* DAC output */

#define PCM8_MONO_DATA 0                    /* Sound file is unsigned 8bit mono 16kHz */
#define PCM16_STEREO_DATA (!PCM8_MONO_DATA) /* Sound file is signed 16bit stereo 16kHz */
#define PCM8_OFFSET 0x8000                  /* unsigned pcm data to signed */
#define PCM16_OFFSET 0                      /* signed pcm data to signed */

#define MONO_TO_STEREO 1 /* input is mono and output is stereo*/

#if PDM_MIC
#define MIC_OFFSET 0 /* MIC  offset */
#elif I2S_MIC
#define MIC_OFFSET 0x8000 /* MIC offset */
#else
#error "MIC is not defined"
#endif
#define MIC_MASK 0xffff /* MIC mask */

#if PDM_OUT
#define OUT_OFFSET 0 /* OUT  offset */
#elif DAC_OUT
#define OUT_OFFSET 0x8000 /* OUT scale offset */
#else
#error "OUT is not defined"
#endif
#define OUT_MASK 0xffff /* OUT mask */
#define OUT_SCALE 4     /* OUT amplification scale */


static const char *TAG = "pdm_mic/out";
#define PARTITION_NAME "storage"

//enable record sound and save in flash
#define RECORD_IN_FLASH_EN (1)
//enable replay recorded sound in flash
#define REPLAY_FROM_FLASH_EN (1)
//enable play recorded sound in array
#define REPLAY_FROM_FILE_EN (1)
//enable repeat replay
#define REPEAT_REPLAY_EN (1)

//i2s number
#define EXAMPLE_PDM_NUM_RX (0)     // 0 for PDM
#define EXAMPLE_I2S_PDM_NUM_TX (0) // 0 for PDM/DAC
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE (FREQ_KILO * 1000)
//PDM sample rate
#define EXAMPLE_PDM_SAMPLE_RATE (FREQ_KILO * 1000) 

//PDM RX data bits
#define EXAMPLE_PDM_SAMPLE_BITS_RX (16) // PDM 16bit input
//PDM TX data bits
#define EXAMPLE_PDM_SAMPLE_BITS_TX (16) // PDM 16bit output
//i2s TX data bits
#define EXAMPLE_I2S_SAMPLE_BITS_TX (16) // I2S 16bit output

//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG (0)
// 8bit (BYTE) unit or 16bit unit
#define DEBUG_DISPLAY_BYTE (0)
//I2S read buffer length
#define EXAMPLE_I2S_READ_BUF_LEN (4 * 1024)
//I2S write buffer length
#define EXAMPLE_I2S_WRITE_BUF_LEN (4 * 1024)

#if MONO_TO_STEREO
//PDM tx data format
#define EXAMPLE_CHANNEL_FORMAT_PDM_TX (I2S_CHANNEL_FMT_RIGHT_LEFT)
//I2S tx data format
#define EXAMPLE_CHANNEL_FORMAT_I2S_TX (I2S_CHANNEL_FMT_RIGHT_LEFT)
#else
//PDM tx data format
#define EXAMPLE_CHANNEL_FORMAT_PDM_TX (I2S_CHANNEL_FMT_ONLY_RIGHT)
//I2S tx data format
#define EXAMPLE_CHANNEL_FORMAT_I2S_TX (I2S_CHANNEL_FMT_ONLY_RIGHT)
#endif

//max length of write data
#if PCM8_MONO_DATA
#if MONO_TO_STEREO
#define EXAMPLE_MAX_WRITE_DATA (EXAMPLE_I2S_WRITE_BUF_LEN/4)
#else
#define EXAMPLE_MAX_WRITE_DATA (EXAMPLE_I2S_WRITE_BUF_LEN/2)
#endif
#else
#define EXAMPLE_MAX_WRITE_DATA (EXAMPLE_I2S_WRITE_BUF_LEN)
#endif


//I2S tx channel number
#define EXAMPLE_PDM_CHANNEL_NUM_TX ((EXAMPLE_CHANNEL_FORMAT_PDM_TX < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S rx data format
#if MONO_TO_STEREO
#define EXAMPLE_CHANNEL_FORMAT_PDM_RX (I2S_CHANNEL_FMT_ONLY_RIGHT)
#else
#define EXAMPLE_CHANNEL_FORMAT_PDM_RX (I2S_CHANNEL_FMT_RIGHT_LEFT)
#endif

//I2S rx channel number
#define EXAMPLE_PDM_CHANNEL_NUM_RX ((EXAMPLE_CHANNEL_FORMAT_PDM_RX < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))

//flash record size, for recording 2 seconds' data
#define RECORDING_PERIOD 2
#define FLASH_RECORD_SIZE (EXAMPLE_PDM_CHANNEL_NUM_RX * EXAMPLE_PDM_SAMPLE_RATE * EXAMPLE_PDM_SAMPLE_BITS_RX / 8 * RECORDING_PERIOD)
#define FLASH_ERASE_SIZE (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
//sector size of flash
#define FLASH_SECTOR_SIZE (0x1000)
//flash read / write address
#define FLASH_ADDR (0x200000)

#define MIC_DMA_BUF_COUNT           4
#define MIC_DMA_BUF_LEN             1024
#define PDM_OUT_DMA_BUF_COUNT       12
#define PDM_OUT_DMA_BUF_LEN         1024
#define I2S_DAC_OUT_DMA_BUF_COUNT   2
#define I2S_DAC_OUT_DMA_BUF_LEN     1024
/**
 * @brief PDM microphone input mode init.
 */
void example_pdm_mic_init()
{
    i2s_config_t pdm_config_rx = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM, // Master, RX, PDM
        .sample_rate = EXAMPLE_PDM_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_PDM_SAMPLE_BITS_RX, // 16bit
        .channel_format = EXAMPLE_CHANNEL_FORMAT_PDM_RX,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB, //pcm data format
        .dma_buf_count = MIC_DMA_BUF_COUNT,              // number of buffers, 128 max.
        .dma_buf_len = MIC_DMA_BUF_LEN,                  // size of each buffer
        .use_apll = 0,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 // Interrupt level 1
    };

    i2s_pin_config_t pin_pdm_config_rx = {
        .ws_io_num = GPIO_NUM_19,
//        .data_in_num = GPIO_NUM_35,       // Lolin D32 cannot use GPIO_NUM_35 as pdm input
        .data_in_num = GPIO_NUM_34, // ADC1 pins [GPIO 32, GPIO 34, GPIO 36 (VP), GPIO 39 (VN)] can be used on Lolin D32
    };

    /* PDM: I2S_NUM_0 */
    i2s_driver_install(EXAMPLE_PDM_NUM_RX, &pdm_config_rx, 0, NULL);
    i2s_set_pin(EXAMPLE_PDM_NUM_RX, &pin_pdm_config_rx);
}

// uninstall PDM microhone (input) driver
void example_pdm_mic_uninit()
{
    i2s_driver_uninstall(EXAMPLE_PDM_NUM_RX);
}

/**
 * @brief PDM output mode init.
 */
void example_pdm_out_init()
{
    i2s_config_t pdm_config_tx = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_PDM, // Master, TX, PDM
        .sample_rate = EXAMPLE_PDM_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_PDM_SAMPLE_BITS_TX, // 16bit
        .channel_format = EXAMPLE_CHANNEL_FORMAT_PDM_TX,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB, //pcm data format
        .dma_buf_count = PDM_OUT_DMA_BUF_COUNT,                             // number of buffers, 128 max.
        .dma_buf_len = PDM_OUT_DMA_BUF_LEN,                             // size of each buffer
        .use_apll = 0,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 // Interrupt level 1
    };

    /* PDM: I2S_NUM_0 */
    i2s_driver_install(EXAMPLE_I2S_PDM_NUM_TX, &pdm_config_tx, 0, NULL);

    i2s_pin_config_t pin_pdm_config_tx = {
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = GPIO_NUM_25,
        .data_in_num = I2S_PIN_NO_CHANGE};
    i2s_set_pin(EXAMPLE_I2S_PDM_NUM_TX, &pin_pdm_config_tx);

    // switch to PDM  from DAC
    rtc_gpio_deinit(GPIO_NUM_25);
    dac_output_disable(DAC_CHANNEL_1);
}

/**
 * @brief I2S DAC mode init.
 */

void example_i2s_dac_out_init()
{
    i2s_config_t i2s_config_tx = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS_TX,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = EXAMPLE_CHANNEL_FORMAT_I2S_TX,
        .intr_alloc_flags = 0,
        .dma_buf_count = I2S_DAC_OUT_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DAC_OUT_DMA_BUF_LEN,

        .use_apll = 0,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 // Interrupt level 1
    };

    /* DAC: I2S_NUM_0 */
    //install and start i2s driver
    i2s_driver_install(EXAMPLE_I2S_PDM_NUM_TX, &i2s_config_tx, 0, NULL);
    //init DAC pad
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
}

// uninstall output driver
void example_output_uninit()
{
    i2s_driver_uninstall(EXAMPLE_I2S_PDM_NUM_TX);
}

void example_mic_init()
{
#if PDM_MIC
    example_pdm_mic_init();
#elif I2S_MIC
    example_i2s_mic_init();
#else
#error "No Microphone"
#endif
}
/*
 * @brief erase flash for recording
 */
void example_erase_flash()
{
#if RECORD_IN_FLASH_EN
    printf("Erasing flash \n");
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
#else
    printf("Skip flash erasing...\n");
#endif
}

/**
 * @brief debug buffer data
 */
void example_disp_buf(uint8_t *buf, int length)
{
#if EXAMPLE_I2S_BUF_DEBUG
    printf("======\n");
    unsigned short *bp = (unsigned short *)buf;
    for (int i = 0; i < length; i++)
    {
        printf("%04x ", (bp[i] << 0) & 0xffff);
        if ((i + 1) % 8 == 0)
        {
            printf("\n");
        }
    }
    printf("======\n");
#endif
}

/**
 * @brief Scale data for I2S DMA output.
 *        DAC can only output 8bit data value.
 *        I2S DMA will still send 16 bit (or 32bit data), the highest 8bit contains DAC data.
 */
// unsigned 8bit mono data to 16bit
int example_i2s_unsigned_pcm8_mono_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
    uint32_t j = 0;

    for (int i = 0; i < len; i++)
    {
        int16_t value = ((int16_t)s_buff[i]) << 8; // 8bit -> 16bit
        value = value + PCM8_OFFSET;               // to signed
        value = value + OUT_OFFSET;                // to output format
        value = value & OUT_MASK;

        // little endian
        d_buff[j++] = value & 0xff;
        d_buff[j++] = (value >> 8) & 0xff;

#if MONO_TO_STEREO
        // another channel
        d_buff[j++] = value & 0xff;
        d_buff[j++] = (value >> 8) & 0xff;
#endif
    }
    return (j);
}

// signed 16bit stereo data to 16bit
int example_i2s_signed_pcm16_stereo_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
    uint32_t j = 0;

    for (int i = 0; i < len; i += 2)
    {
        int16_t value = (s_buff[i + 1] << 8) | s_buff[i];
        value = value + PCM16_OFFSET; // to signed
        value = value + OUT_OFFSET;   // to output format
        value = value & OUT_MASK;

        // little endian
        d_buff[j++] = value & 0xff;
        d_buff[j++] = (value >> 8) & 0xff;

#if MONO_TO_STEREO
        // another channel
        d_buff[j++] = value & 0xff;
        d_buff[j++] = (value >> 8) & 0xff;

        i += 2; // skip another channel source data
#endif
    }
    return (j);
}

int example_i2s_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
#if PCM8_MONO_DATA
    return example_i2s_unsigned_pcm8_mono_data_scale(d_buff, s_buff, len);
#elif PCM16_STEREO_DATA
    return example_i2s_signed_pcm16_stereo_data_scale(d_buff, s_buff, len);
#else
#error "NO DATA"
#endif
}

/**
 * @brief Scale data for output from Mic.
 *        Data from PDM Mic are 16bit width.
 *        PDM can output 16 bit data.
 *        DAC can only output 8 bit data.
 *        Scale each 16bit Mic data to output.
 */
// 16bit data to 16bit
int_least64_t example_pdm_mic_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint16_t mic_value = 0;
    uint16_t out_value = 0;

    for (int i = 0; i < len; i += 2)
    {
        // little endian
        mic_value = (s_buff[i + 1] << 8) | s_buff[i]; // signed value
        mic_value = mic_value & MIC_MASK;
        out_value = mic_value - MIC_OFFSET;

        out_value = (out_value << OUT_SCALE); // amplification
        out_value = (out_value + OUT_OFFSET) & OUT_MASK;

        // little endian
        d_buff[j++] = out_value & 0xff;        // LSB 8bit
        d_buff[j++] = (out_value >> 8) & 0xff; // MSB 8bit

#if MONO_TO_STEREO
        // another channel
        d_buff[j++] = out_value & 0xff;        // LSB 8bit
        d_buff[j++] = (out_value >> 8) & 0xff; // MSB 8bit
#else
        i += 2;
#endif
    }
    return j;
}

// flash out PDM DMA buffer
void flash_out_pdm_dma_buffer()
{
        uint8_t *i2s_write_buff = (uint8_t *)calloc(EXAMPLE_I2S_WRITE_BUF_LEN, sizeof(uint8_t));
        uint8_t *empty_buff = (uint8_t *)calloc(EXAMPLE_MAX_WRITE_DATA, sizeof(uint8_t));
        size_t bytes_written;

        int i2s_wr_len = example_i2s_data_scale(i2s_write_buff, empty_buff, EXAMPLE_MAX_WRITE_DATA);
        for ( int i = 0 ; i < PDM_OUT_DMA_BUF_COUNT ; i++ ) {
            i2s_write(EXAMPLE_I2S_PDM_NUM_TX, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
        }
        free(empty_buff);
        free(i2s_write_buff);
}

/**
 * @brief PDM Mic to PDM/DAC example
 *        1. Erase flash
 *        2. Record audio from mic and save in flash
 *        3. Read flash and replay the sound via PDM/DAC
 *        4. Play an example audio data
 *        5. Loop back to step 3
 */
void example_pdm_out(void *arg)
{
    //    printf("example_pdm_out starts.\n");
    example_mic_init();

    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    else
    {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
#if RECORD_IN_FLASH_EN
    //1. Erase flash
    example_erase_flash();
#endif
    int i2s_read_len = EXAMPLE_I2S_READ_BUF_LEN;
    size_t bytes_written;
#if RECORD_IN_FLASH_EN
    int flash_wr_size = 0;
    size_t bytes_read;

    //2. Record audio from PDM and save in flash
    char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
    while (flash_wr_size < FLASH_RECORD_SIZE)
    {
        //read data from I2S bus, in this case, from Mic.
        i2s_read(EXAMPLE_PDM_NUM_RX, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        //save original data from I2S(Mic) into flash.
        esp_partition_write(data_partition, flash_wr_size, i2s_read_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;
        example_disp_buf((uint8_t *)i2s_read_buff, 16);
        ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    }
    free(i2s_read_buff);
    i2s_read_buff = NULL;
#endif
    example_pdm_mic_uninit();

#if PDM_OUT
    example_pdm_out_init();
#elif DAC_OUT
    example_i2s_dac_out_init();
#else
#error "OUT is not specified"
#endif
#if REPEAT_REPLAY_EN
    while (1)
#endif
    {
        //3. Read flash and replay the sound via DAC
#if REPLAY_FROM_FLASH_EN
        uint8_t *flash_read_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
        uint8_t *i2s_write_data = (uint8_t *)calloc((MONO_TO_STEREO ? 2 : 1) * i2s_read_len, sizeof(char));

        for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE)
        {
            //read PDM original data from flash
            esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
            //process data and scale to 8bit for I2S DAC.
            example_disp_buf((uint8_t *)flash_read_buff, 16);
            int i2s_wr_len = example_pdm_mic_data_scale(i2s_write_data, flash_read_buff, FLASH_SECTOR_SIZE);
            //send data
            i2s_write(EXAMPLE_I2S_PDM_NUM_TX, i2s_write_data, i2s_wr_len, &bytes_written, portMAX_DELAY);
            example_disp_buf((uint8_t *)i2s_write_data, 16);
            printf("playing: %d %%\n", rd_offset * 100 / flash_wr_size);
        }
        free(flash_read_buff);
        free(i2s_write_data);
        flash_read_buff = NULL;
        i2s_write_data = NULL;

        // flash out DMA buffer to stop sounds
        flash_out_pdm_dma_buffer();
#endif
        vTaskDelay(1000 / portTICK_PERIOD_MS); // pause
#if REPLAY_FROM_FILE_EN
        //4. Play an example audio file(file format: 8bit/16khz/single channel)
        printf("Playing file example: \n");
        int offset = 0;

        // PCM data
#if PCM8_MONO_DATA
        int tot_size = sizeof(audio_table);
        uint8_t *start = (uint8_t *)audio_table;
#elif PCM16_STEREO_DATA
#if FREQ_KILO == (16)
        int tot_size = sizeof(Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_03_18_raw);
        uint8_t *start = (uint8_t *)Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_03_18_raw;
#elif FREQ_KILO == (48)
        int tot_size = sizeof(Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_48k_raw);
        uint8_t *start = (uint8_t *)Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_48k_raw;
#elif FREQ_KILO == (96)
        int tot_size = sizeof(Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_96k_raw);
        uint8_t *start = (uint8_t *)Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_96k_raw;
#elif FREQ_KILO == (192)
        int tot_size = sizeof(Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_192k_raw);
        uint8_t *start = (uint8_t *)Kimiko_Ishizaka___02___Variatio_1_a_1_Clav_0334_834_192k_raw;
#else
#error "NO DATA for EXAMPLE_PDM_SAMPLE_RATE"
#endif
#else
#error "NO DATA"
#endif
        uint8_t *i2s_write_buff = (uint8_t *)calloc(EXAMPLE_I2S_WRITE_BUF_LEN, sizeof(uint8_t));
        while (offset < tot_size)
        {
            int play_len = ((tot_size - offset) > EXAMPLE_MAX_WRITE_DATA) ? EXAMPLE_MAX_WRITE_DATA : (tot_size - offset);

            int i2s_wr_len = example_i2s_data_scale(i2s_write_buff, (start + offset), play_len);
            i2s_write(EXAMPLE_I2S_PDM_NUM_TX, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
            offset += play_len;
            example_disp_buf((uint8_t *)i2s_write_buff, 16);
        }

        free(i2s_write_buff);
        i2s_write_buff = NULL;

        // flash out DMA buffer to stop sounds
        flash_out_pdm_dma_buffer();

        printf("Playing done.\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif
    }
    example_output_uninit();
    heap_caps_check_integrity(MALLOC_CAP_8BIT,true);
    vTaskDelete(NULL);
}

esp_err_t app_main()
{
    esp_log_level_set("PDM", ESP_LOG_INFO);
    xTaskCreate(example_pdm_out, "example_pdm_out", 1024 * 2, NULL, 5, NULL);
    return ESP_OK;
}
