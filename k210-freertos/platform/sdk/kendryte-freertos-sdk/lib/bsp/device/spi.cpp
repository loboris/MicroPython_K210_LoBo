/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <fpioa.h>
#include <hal.h>
#include <kernel/driver_impl.hpp>
#include <atomic.h>
#include <math.h>
#include <semphr.h>
#include <spi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysctl.h>
#include <utility.h>
#include <devices.h>
#include <syslog.h>
#include <printf.h>
#include "gpiohs.h"

#define SPI_SLAVE_MUTEX_WAIT_TIME   20

extern uint64_t dmac_intstatus;

// transmission length (frames) bellow which DMA transfer is not used
// used only by SPI master
// during the non-DMA transfer interrupts are disabled !
// LoBo: make this global variable so it can be changed during runtime
uint32_t SPI_TRANSMISSION_THRESHOLD  = 0x4000; // byte threshold above which a DMA transfer is used
uint32_t SPI_SLAVE_TRANS_THRESHOLD   = 128;    // byte threshold above which a SPI Slave DMA transfer is used

using namespace sys;

#define SPI_DMA_BLOCK_TIME          1000UL          // DMA transfer block time in ms
#define SPI_SLAVE_INFO              "K210 v01.03"   // !must be exactly 11 bytes (+ending 0)!
#define CYCLES_PER_US               (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000)
#define MAX_SLAVE_TRANSFER_SIZE     65536
#define SLAVE_OUTPUT_ENABLE_BITS    10
#define SLAVE_COMMAND_LENGTH        16
#define SLAVE_READ_BUFFER_LENGTH    (SLAVE_COMMAND_LENGTH+2)
#define SLAVE_COMMAND_STATUS_LENGTH 12


// !!! SPI SLAVE HAS ONLY 8 32-BIT FIFO ENTRIES !!!
#define SLAVE_FIFO_SIZE             8

/* SPI Controller */

#define TMOD_MASK (3 << tmod_off_)
#define TMOD_VALUE(value) (value << tmod_off_)
#define COMMON_ENTRY \
    semaphore_lock locker(free_mutex_);

typedef enum
{
    IDLE,
    COMMAND,
    DEINIT,
} spi_slave_status_e;

typedef struct _spi_slave_instance
{
    size_t data_bit_length;
    volatile spi_slave_status_e status;
    volatile spi_slave_command_t command;
    volatile void *databuff_ptr;
    uint32_t databuff_size;
    uint32_t databuff_ro_size;
    int32_t  mosi;
    int32_t  miso;
    int32_t  cs_io_num;
    int32_t  handshake;
    uint32_t crc_speed;
    bool     in_transfer; // true while in slave ISR
    uint8_t  cmd_data[SLAVE_COMMAND_LENGTH];
    uint8_t  read_data[SLAVE_READ_BUFFER_LENGTH];
    uint8_t  cmd_status[SLAVE_COMMAND_STATUS_LENGTH];

    uintptr_t dma;
    SemaphoreHandle_t   dma_event;
    SemaphoreHandle_t   slave_event;
    TaskHandle_t        slave_task_handle;
    QueueHandle_t       slave_queue;
    QueueHandle_t       slave_mutex;        // used to protect access to spi slave buffer
} spi_slave_instance_t;

static fpioa_io_config_t FUNC_SPI_SLAVE_MISO = {
        .ch_sel  = FUNC_SPI_SLAVE_D0,
        .ds      = 0xf,
        .oe_en   = 1,
        .oe_inv  = 1,
        .do_sel  = 0,
        .do_inv  = 0,
        .pu      = 0,
        .pd      = 0,
        .resv0   = 0,
        .sl      = 0,
        .ie_en   = 0,
        .ie_inv  = 0,
        .di_inv  = 0,
        .st      = 0,
        .resv1   = 0,
        .pad_di  = 0
    };

volatile gpiohs_t* const gpiohs = (volatile gpiohs_t*)GPIOHS_BASE_ADDR;

static const char *TAG = "[SPI_DRIVER]";
static const char *SLAVE_TAG = "[SPI_SLAVE_DRIVER]";

class k_spi_device_driver;

class k_spi_driver : public spi_driver, public static_object, public free_object_access
{
public:
    k_spi_driver(uintptr_t base_addr, sysctl_clock_t clock, sysctl_dma_select_t dma_req, uint8_t mod_off, uint8_t dfs_off, uint8_t tmod_off, uint8_t frf_off)
        : spi_(*reinterpret_cast<volatile spi_t *>(base_addr)), clock_(clock), dma_req_(dma_req), mod_off_(mod_off), dfs_off_(dfs_off), tmod_off_(tmod_off), frf_off_(frf_off)
    {
    }

    virtual void install() override
    {
        free_mutex_ = xSemaphoreCreateMutex();
        sysctl_clock_disable(clock_);
    }

    virtual void on_first_open() override
    {
        sysctl_clock_enable(clock_);
    }

    virtual void on_last_close() override
    {
        sysctl_clock_disable(clock_);
    }

    virtual object_ptr<spi_device_driver> get_device(spi_mode_t mode, spi_frame_format_t frame_format, uint32_t chip_select_mask, uint32_t data_bit_length) override;

    bool set_xip_mode(k_spi_device_driver &device, bool enable); // LoBo
    void master_config_half_duplex(k_spi_device_driver &device, int8_t mosi, int8_t miso); // LoBo
    double set_clock_rate(k_spi_device_driver &device, double clock_rate);
    void set_endian(k_spi_device_driver &device, uint32_t endian);
    int read(k_spi_device_driver &device, gsl::span<uint8_t> buffer);
    int write(k_spi_device_driver &device, gsl::span<const uint8_t> buffer);
    int transfer_full_duplex(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer);
    int transfer_sequential(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer);
    int transfer_sequential_with_delay(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer, uint16_t delay);
    int read_write(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer);
    void fill(k_spi_device_driver &device, uint32_t instruction, uint32_t address, uint32_t value, size_t count);

    // LoBo: slave driver changed ---------------------------------------------------------------------------------------------
    //-----------------
    void slave_deinit()
    {
        spi_.imr = 0x00;
        pic_set_irq_enable(IRQN_SPI_SLAVE_INTERRUPT, 0);
        atomic_set(&slave_instance_.status, DEINIT);
        int tmo = 5000;
        while (slave_instance_.slave_task_handle != NULL) {
            if ((tmo % 100) == 0) xSemaphoreGive(slave_instance_.slave_event);
            vTaskDelay(5);
            tmo -= 5;
            if (tmo == 0) {
                vTaskDelete(slave_instance_.slave_task_handle);
                slave_instance_.slave_task_handle = NULL;
                LOGW(SLAVE_TAG, "Slave task forced to stop");
                break;
            }
        }
        spi_.dmacr = 0x00;
        spi_.txftlr = 0;
        spi_.rxftlr = 0;
        spi_.ssienr = 0x00;
        sysctl_clock_disable(SYSCTL_CLOCK_SPI2);
        sysctl_reset(SYSCTL_RESET_SPI2);
        vSemaphoreDelete(slave_instance_.dma_event);
        vSemaphoreDelete(slave_instance_.slave_event);
        dma_close(slave_instance_.dma);
        vSemaphoreDelete(slave_instance_.slave_mutex);
    }

    //---------------
    void slave_init()
    {

    }

    //--------------------------------------------------------------------------------------------------------------------------------
    bool slave_config(void *data, uint32_t len, uint32_t ro_len, QueueHandle_t queue, int priority, int mosi, int miso, int handshake)
    {
        slave_instance_.slave_mutex = xSemaphoreCreateMutex();
        if (slave_instance_.slave_mutex == NULL) {
            return false;
        }
        slave_instance_.dma_event = xSemaphoreCreateBinary();
        if (slave_instance_.dma_event == NULL) {
            vSemaphoreDelete(slave_instance_.slave_mutex);
            return false;
        }
        slave_instance_.slave_event = xSemaphoreCreateBinary();
        if (slave_instance_.slave_event == NULL) {
            vSemaphoreDelete(slave_instance_.slave_mutex);
            vSemaphoreDelete(slave_instance_.dma_event);
            return false;
        }
        slave_instance_.cs_io_num = fpioa_get_io_by_function(FUNC_SPI_SLAVE_SS);
        if (slave_instance_.cs_io_num < 0) {
            vSemaphoreDelete(slave_instance_.slave_mutex);
            vSemaphoreDelete(slave_instance_.dma_event);
            vSemaphoreDelete(slave_instance_.slave_event);
            return false;
        }
        slave_instance_.status = IDLE;
        slave_instance_.databuff_ptr = data;
        slave_instance_.databuff_size = len;
        slave_instance_.databuff_ro_size = ro_len;
        slave_instance_.data_bit_length = 8;
        slave_instance_.dma = dma_open_free();
        slave_instance_.slave_queue = queue;
        slave_instance_.mosi = mosi;
        slave_instance_.miso = miso;
        slave_instance_.handshake = handshake;
        slave_instance_.in_transfer = false;
        memset(slave_instance_.cmd_data, 0, SLAVE_COMMAND_LENGTH);
        slave_set_read_buffer(slave_instance_.cmd_data);
        uint16_t size = (len > 1000) ? 1000 : len;
        uint64_t tend, tstart = read_csr64(mcycle);
        uint32_t crc_test = hal_crc16((const void*)(slave_instance_.databuff_ptr), size, 0);
        tend = read_csr64(mcycle);
        crc_test = (((tend - tstart)  * 1000) / CYCLES_PER_US) * (1000 / size);
        slave_instance_.crc_speed = crc_test;

        sysctl_reset(SYSCTL_RESET_SPI2);
        sysctl_clock_enable(SYSCTL_CLOCK_SPI2);
        sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI2, 4);
        LOGV(SLAVE_TAG, "Clock = %u", sysctl_clock_get_freq(SYSCTL_CLOCK_SPI2));

        spi_.ssienr = 0x00;  // disable spi
        /* Setup SPI:
         * SPI_FRF (SpiFrame Format)              = Standard
         * DFS_32  (Data Frame Size in 32bitmode) = 7 (8-bit mode)
         * CFS     (Control Frame size)           = 0
         * SRL     (Shift register loop)          = 0
         * SLV_OE  (Slave Output Enable)          = 1 (disabled)
         * TMOD    (Transfer Mode)                = 0 (Transmit&receive)
         * SCPOL   (Serial Clock Polarity)        = 0 (Inactive state is low)
         * SCPH    (Serial Clock Phase)           = 0 (Clock toggles in middle of first bit)
         * FRF     (Frame Format)                 = 0 (Motorola SPI Frame Format)
         * DFS     (Data Frame Size)              = 0
         */
        spi_.ctrlr0 = (0x0 << mod_off_) | (0x1 << SLAVE_OUTPUT_ENABLE_BITS) | (7 << dfs_off_);
        spi_.dmatdlr = 0x01;    // DMA Transmit Data Level = 1
        spi_.dmardlr = 0x00;    // DMA Receive Data Level = 0+1
        spi_.dmacr = 0x00;      // DMA Control Register = 0 (DMA disabled)
        spi_.txftlr = 0;        // Transmit FIFO Threshold Level = 0
        spi_.rxftlr = 0;        // Receive FIFO Threshold level = 0 (Interrupt on 1 byte received)

        // Configure interrupt controller for SPI interrupts
        pic_set_irq_priority(IRQN_SPI_SLAVE_INTERRUPT, 4);
        pic_set_irq_enable(IRQN_SPI_SLAVE_INTERRUPT, 1);
        pic_set_irq_handler(IRQN_SPI_SLAVE_INTERRUPT, spi_slave_irq, this);

        // Create SPI Slave task
        slave_instance_.slave_task_handle = NULL;
        auto ret = xTaskCreate(spi_slave_process_task, "SPI Slave Task", configMINIMAL_STACK_SIZE, this, priority+2, &slave_instance_.slave_task_handle);
        if (ret != pdTRUE) {
            vSemaphoreDelete(slave_instance_.slave_mutex);
            vSemaphoreDelete(slave_instance_.dma_event);
            vSemaphoreDelete(slave_instance_.slave_event);
            return false;
        }

        spi_.imr = 0x10;    // spi interrupt enable
        spi_.ssienr = 0x01; // spi enable

        set_handshake(slave_instance_.handshake, GPIO_PV_HIGH);
        return true;
    }

    // Set the handshake line (if used) to requested level
    // or pulse it low for requested time in us
    //--------------------------------------
    bool slave_set_handshake(uint16_t value)
    {
        bool ret = false;
        bool in_transfer = atomic_read(&slave_instance_.in_transfer);
        if ((!in_transfer) && (slave_instance_.handshake >= 0)) {
            if (value < 2) set_handshake(slave_instance_.handshake, value);
            else {
                if(value > 500) value = 500;
                uint64_t tend = read_csr64(mcycle) + (value * CYCLES_PER_US);
                uint64_t tcurrent = tend;
                set_handshake(slave_instance_.handshake, GPIO_PV_LOW);
                while (tcurrent <= tend) {
                    tcurrent = read_csr64(mcycle);
                }
                set_handshake(slave_instance_.handshake, GPIO_PV_HIGH);
                ret = true;
            }
        }
        return ret;
    }

    //------------------------------------------------------------------
    bool slave_set_buffer(uint8_t *buffer, uint32_t addr, uint32_t size)
    {
        if (xSemaphoreTake(slave_instance_.slave_mutex, 10) != pdTRUE) return false;
        memcpy((uint8_t *)slave_instance_.databuff_ptr+addr, buffer, size);
        xSemaphoreGive(slave_instance_.slave_mutex);
        return true;
    }

    //-----------------------------------------
    bool slave_set_read_buffer(uint8_t *buffer)
    {
        if (atomic_read(&slave_instance_.in_transfer)) return false;
        spi_.imr = 0x00;    // Disable SPI interrupt

        if (buffer) memcpy(slave_instance_.read_data, buffer, SLAVE_COMMAND_LENGTH);
        else memcpy(slave_instance_.read_data, (uint8_t *)slave_instance_.databuff_ptr, SLAVE_COMMAND_LENGTH);

        uint16_t crc = hal_crc16((const void*)(slave_instance_.read_data), SLAVE_COMMAND_LENGTH, 0);
        slave_instance_.read_data[SLAVE_READ_BUFFER_LENGTH-2] = crc & 0xff;
        slave_instance_.read_data[SLAVE_READ_BUFFER_LENGTH-1] = crc >> 8;
        spi_.imr = 0x10;    // Enable SPI interrupt
        return true;
    }

    //---------------------------------------------------------------------
    bool slave_fill_buffer(uint8_t fill_byte, uint32_t addr, uint32_t size)
    {
        if (xSemaphoreTake(slave_instance_.slave_mutex, 10) != pdTRUE) return false;
        memset((uint8_t *)slave_instance_.databuff_ptr+addr, fill_byte, size);
        xSemaphoreGive(slave_instance_.slave_mutex);
        return true;
    }

    //------------------------------------------------------------------
    bool slave_get_buffer(uint8_t *buffer, uint32_t addr, uint32_t size)
    {
        if (xSemaphoreTake(slave_instance_.slave_mutex, 10) != pdTRUE) return false;
        memcpy(buffer, (uint8_t *)slave_instance_.databuff_ptr+addr, size);
        xSemaphoreGive(slave_instance_.slave_mutex);
        return true;
    }

private:

    //--------------------------------------------------------------------------
    static bool wait_cs(int32_t cs_io_num, uint32_t tmo, gpio_pin_value_t level)
    {
        bool res = false;
        uint64_t time_end = read_csr64(mcycle) + (tmo * CYCLES_PER_US);
        // wait for CS level
        while (read_csr64(mcycle) < time_end) {
            if (fpioa_get_pad_di(cs_io_num) == level) {
                res = true;
                break;
            }
        }
        return res;
    }

    //------------------------------
    static void wait_us(uint16_t us)
    {
        uint64_t time_end = read_csr64(mcycle) + (us * CYCLES_PER_US);
        while (read_csr64(mcycle) < time_end) {
            ;
        }
    }

    // ------------------------------------------------------------------------------------
    // SPI SLAVE uses only one pin (mosi) for both receive and transfer (FUNC_SPI_SLAVE_D0)
    // If 'miso' pin is defined, we are switching SPI slave pin to it when sending data
    //--------------------------------------------
    static void switch_to_miso(int mosi, int miso)
    {
        if (miso >= 0) {
            // disconnect 'mosi' pin and
            // setup 'miso' pin for FUNC_SPI_SLAVE_D0
            fpioa_set_function(mosi, FUNC_RESV0);
            fpioa_set_io(miso, &FUNC_SPI_SLAVE_MISO);
        }
    }

    //--------------------------------------------
    static void switch_to_mosi(int mosi, int miso)
    {
        if (miso >= 0) {
            // disconnect 'miso' pin and
            // setup 'mosi' for FUNC_SPI_SLAVE_D0 again
            fpioa_set_function(miso, FUNC_RESV0);
            fpioa_set_function(mosi, FUNC_SPI_SLAVE_D0);
        }
    }

    //-----------------------------------------------
    static void set_handshake(int32_t pin, int value)
    {
        if (pin >= 0) {
            uint32_t org = (*gpiohs->output_val.u32) & ~(1 << pin);
            *gpiohs->output_val.u32 = org | (value & (1 << pin));
        }
    }

    /* ===== SPI Slave command processing task =====
     * Runs with the priority higher than the main task
     *
     */
    //------------------------------------------------
    static void spi_slave_process_task(void *userdata)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        while (1)
        {
            if (xSemaphoreTake(driver.slave_instance_.slave_event, portMAX_DELAY) == pdTRUE)
            {
                // slave interrupt occurred
                spi_slave_status_e status = atomic_read(&driver.slave_instance_.status);
                if (status == IDLE) {
                    // Full command block received
                    atomic_cas((volatile int *)&driver.slave_instance_.status, IDLE, COMMAND);
                    spi_slave_command_mode(userdata);
                    atomic_set(&driver.slave_instance_.in_transfer, false);
                }
                else if (status == DEINIT) {
                    auto &command = driver.command();
                    memset((void *)&driver.slave_instance_.command, 0, sizeof(spi_slave_command_t));
                    command.err = SPI_CMD_ERR_EXIT;
                    if (driver.slave_instance_.slave_queue != NULL)
                        xQueueSend(driver.slave_instance_.slave_queue, (void *)&command, 0);
                    break;
                }
            }
        }
        driver.slave_instance_.slave_task_handle = NULL;
        vTaskDelete(NULL);
    }

    /* ========= SPI Slave ISR =========
     * Executed during the Idle phase whenever SPI slave receives a new byte
     * 16 bytes command block is then received
     */
    //----------------------------------
    static void spi_slave_irq(void *ctx)
    {
        uint64_t time_start = read_csr64(mcycle);
        auto &driver = *reinterpret_cast<k_spi_driver *>(ctx);
        auto &spi_handle = driver.spi();

        *reinterpret_cast<volatile uint32_t *>(spi_handle.icr); // clear interrupt

        spi_slave_status_e status = atomic_read(&driver.slave_instance_.status);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t i;
        uint32_t size = SLAVE_COMMAND_LENGTH;
        uint32_t fifo_len, to_read;
        uint8_t *buf = driver.slave_instance_.cmd_data;
        bool is_ok = true;

        atomic_set(&driver.slave_instance_.in_transfer, true);
        int mask = taskENTER_CRITICAL_FROM_ISR();
        spi_handle.imr = 0x00;  // disable SPI interrupts

        // save the 1st byte
        //while (spi_handle.rxflr == 0) {
        //    ;
        //}
        *buf = (uint8_t)spi_handle.dr[0];
        if ((*buf == SPI_CMD_READ_TRANS) || (*buf == SPI_CMD_STATUS_TRANS)) {
            // --- Master read transaction ---
            uint8_t *tbuf = driver.slave_instance_.read_data;
            size = SLAVE_READ_BUFFER_LENGTH;
            if (*buf == SPI_CMD_STATUS_TRANS) {
                tbuf = driver.slave_instance_.cmd_status;
                size = SLAVE_COMMAND_STATUS_LENGTH;
            }
            uint32_t idx = 0;
            uint32_t to_send;
            // wait for transaction to finish
            if (wait_cs(driver.slave_instance_.cs_io_num, 50, GPIO_PV_HIGH)) {
                spi_handle.ssienr = 0x00; // spi disable
                // set output mode
                switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);
                set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);                 // Set Transmit only mode
                set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 0 << SLAVE_OUTPUT_ENABLE_BITS); // Enable SPI Slave output
                spi_handle.ssienr = 0x01; // spi enable
                while (size) {
                    to_send = SLAVE_FIFO_SIZE - spi_handle.txflr;
                    if (size < to_send) to_send = size;
                    for (int i=0; i<to_send; i++) {
                        spi_handle.dr[0] = tbuf[idx++];
                    }
                    size -= to_send;
                    if (size == 0) break;
                }
                // wait for transaction to finish
                wait_cs(driver.slave_instance_.cs_io_num, 100, GPIO_PV_HIGH);

                // set input mode
                spi_handle.ssienr = 0x00; // spi disable
                switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
                set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 1 << SLAVE_OUTPUT_ENABLE_BITS); // Disable SPI Slave output
                set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 2 << driver.tmod_off_);                 // Set Receive only mode
                spi_handle.rxftlr = 0;      // Enable interrupt after 1st received byte
                spi_handle.imr = 0x10;      // enable spi interrupt
                spi_handle.ssienr = 0x01;   // spi enable

                xSemaphoreGiveFromISR(driver.slave_instance_.slave_event, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
            }

            taskEXIT_CRITICAL_FROM_ISR(mask);
            atomic_set(&driver.slave_instance_.in_transfer, false);
        }
        else {
            size--;
            buf++;
            // The 1st byte was received, receive the remaining 15 bytes of the command block
            while (size) {
                fifo_len = spi_handle.rxflr;
                to_read = (fifo_len > size) ? size : fifo_len;
                for (i=0; i<to_read; i++) {
                    *buf++ = (uint8_t)spi_handle.dr[0];
                }
                size -= to_read;
                // Check if CS line is high
                if ((size > 0) && (fpioa_get_pad_di(driver.slave_instance_.cs_io_num))) {
                    // timeout
                    is_ok = false;
                    break;
                }
            }

            if ((is_ok) && (status == IDLE)) {
                // save the command block start time
                // the actual transaction start time is 1 byte transfer time less than this
                driver.slave_instance_.command.command_time = read_csr64(mcycle);
                spi_handle.ssienr = 0x00; // spi disable
                taskEXIT_CRITICAL_FROM_ISR(mask);
                // Command block received, inform the SPI Slave task
                driver.slave_instance_.command.start_time = time_start-500;
                xSemaphoreGiveFromISR(driver.slave_instance_.slave_event, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
            }
            else {
                taskEXIT_CRITICAL_FROM_ISR(mask);
                // error receiving command block, re-enable spi interrupt
                atomic_set(&driver.slave_instance_.in_transfer, false);
                spi_handle.imr = 0x10;
            }
        }
    }

    // Wait for DMA transfer initiated from command handler to finish
    //-----------------------------------------------------------------------------
    static void _wait_slave_transfer_finish(void *userdata, volatile uint32_t fifo)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);

        if (xSemaphoreTake(driver.slave_instance_.dma_event, 100 / portTICK_PERIOD_MS) != pdTRUE) {
            driver.slave_instance_.command.end_time = read_csr64(mcycle);
            driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
            LOGD(SLAVE_TAG, "DMAC error status=0x%08lX", dmac_intstatus);
        }
        else {
            // wait until transfer finished (CS = high)
            if (!wait_cs(driver.slave_instance_.cs_io_num, 100, GPIO_PV_HIGH)) {
                driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
            }
            driver.slave_instance_.command.end_time = read_csr64(mcycle);
        }
    }

    /*
    // This is executed from DMAC when the transfer has ended
    //--------------------------------------------------
    static void slave_completion_handler(void *userdata)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        dma_stop(driver.slave_instance_.dma);
    }
    */

    //--------------------------------------------------------------------------------------
    static uint32_t *alloc_buffer(uint32_t size, void *src_buf, uint16_t crc, uint8_t dummy)
    {
        uint32_t sz = (size & 0xfffffffc) + dummy + 8;
        uint32_t *buf = (uint32_t *)pvPortMalloc(sz * sizeof(uint32_t));

        if (buf) {
            if (src_buf) {
                for (int i=0; i<dummy; i++) {
                    buf[i] = i;
                }
                uint8_t *srcbuf = (uint8_t *)src_buf;
                for (int i=0; i<size; i++) {
                    buf[i+dummy] = (uint32_t)srcbuf[i];
                }
                buf[dummy+size] = (uint32_t)(crc & 0xFF);
                buf[dummy+size+1] = (uint32_t)((crc >> 8) & 0xFF);
            }
            if ((buf+sz) > (uint32_t *)0x80600000) buf -= 0x40000000;
        }
        return buf;
    }

    //--------------------------------------
    static void prepare_send(void *userdata)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();

        switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

        //spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x0 << SLAVE_OUTPUT_ENABLE_BITS) | (7 << driver.dfs_off_);
        set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 0 << SLAVE_OUTPUT_ENABLE_BITS); // Enable SPI Slave output
        set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);                 // Set Transmit only mode
        spi_handle.dmacr = 0x02;    // Transmit DMA Enable
        spi_handle.imr = 0x00;      // Disable SPI interrupts
        spi_handle.ssienr = 0x01;   // SPI enable

        dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_ + 1);

        driver.slave_instance_.command.transfer_time = read_csr64(mcycle);
    }

    //--------------------------------------------------------------------------
    static void receive_no_dma(void *userdata, uint8_t *buffer, uint32_t length)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();

        vTaskEnterCritical();

        // prepare receive
        set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 1 << SLAVE_OUTPUT_ENABLE_BITS); // Disable SPI Slave output
        set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 2 << driver.tmod_off_);                 // Set Receive only mode
        spi_handle.dmacr = 0x00;    // Receive DMA Disable
        spi_handle.imr = 0x00;      // Disable SPI interrupts
        spi_handle.ssienr = 0x01;   // SPI enable

        driver.slave_instance_.command.transfer_time = read_csr64(mcycle);
        set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

        uint32_t size = length;
        uint32_t idx = 0;
        uint32_t to_receive;
        if (wait_cs(driver.slave_instance_.cs_io_num, 200, GPIO_PV_LOW)) {
            while (size) {
                to_receive = spi_handle.rxflr;
                if (size < to_receive) to_receive = size;
                for (int i=0; i<to_receive; i++) {
                    buffer[idx++] = spi_handle.dr[0];
                }
                size -= to_receive;
                // check for timeout
                if (fpioa_get_pad_di(driver.slave_instance_.cs_io_num) != GPIO_PV_LOW) {
                    driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
                    break;
                }
            }
            if (driver.slave_instance_.command.err !=  SPI_CMD_ERR_TIMEOUT) {
                if (!wait_cs(driver.slave_instance_.cs_io_num, 100, GPIO_PV_HIGH)) {
                    driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
                }
            }
        }
        else {
            driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
        }
        driver.slave_instance_.command.end_time = read_csr64(mcycle);

        vTaskExitCritical();
    }

    //---------------------------------------------------------
    static void set_confirm_data(void *userdata, uint8_t *sbuf)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &command = driver.command();

        // confirm receiving data by sending the status 9-byte block
        // 1st 6 bytes of the received command block + status + crc16
        for (int i=0; i<command.dummy_bytes; i++) {
            sbuf[i] = i;
        }
        uint8_t *status = sbuf +  command.dummy_bytes;
        memcpy(status, driver.slave_instance_.cmd_data, 6);
        status[6] = command.err;
        uint16_t st_crc = hal_crc16((const void*)(status), 7, 0);
        status[7] = st_crc & 0xff;
        status[8] = st_crc >> 8;
    }

    //-----------------------------------------------------------------------
    static void send_no_dma(void *userdata, uint8_t *buffer, uint32_t length)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();
        auto &command = driver.command();

        uint8_t *buf = buffer;
        uint32_t size = length;
        if (buffer == NULL) {
            // confirm receiving data by sending the status 9-byte block
            // 1st 6 bytes of the received command block + status + crc16
            uint8_t sbuf[9+command.dummy_bytes];
            set_confirm_data(userdata, sbuf);
            buf = sbuf;
            size = 9 + command.dummy_bytes;
        }

        if (driver.slave_instance_.miso < 0) wait_us(20);
        vTaskEnterCritical();
        switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

        // Switch to spi slave output mode
        set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 0 << SLAVE_OUTPUT_ENABLE_BITS); // Enable SPI Slave output
        set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);                 // Set Transmit only mode
        spi_handle.dmacr = 0x00;    // Transmit DMA Disable
        spi_handle.imr = 0x00;      // Disable SPI interrupts
        spi_handle.ssienr = 0x01;   // SPI enable

        uint32_t idx = 0;
        uint32_t fifo_len = SLAVE_FIFO_SIZE - spi_handle.txflr;
        uint32_t to_send = (fifo_len > size) ? size : fifo_len ;
        for (int i=0; i<to_send; i++) {
            spi_handle.dr[0] = buf[idx++];
        }
        size -= to_send;

        set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

        if (wait_cs(driver.slave_instance_.cs_io_num, 200, GPIO_PV_LOW)) {
            while (size) {
                fifo_len = SLAVE_FIFO_SIZE - spi_handle.txflr;
                to_send = (fifo_len > size) ? size : fifo_len ;
                for (int i=0; i<to_send; i++) {
                    spi_handle.dr[0] = buf[idx++];
                }
                size -= to_send;
                // check if CS still active
                if (fpioa_get_pad_di(driver.slave_instance_.cs_io_num) != GPIO_PV_LOW) {
                    driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
                    break;
                }
            }
            if (!wait_cs(driver.slave_instance_.cs_io_num, 200, GPIO_PV_HIGH)) {
                driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
            }
        }
        else {
            driver.slave_instance_.command.err =  SPI_CMD_ERR_TIMEOUT;
        }

        switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
        vTaskExitCritical();
    }

    /* Command structure:
     * ------------------
     *  0       command code & options
     *  1 -  3  20-bit address & dummy bytes
     *  4 -  5  16_bit length
     *  6 - 13  user data (8 bytes)
     * 14 - 15  16-bit crc
     * -------------------
     *
     * Different format for: SPI_CMD_WRSTAT & SPI_CMD_WRSTAT_CONFIRM
     * -------------------------------------------------------------
     *  0       user data command code
     *  1       user data command type (b0-b3) & dummy bytes (b4-b7)
     *  2 - 13  user data (12 bytes)
     * 14 - 15  16-bit crc
     * -------------------------------------------------------------
     */

    //=======================================
    // 8-byte command from master is received
    //================================================
    static void spi_slave_command_mode(void *userdata)
    {
        // === 16-byte command structure is received in 'slave_instance_.cmd_data' ===
        // SPI is disabled !

        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();
        auto &command = driver.command();

        uint64_t time_us = driver.slave_instance_.command.start_time;
        uint64_t time_end = driver.slave_instance_.command.command_time;
        memset((void *)&driver.slave_instance_.command, 0, sizeof(spi_slave_command_t));
        if ((driver.slave_instance_.cmd_data[0] == SPI_CMD_READ_TRANS) || (driver.slave_instance_.cmd_data[0] == SPI_CMD_STATUS_TRANS)) {
            // special case, processed in ISR
            command.cmd = driver.slave_instance_.cmd_data[0];
            // Send the status (transaction result) to the main task
            if (driver.slave_instance_.slave_queue != NULL)
                xQueueSend(driver.slave_instance_.slave_queue, (void *)&command, 0);
            atomic_set(&driver.slave_instance_.status, IDLE);
            return;
        }

        driver.slave_instance_.command.start_time = time_us;
        driver.slave_instance_.command.command_time = time_end;

        uint16_t cmd_crc = 0xFFFF;
        uint16_t rcv_crc;
        int stop_signal = 0;
        const volatile void *srcs[1];
        volatile void *dests[1];
        volatile uint8_t spi_slave_dummy_bytes[8];
        uint32_t *tmp_buffer = NULL;
        uint32_t data_length = 0;
        bool with_crc = false;
        bool wr_confirm = false;

        memset((void *)spi_slave_dummy_bytes, 0xFF, 8);
        // set the command start time
        command.transfer_time = command.start_time;
        command.end_time = 0;
        command.crc16 = 0xFFFF;

        // Check command crc
        cmd_crc = hal_crc16((const void*)driver.slave_instance_.cmd_data, 14, 0);
        rcv_crc = driver.slave_instance_.cmd_data[14] | (driver.slave_instance_.cmd_data[15] << 8);
        if ( rcv_crc != cmd_crc) {
            command.err =  SPI_CMD_ERR_CSUM;
            goto exit;
        }

        command.cmd = driver.slave_instance_.cmd_data[0] & 0x0F;
        if ((command.cmd == SPI_CMD_WRSTAT) || (command.cmd == SPI_CMD_WRSTAT_CONFIRM)) {
            command.opt = driver.slave_instance_.cmd_data[1] & 0x0F;
            command.dummy_bytes = driver.slave_instance_.cmd_data[1] >> 4;
            memcpy((void *)command.user_data, driver.slave_instance_.cmd_data+2, 12);
        }
        else {
            command.opt = (driver.slave_instance_.cmd_data[0] >> 4);
            command.addr = driver.slave_instance_.cmd_data[1] | (driver.slave_instance_.cmd_data[2] << 8) | ((driver.slave_instance_.cmd_data[3] & 0x0F) << 16);
            command.len = (driver.slave_instance_.cmd_data[4] | (driver.slave_instance_.cmd_data[5] << 8)) + 1;
            command.dummy_bytes = driver.slave_instance_.cmd_data[3] >> 4;
            with_crc = (command.opt & 0x01);
            wr_confirm = (command.opt & 0x02);
        }
        data_length = command.len + command.dummy_bytes;

        if ((command.len > driver.slave_instance_.databuff_size) || (command.len > MAX_SLAVE_TRANSFER_SIZE)) {
            command.err =  SPI_CMD_ERR_LENGTH;
            goto exit;
        }

        // === Command ok, process it =================================================================
        //xSemaphoreTake(driver.slave_instance_.dma_event, 0); // ignore any received events
        stop_signal = 1;            // finish DMA transfer after all bytes transfered
        command.err = SPI_CMD_ERR_OK;

        //-----------------------------------------------------------------------------
        if ((command.cmd == SPI_CMD_WRSTAT) || (command.cmd == SPI_CMD_WRSTAT_CONFIRM))
        {
            if (command.cmd == SPI_CMD_WRSTAT_CONFIRM) {
                send_no_dma(userdata, NULL, 0);
            }
        }
        //-------------------------------------
        else if (command.cmd == SPI_CMD_RDSTAT)
        {
            // ----------------------------------------------
            // Send 1st 4 bytes from SPI Slave buffer + crc16
            // ----------------------------------------------

            if ((command.len != 4) || (!with_crc)) {
                command.err =  SPI_CMD_ERR_LENGTH;
                goto dummysend;
            }
            uint8_t buf[6+command.dummy_bytes];

            xSemaphoreTake(driver.slave_instance_.slave_mutex, 2);
            memcpy(buf+command.dummy_bytes, (uint8_t *)driver.slave_instance_.databuff_ptr, 4);
            xSemaphoreGive(driver.slave_instance_.slave_mutex);

            command.crc16 = hal_crc16((const void*)(buf+command.dummy_bytes), 4, 0);
            buf[4+command.dummy_bytes] = command.crc16 & 0xff;
            buf[5+command.dummy_bytes] = command.crc16 >> 8;

            send_no_dma(userdata, buf, 6+command.dummy_bytes);
        }
        //------------------------------------------------
        else if ((command.cmd == SPI_CMD_READ_DATA_BLOCK))
        {
            // -----------------------------------------------------------------------------------------
            // Send requested number of bytes from requested slave buffer address
            // If the crc is requested, append 2-bytes checksum to the sent data
            // 2-bytes CRC16 in case of SPI_CMD_READ_DATA_BLOCK_CSUM is NOT INCLUDED in requested length
            // -----------------------------------------------------------------------------------------

            // check if the requested data are inside the slave buffer
            if (command.addr >= driver.slave_instance_.databuff_size) {
                command.err =  SPI_CMD_ERR_ADDRESS;
                goto dummysend;
            }
            if ((command.addr + command.len) > driver.slave_instance_.databuff_size) {
                command.err =  SPI_CMD_ERR_LENGTH;
                goto dummysend;
            }
            xSemaphoreTake(driver.slave_instance_.slave_mutex, 2);
            if (with_crc) {
                // 2-byte checksum will be appended to the data
                command.crc16 = hal_crc16((const void*)((uint8_t *)driver.slave_instance_.databuff_ptr + command.addr), command.len, 0);
                data_length += 2;
            }

            if (command.len > SPI_SLAVE_TRANS_THRESHOLD) {
                // allocate temporary buffer for transfer and copy data from slave buffer into it
                tmp_buffer = alloc_buffer(command.len, (void *)((uint8_t *)driver.slave_instance_.databuff_ptr+command.addr), command.crc16, command.dummy_bytes);
                xSemaphoreGive(driver.slave_instance_.slave_mutex);
                if (tmp_buffer == NULL) {
                    command.err = SPI_CMD_ERR_MEMORY;
                    goto dummysend;
                }

                srcs[0] = (void *)tmp_buffer;
                dests[0] = (void *)&spi_handle.dr[0];

                prepare_send(userdata);

                dma_loop_async(driver.slave_instance_.dma, srcs, 1, dests, 1, true, false, 4, data_length, 1,
                        NULL, userdata, driver.slave_instance_.dma_event, &stop_signal);
                set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

                _wait_slave_transfer_finish(userdata, spi_handle.txflr);

                switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
            }
            else {
                // short blocks are sent without using DMA
                uint8_t buf[data_length];
                for (int i=0; i<command.dummy_bytes; i++) {
                    buf[i] = i;
                }
                memcpy(buf+command.dummy_bytes, (uint8_t *)driver.slave_instance_.databuff_ptr + command.addr, command.len);
                xSemaphoreGive(driver.slave_instance_.slave_mutex);
                buf[command.dummy_bytes+command.len] = command.crc16 & 0xFF;
                buf[command.dummy_bytes+command.len+1] = command.crc16 >> 8;

                send_no_dma(userdata, buf, data_length);
            }
        }
        //-------------------------------------------------------------------------------------------------------------------------------------------------
        else if (command.cmd == SPI_CMD_WRITE_DATA_BLOCK)
        {
            // ------------------------------------------------------------------------------------------
            // Receive requested number of bytes into requested slave buffer address
            // 2-bytes CRC16 in case of SPI_CMD_WRITE_DATA_BLOCK_CSUM is NOT INCLUDED in requested length
            // if CRC is provided, received data is checked for crc match before saving to spi buffer
            // ------------------------------------------------------------------------------------------

            if (with_crc) data_length += 2;
            else data_length = command.len;

            // check if the requested data address is inside slave buffer
            if (command.addr >= (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) {
                command.err = SPI_CMD_ERR_ADDRESS;
                goto dummyreceive;
            }
            // check data length
            if ( (command.len > (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) ||
                 ((command.addr + command.len) > (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) ) {
                command.err = SPI_CMD_ERR_LENGTH;
                goto dummyreceive;
            }

            uint8_t *recv_buf = NULL;
            if (command.len > SPI_SLAVE_TRANS_THRESHOLD) {
                // allocate temporary buffer for transfer
                tmp_buffer = alloc_buffer(command.len, NULL, 0, 0);
                if (tmp_buffer == NULL) {
                    driver.slave_instance_.command.err = SPI_CMD_ERR_MEMORY;
                    goto dummyreceive;
                }
                recv_buf = (uint8_t *)tmp_buffer;

                srcs[0] = (void *)&spi_handle.dr[0];
                dests[0] = tmp_buffer;

                // prepare receive
                set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 1 << SLAVE_OUTPUT_ENABLE_BITS); // Disable SPI Slave output
                set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 2 << driver.tmod_off_);                 // Set Receive only mode
                spi_handle.dmacr = 0x01;    // Receive DMA Enable
                spi_handle.imr = 0x00;      // Disable SPI interrupts
                spi_handle.ssienr = 0x01;   // SPI enable

                dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_);
                spi_handle.dmacr = 0x01;

                command.transfer_time = read_csr64(mcycle);

                dma_loop_async(driver.slave_instance_.dma, srcs, 1, dests, 1, false, true, 4, data_length, 1,
                        NULL, userdata, driver.slave_instance_.dma_event, &stop_signal);

                set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

                _wait_slave_transfer_finish(userdata, spi_handle.rxflr);
            }
            else {
                // short blocks are received without using DMA
                uint8_t buf[data_length];
                recv_buf = buf;
                receive_no_dma(userdata, buf, data_length);
            }

            if (command.err == SPI_CMD_ERR_OK) {
                if (tmp_buffer != NULL) {
                    // data received in 32-bit temporary buffer, convert it to 8-bit
                    uint8_t *buf8 = (uint8_t *)tmp_buffer;
                    uint32_t *buf32 = (uint32_t *)tmp_buffer;
                    for (int i=0; i<(command.len + 2); i++) {
                        buf8[i] = (uint8_t)buf32[i];
                    }
                }

                if (with_crc) {
                    // received data with checksum, test checksum
                    uint16_t rcsum = *((uint8_t *)((uint8_t *)recv_buf + driver.slave_instance_.command.len));
                    rcsum |= *((uint8_t *)((uint8_t *)recv_buf + driver.slave_instance_.command.len + 1)) << 8;
                    // calculated checksum
                    command.crc16 = hal_crc16((const void*)recv_buf, command.len, 0);
                    if (command.crc16 == rcsum) {
                        // copy received data to the slave buffer
                        xSemaphoreTake(driver.slave_instance_.slave_mutex, 2);
                        memcpy((uint8_t *)((uint8_t *)driver.slave_instance_.databuff_ptr+command.addr), recv_buf, command.len);
                        xSemaphoreGive(driver.slave_instance_.slave_mutex);
                    }
                    else command.err = SPI_CMD_ERR_DATA_CSUM;
                }
                else {
                    // copy received data to the slave buffer without check
                    xSemaphoreTake(driver.slave_instance_.slave_mutex, 2);
                    memcpy((uint8_t *)((uint8_t *)driver.slave_instance_.databuff_ptr+command.addr), recv_buf, command.len);
                    xSemaphoreGive(driver.slave_instance_.slave_mutex);
                }
            }

            if (wr_confirm) {
                memset(driver.slave_instance_.cmd_status, 0, SLAVE_COMMAND_STATUS_LENGTH);
                set_confirm_data(userdata, driver.slave_instance_.cmd_status);
                uint16_t crc = hal_crc16((const void*)(driver.slave_instance_.cmd_status), SLAVE_COMMAND_STATUS_LENGTH-2, 0);
                driver.slave_instance_.cmd_status[SLAVE_COMMAND_STATUS_LENGTH-2] = crc & 0xff;
                driver.slave_instance_.cmd_status[SLAVE_COMMAND_STATUS_LENGTH-1] = crc >> 8;
            }
        }
        //----------------------------------------
        else if (command.cmd == SPI_CMD_READ_INFO)
        {
            // -----------------------------------------------------------------
            // Send the SPI slave info to the master (19 bytes + 2-byte checksum
            // -----------------------------------------------------------------

            if ((command.len != 22) || (!with_crc)) {
                command.err =  SPI_CMD_ERR_LENGTH;
                goto dummysend;
            }

            uint8_t buf[24+command.dummy_bytes] = { 0 };
            uint8_t * slave_info = buf + command.dummy_bytes;
            for (int i=0; i<command.dummy_bytes; i++) {
                buf[i] = i;
            }
            memcpy((uint8_t *)slave_info, SPI_SLAVE_INFO, 11);
            slave_info[12] = driver.slave_instance_.databuff_size & 0xff;
            slave_info[13] = (driver.slave_instance_.databuff_size >> 8) & 0xff;
            slave_info[14] = (driver.slave_instance_.databuff_size >> 16) & 0xff;
            slave_info[15] = driver.slave_instance_.databuff_ro_size & 0xff;
            slave_info[16] = (driver.slave_instance_.databuff_ro_size >> 8) & 0xff;
            slave_info[17] = (driver.slave_instance_.databuff_ro_size >> 16) & 0xff;
            slave_info[18] = driver.slave_instance_.crc_speed & 0xff;
            slave_info[19] = (driver.slave_instance_.crc_speed >> 8) & 0xff;
            slave_info[20] = (driver.slave_instance_.crc_speed >> 16) & 0xff;
            slave_info[21] = (uint8_t)(driver.slave_instance_.handshake >= 0);
            command.crc16 = hal_crc16((const void*)(slave_info), 22, 0);
            slave_info[22] = command.crc16 & 0xff;
            slave_info[23] = command.crc16 >> 8;

            send_no_dma(userdata, buf, 24+command.dummy_bytes);
        }
        //---- unhandled command, return to idle mode ----
        else
        {
            command.cmd = SPI_CMD_MAX;
            command.err = SPI_CMD_ERR_COMMAND;
        }
        // === Command processing finished ===

exit:
        if (tmp_buffer) {
            if (tmp_buffer < (uint32_t *)0x80000000) tmp_buffer += 0x40000000;
            vPortFree(tmp_buffer);
        }
        // set command execution time
        if (command.end_time == 0) command.end_time = read_csr64(mcycle);

        // Send the status (transaction result) to the main task
        if (driver.slave_instance_.slave_queue != NULL)
            xQueueSend(driver.slave_instance_.slave_queue, (void *)&command, 0);

        // ===== Back to IDLE mode ===============================================================================

        atomic_set(&driver.slave_instance_.status, IDLE);
        spi_handle.imr = 0x00;      // disable interrupts (should already be disabled)
        // dummy read, clear spi fifo
        while (spi_handle.rxflr > 0) {
            data_length = spi_handle.dr[0];
        }
        spi_handle.ssienr = 0x00;   // SPI disable (should be already disabled)
        // prepare for receive
        set_bit_mask(&spi_handle.ctrlr0, 1 << SLAVE_OUTPUT_ENABLE_BITS, 1 << SLAVE_OUTPUT_ENABLE_BITS); // Disable SPI Slave output
        set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 2 << driver.tmod_off_);                 // Set Receive only mode
        spi_handle.dmacr = 0x00;    // Disable DMA Receive&Transmit
        spi_handle.imr = 0x00;      // Disable SPI interrupts
        spi_handle.rxftlr = 0;      // Enable interrupt after 1st received byte

        spi_handle.imr = 0x10;      // enable spi interrupt

        set_handshake(driver.slave_instance_.handshake, GPIO_PV_HIGH);
        if (driver.slave_instance_.command.err != SPI_CMD_ERR_EXIT) spi_handle.ssienr = 0x01; // enable spi

        // =======================================================================================================

        return; // spi_slave_command_mode -> spi slave task

dummysend:
        srcs[0] = (void *)spi_slave_dummy_bytes;
        dests[0] = (void *)&spi_handle.dr[0];

        prepare_send(userdata);

        dma_loop_async(driver.slave_instance_.dma, srcs, 1, dests, 1, false, false, 4, data_length, 1,
                NULL, userdata, driver.slave_instance_.dma_event, &stop_signal);
        set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

        _wait_slave_transfer_finish(userdata, spi_handle.txflr);

        switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
        goto exit;

dummyreceive:
        dests[0] = (void *)spi_slave_dummy_bytes;
        srcs[0] = (void *)&spi_handle.dr[0];

        dma_loop_async(driver.slave_instance_.dma, srcs, 1, dests, 1, false, false, 4, data_length, 1,
                NULL, userdata, driver.slave_instance_.dma_event, &stop_signal);
        set_handshake(driver.slave_instance_.handshake, GPIO_PV_LOW);

        _wait_slave_transfer_finish(userdata, spi_handle.rxflr);
        goto exit;
    }

    // ==== End of SPI Slave functions =====================================================

    //---------------------------------------------
    void setup_device(k_spi_device_driver &device);

    //-------------------
    volatile spi_t &spi()
    {
        return spi_;
    }

    //-------------------------------------
    volatile spi_slave_command_t &command()
    {
        return slave_instance_.command;
    }

    //--------------------------------------------------------------------------------------
    static void write_inst_addr(volatile uint32_t *dr, const uint8_t **buffer, size_t width)
    {
        configASSERT(width <= 4);
        if (width)
        {
            uint32_t cmd = 0;
            uint8_t *pcmd = (uint8_t *)&cmd;
            size_t i;
            for (i = 0; i < width; i++)
            {
                pcmd[i] = **buffer;
                ++(*buffer);
            }

            *dr = cmd;
        }
    }

private:
    volatile spi_t &spi_;
    sysctl_clock_t clock_;
    sysctl_dma_select_t dma_req_;
    uint8_t mod_off_;
    uint8_t dfs_off_;
    uint8_t tmod_off_;
    uint8_t frf_off_;

    SemaphoreHandle_t free_mutex_;
    spi_slave_instance_t slave_instance_;
};

/* SPI Device */

class k_spi_device_driver : public spi_device_driver, public heap_object, public exclusive_object_access
{
public:
    k_spi_device_driver(object_accessor<k_spi_driver> spi, spi_mode_t mode, spi_frame_format_t frame_format, uint32_t chip_select_mask, uint32_t data_bit_length)
        : spi_(std::move(spi)), mode_(mode), frame_format_(frame_format), chip_select_mask_(chip_select_mask), data_bit_length_(data_bit_length)
    {
        configASSERT(data_bit_length >= 4 && data_bit_length <= 32);
        configASSERT(chip_select_mask);

        switch (frame_format)
        {
        case SPI_FF_DUAL:
            configASSERT(data_bit_length % 2 == 0);
            break;
        case SPI_FF_QUAD:
            configASSERT(data_bit_length % 4 == 0);
            break;
        case SPI_FF_OCTAL:
            configASSERT(data_bit_length % 8 == 0);
            break;
        default:
            break;
        }

        buffer_width_ = get_buffer_width(data_bit_length);
    }

    virtual void install() override
    {
    }

    virtual void config_non_standard(uint32_t instruction_length, uint32_t address_length, uint32_t wait_cycles, spi_inst_addr_trans_mode_t trans_mode) override
    {
        instruction_length_ = instruction_length;
        address_length_ = address_length;
        inst_width_ = get_inst_addr_width(instruction_length);
        addr_width_ = get_inst_addr_width(address_length);
        wait_cycles_ = wait_cycles;
        trans_mode_ = trans_mode;
    }

    virtual bool set_xip_mode(bool enable) override // LoBo
    {
        return spi_->set_xip_mode(*this, enable);
    }

    virtual void master_config_half_duplex(int8_t mosi, int8_t miso) override // LoBo
    {
        spi_->master_config_half_duplex(*this, mosi, miso);
    }

    virtual double set_clock_rate(double clock_rate) override
    {
        return spi_->set_clock_rate(*this, clock_rate);
    }

    virtual void set_endian(uint32_t endian) override
    {
        spi_->set_endian(*this, endian);
    }
	
    virtual int read(gsl::span<uint8_t> buffer) override
    {
        return spi_->read(*this, buffer);
    }

    virtual int write(gsl::span<const uint8_t> buffer) override
    {
        return spi_->write(*this, buffer);
    }

    virtual int transfer_full_duplex(gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer) override
    {
        return spi_->transfer_full_duplex(*this, write_buffer, read_buffer);
    }

    virtual int transfer_sequential(gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer) override
    {
        return spi_->transfer_sequential(*this, write_buffer, read_buffer);
    }

    virtual int transfer_sequential_with_delay(gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer, uint16_t delay) override
    {
        return spi_->transfer_sequential_with_delay(*this, write_buffer, read_buffer, delay);
    }

    virtual void fill(uint32_t instruction, uint32_t address, uint32_t value, size_t count) override
    {
        spi_->fill(*this, instruction, address, value, count);
    }

private:
    static int get_buffer_width(size_t data_bit_length)
    {
        if (data_bit_length <= 8)
            return 1;
        else if (data_bit_length <= 16)
            return 2;
        return 4;
    }

    static int get_inst_addr_width(size_t length)
    {
        if (length == 0)
            return 0;
        else if (length <= 8)
            return 1;
        else if (length <= 16)
            return 2;
        else if (length <= 24)
            return 3;
        return 4;
    }


private:
    friend class k_spi_driver;

    object_accessor<k_spi_driver> spi_;
    spi_mode_t mode_;
    spi_frame_format_t frame_format_;
    uint32_t chip_select_mask_;
    uint32_t data_bit_length_;
    uint32_t instruction_length_ = 0;
    uint32_t address_length_ = 0;
    uint32_t inst_width_ = 0;
    uint32_t addr_width_ = 0;
    uint32_t wait_cycles_ = 0;
    spi_inst_addr_trans_mode_t trans_mode_;
    uint32_t baud_rate_ = 0x2;
    uint32_t buffer_width_ = 0;
    uint32_t endian_ = 0;
    // LoBo
    int8_t mosi_ = -1;
    int8_t miso_ = -1;
    uint16_t spi_mosi_func_ = FUNC_MAX;
};

object_ptr<spi_device_driver> k_spi_driver::get_device(spi_mode_t mode, spi_frame_format_t frame_format, uint32_t chip_select_mask, uint32_t data_bit_length)
{
    auto driver = make_object<k_spi_device_driver>(make_accessor<k_spi_driver>(this), mode, frame_format, chip_select_mask, data_bit_length);
    driver->install();
    return driver;
}

// LoBo: added function
bool k_spi_driver::set_xip_mode(k_spi_device_driver &device, bool enable)
{
    // XiP mode can only be enabled on SPI3 in QUAD mode
    if (&spi_ != (volatile spi_t *)SPI3_BASE_ADDR) {
        LOGE(TAG, "XiP enable: not an SPI3 device");
        return false;
    }
    if (device.frame_format_ != SPI_FF_QUAD) {
        LOGE(TAG, "XiP enable: not in QUAD mode");
        return false;
    }

    if (enable) {
        spi_.xip_ctrl = (0x01 << 29) | (0x02 << 26) | (0x01 << 23) | (0x01 << 22) | (0x04 << 13) |
                   (0x01 << 12) | (0x02 << 9) | (0x06 << 4) | (0x01 << 2) | 0x02;
        spi_.xip_incr_inst = 0xEB;
        spi_.xip_mode_bits = 0x00;
        spi_.xip_ser = 0x01;
        spi_.ssienr = 0x01;
        sysctl->peri.spi3_xip_en = 1;
    }
    else {
        sysctl->peri.spi3_xip_en = 0;
    }
    return true;
}

// LoBo: added function
void k_spi_driver::master_config_half_duplex(k_spi_device_driver &device, int8_t mosi, int8_t miso)
{
    device.spi_mosi_func_ = FUNC_MAX; // indicates device is not in half-duplex mode
    device.mosi_ = mosi;
    device.miso_ = miso;
    if ((mosi >= 0) && (miso == -1)) {
        // half-duplex mode (device.spi_mosi_func_ < FUNC_MAX)
        if (((uintptr_t)&spi_ == SPI0_BASE_ADDR)) device.spi_mosi_func_ = FUNC_SPI0_D0;
        else if (((uintptr_t)&spi_ == SPI1_BASE_ADDR)) device.spi_mosi_func_ = FUNC_SPI1_D0;
    }
}

// LoBo:
double k_spi_driver::set_clock_rate(k_spi_device_driver &device, double clock_rate)
{
    if (clock_ == SYSCTL_CLOCK_SPI0) sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI0, 0);
    else if (clock_ == SYSCTL_CLOCK_SPI1) sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI1, 0);

    double clk = (double)sysctl_clock_get_freq(clock_);
    uint32_t div = std::min(65534U, std::max((uint32_t)ceil(clk / clock_rate), 2U));
    if (div & 1) div++; // only even divisors allowed
    if (device.baud_rate_ != div)
        LOGV(TAG, "SPI_%d Bdr: clk=%u, div=%u, bdr=%u", clock_ - SYSCTL_CLOCK_SPI0, (uint32_t)clk, div, (uint32_t)(clk / div));
    device.baud_rate_ = div;
    return clk / div;
}

void k_spi_driver::set_endian(k_spi_device_driver &device, uint32_t endian)
{
    device.endian_ = endian;
}

// Wait for DMA transfer to finish
//------------------------------------------------------------------------------------------------------------------------------------------------
static int _wait_DMA_transfer(SemaphoreHandle_t dma_event1, SemaphoreHandle_t dma_event2, uintptr_t dma_trans1, uintptr_t dma_trans2, int timeout)
{
    // timeout in ms
    uint64_t tstart = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
    uint64_t tend = tstart;
    uint64_t tcurr;
    int ret = 1;
    bool check1 = true;
    bool check2 = (dma_event2 != NULL);
    uint64_t tmo = tstart + (timeout * 1000);
    while (1) {
        if (check1) {
            if (xSemaphoreTake(dma_event1, 2 / portTICK_PERIOD_MS) == pdTRUE) {
                check1 = false;
                if (!check2) break;
            }
        }
        if (check2) {
            check2 = false;
            if (xSemaphoreTake(dma_event2, 2 / portTICK_PERIOD_MS) == pdTRUE) {
                if (!check1) break;
            }
        }
        if ((!check1) && (!check2)) break;

        tcurr = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
        if (tcurr > tmo) {
            // timeout
            if (check1) {
                if (xSemaphoreTake(dma_event1, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                    ret = 0;
                    LOGW(TAG, "DMA transfer not finished, stopped");
                }
                else {
                    ret = -1;
                    LOGE(TAG, "DMA transfer not finished, NOT stopped");
                }
            }
            if (check2) {
                if (xSemaphoreTake(dma_event2, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                    ret = 0;
                    LOGW(TAG, "DMA transfer not finished, stopped");
                }
                else {
                    ret = -1;
                    LOGE(TAG, "DMA transfer not finished, NOT stopped");
                }
            }
            break;
        }
    }
    tend = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
    LOGV(TAG, "Transfer time: %lu us)", tend - tstart);
    return ret;
}

int k_spi_driver::read(k_spi_device_driver &device, gsl::span<uint8_t> buffer)
{
    COMMON_ENTRY;

    setup_device(device);

    int ret = buffer.size();
    uint32_t i = 0;
    size_t rx_buffer_len = buffer.size();
    size_t rx_frames = rx_buffer_len / device.buffer_width_;
    auto buffer_read = buffer.data();
    set_bit_mask(&spi_.ctrlr0, TMOD_MASK, TMOD_VALUE(2));
    spi_.ctrlr1 = rx_frames - 1;
    spi_.ssienr = 0x01;
    if (device.frame_format_ == SPI_FF_STANDARD)
    {
        spi_.dr[0] = 0xFFFFFFFF;
    }

    if (device.spi_mosi_func_ < FUNC_MAX) {
        // In half-duplex mode use MOSI pin as input (MISO)
        fpioa_set_function(device.mosi_, (fpioa_function_t)(device.spi_mosi_func_+1));
    }
    if (rx_frames < SPI_TRANSMISSION_THRESHOLD)
    {
        vTaskEnterCritical();
        size_t index, fifo_len;
        while (rx_frames)
        {
            const uint8_t *buffer_it = buffer.data();
            write_inst_addr(spi_.dr, &buffer_it, device.inst_width_);
            write_inst_addr(spi_.dr, &buffer_it, device.addr_width_);
            spi_.ser = device.chip_select_mask_;

            fifo_len = spi_.rxflr;
            fifo_len = fifo_len < rx_frames ? fifo_len : rx_frames;
            switch (device.buffer_width_)
            {
            case 4:
                for (index = 0; index < fifo_len; index++)
                    ((uint32_t *)buffer_read)[i++] = spi_.dr[0];
                break;
            case 2:
                for (index = 0; index < fifo_len; index++)
                    ((uint16_t *)buffer_read)[i++] = (uint16_t)spi_.dr[0];
                break;
            default:
                for (index = 0; index < fifo_len; index++)
                    buffer_read[i++] = (uint8_t)spi_.dr[0];
                break;
            }
            rx_frames -= fifo_len;
        }
        vTaskExitCritical();
    }
    else
    {
        uintptr_t dma_read = dma_open_free();
        dma_set_request_source(dma_read, dma_req_);
        spi_.dmacr = 0x1;
        SemaphoreHandle_t event_read = xSemaphoreCreateBinary();

        dma_transmit_async(dma_read, &spi_.dr[0], buffer_read, 0, 1, device.buffer_width_, rx_frames, 1, event_read);
        const uint8_t *buffer_it = buffer.data();
        write_inst_addr(spi_.dr, &buffer_it, device.inst_width_);
        write_inst_addr(spi_.dr, &buffer_it, device.addr_width_);
        spi_.ser = device.chip_select_mask_;

        int dma_ret = _wait_DMA_transfer(event_read, NULL, dma_read, 0, SPI_DMA_BLOCK_TIME);
        configASSERT(dma_ret >= 0);
        if (dma_ret < 1) ret = -1;

        dma_close(dma_read);
        vSemaphoreDelete(event_read);
    }
    if (device.spi_mosi_func_ < FUNC_MAX) {
        // In half-duplex mode use MOSI again as output
        fpioa_set_function(device.mosi_, (fpioa_function_t)device.spi_mosi_func_);
    }

    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    return ret;
}

int k_spi_driver::write(k_spi_device_driver &device, gsl::span<const uint8_t> buffer)
{
    COMMON_ENTRY;

    setup_device(device);

    int ret = buffer.size();
    uint32_t i = 0;
    size_t tx_buffer_len = buffer.size() - (device.inst_width_ + device.addr_width_);
    size_t tx_frames = tx_buffer_len / device.buffer_width_;
    auto buffer_write = buffer.data();
    set_bit_mask(&spi_.ctrlr0, TMOD_MASK, TMOD_VALUE(1));

    if (tx_frames < SPI_TRANSMISSION_THRESHOLD)
    {
        vTaskEnterCritical();
        size_t index, fifo_len;
        spi_.ssienr = 0x01;
        write_inst_addr(spi_.dr, &buffer_write, device.inst_width_);
        write_inst_addr(spi_.dr, &buffer_write, device.addr_width_);
        spi_.ser = device.chip_select_mask_;
        while (tx_buffer_len)
        {
            fifo_len = 32 - spi_.txflr;
            fifo_len = fifo_len < tx_buffer_len ? fifo_len : tx_buffer_len;
            switch (device.buffer_width_)
            {
            case 4:
                fifo_len = fifo_len / 4 * 4;
                for (index = 0; index < fifo_len / 4; index++)
                    spi_.dr[0] = ((uint32_t *)buffer_write)[i++];
                break;
            case 2:
                fifo_len = fifo_len / 2 * 2;
                for (index = 0; index < fifo_len / 2; index++)
                    spi_.dr[0] = ((uint16_t *)buffer_write)[i++];
                break;
            default:
                for (index = 0; index < fifo_len; index++)
                    spi_.dr[0] = buffer_write[i++];
                break;
            }
            tx_buffer_len -= fifo_len;
        }
        vTaskExitCritical();
    }
    else
    {
        uintptr_t dma_write = dma_open_free();
        dma_set_request_source(dma_write, dma_req_ + 1);
        spi_.dmacr = 0x2;
        spi_.ssienr = 0x01;
        write_inst_addr(spi_.dr, &buffer_write, device.inst_width_);
        write_inst_addr(spi_.dr, &buffer_write, device.addr_width_);
        SemaphoreHandle_t event_write = xSemaphoreCreateBinary();

        dma_transmit_async(dma_write, buffer_write, &spi_.dr[0], 1, 0, device.buffer_width_, tx_frames, 4, event_write);
        spi_.ser = device.chip_select_mask_;
        int dma_ret = _wait_DMA_transfer(event_write, NULL, dma_write, 0, SPI_DMA_BLOCK_TIME);
        configASSERT(dma_ret >= 0);
        if (dma_ret < 1) ret = -1;

        dma_close(dma_write);
        vSemaphoreDelete(event_write);
    }
    while ((spi_.sr & 0x05) != 0x04)
        ;
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    return ret;
}

int k_spi_driver::transfer_full_duplex(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer)
{
    COMMON_ENTRY;
    setup_device(device);
    set_bit_mask(&spi_.ctrlr0, TMOD_MASK, TMOD_VALUE(0));
    return read_write(device, write_buffer, read_buffer);
}

int k_spi_driver::transfer_sequential(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer)
{
    // LoBo: handle half-duplex
    if (device.spi_mosi_func_ < FUNC_MAX) {
        // In half-duplex mode MOSI pin is used also as input (MISO)
        // so, execute write and read operation separately
        configASSERT(device.frame_format_ == SPI_FF_STANDARD);
        write(device, write_buffer);
        return read(device, read_buffer);
    }
    COMMON_ENTRY;
    setup_device(device);
    set_bit_mask(&spi_.ctrlr0, TMOD_MASK, TMOD_VALUE(3));
    return read_write(device, write_buffer, read_buffer);
}

// LoBo: added function
int k_spi_driver::transfer_sequential_with_delay(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer, uint16_t delay)
{
    configASSERT(device.frame_format_ == SPI_FF_STANDARD);
    write(device, write_buffer);
    if (delay > 0) {
        // Delay before read (in us)
        uint64_t start_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
        uint64_t end_us = start_us + delay;
        while ((read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000)) < end_us) {
            ;
        }
    }
    return read(device, read_buffer);
}

int k_spi_driver::read_write(k_spi_device_driver &device, gsl::span<const uint8_t> write_buffer, gsl::span<uint8_t> read_buffer)
{
    configASSERT(device.frame_format_ == SPI_FF_STANDARD);
    int ret = read_buffer.size();
    size_t tx_buffer_len = write_buffer.size();
    size_t rx_buffer_len = read_buffer.size();
    size_t tx_frames = tx_buffer_len / device.buffer_width_;
    size_t rx_frames = rx_buffer_len / device.buffer_width_;
    auto buffer_read = read_buffer.data();
    auto buffer_write = write_buffer.data();
    uint32_t i = 0;

    if ((rx_frames < SPI_TRANSMISSION_THRESHOLD) || (device.spi_mosi_func_ < FUNC_MAX))
    {
        vTaskEnterCritical();
        size_t index, fifo_len;
        spi_.ctrlr1 = rx_frames - 1;
        spi_.ssienr = 0x01;
        while (tx_buffer_len)
        {
            fifo_len = 32 - spi_.txflr;
            fifo_len = fifo_len < tx_buffer_len ? fifo_len : tx_buffer_len;
            switch (device.buffer_width_)
            {
            case 4:
                fifo_len = fifo_len / 4 * 4;
                for (index = 0; index < fifo_len / 4; index++)
                    spi_.dr[0] = ((uint32_t *)buffer_write)[i++];
                break;
            case 2:
                fifo_len = fifo_len / 2 * 2;
                for (index = 0; index < fifo_len / 2; index++)
                    spi_.dr[0] = ((uint16_t *)buffer_write)[i++];
                break;
            default:
                for (index = 0; index < fifo_len; index++)
                    spi_.dr[0] = buffer_write[i++];
                break;
            }
            spi_.ser = device.chip_select_mask_;
            tx_buffer_len -= fifo_len;
        }

        if (device.spi_mosi_func_ < FUNC_MAX) {
            // In half-duplex mode use MOSI pin as input (MISO)
            fpioa_set_function(device.mosi_, (fpioa_function_t)(device.spi_mosi_func_+1));
        }
        i = 0;
        while (rx_buffer_len)
        {
            fifo_len = spi_.rxflr;
            fifo_len = fifo_len < rx_buffer_len ? fifo_len : rx_buffer_len;
            switch (device.buffer_width_)
            {
            case 4:
                fifo_len = fifo_len / 4 * 4;
                for (index = 0; index < fifo_len / 4; index++)
                    ((uint32_t *)buffer_read)[i++] = spi_.dr[0];
                break;
            case 2:
                fifo_len = fifo_len / 2 * 2;
                for (index = 0; index < fifo_len / 2; index++)
                    ((uint16_t *)buffer_read)[i++] = (uint16_t)spi_.dr[0];
                break;
            default:
                for (index = 0; index < fifo_len; index++)
                    buffer_read[i++] = (uint8_t)spi_.dr[0];
                break;
            }
            spi_.ser = device.chip_select_mask_;
            rx_buffer_len -= fifo_len;
        }
        if (device.spi_mosi_func_ < FUNC_MAX) {
            // In half-duplex mode use MOSI again as output
            fpioa_set_function(device.mosi_, (fpioa_function_t)device.spi_mosi_func_);
        }
        vTaskExitCritical();
    }
    else
    {
        uintptr_t dma_write = dma_open_free();
        uintptr_t dma_read = dma_open_free();

        dma_set_request_source(dma_write, dma_req_ + 1);
        dma_set_request_source(dma_read, dma_req_);
        spi_.ctrlr1 = rx_frames - 1;
        spi_.dmacr = 0x3;
        spi_.ssienr = 0x01;
        spi_.ser = device.chip_select_mask_;
        SemaphoreHandle_t event_read = xSemaphoreCreateBinary(), event_write = xSemaphoreCreateBinary();
        dma_transmit_async(dma_read, &spi_.dr[0], buffer_read, 0, 1, device.buffer_width_, rx_frames, 1, event_read);
        dma_transmit_async(dma_write, buffer_write, &spi_.dr[0], 1, 0, device.buffer_width_, tx_frames, 4, event_write);

        int dma_ret = _wait_DMA_transfer(event_read, event_write, dma_read, dma_write, SPI_DMA_BLOCK_TIME);
        configASSERT(dma_ret >= 0);
        if (dma_ret < 1) ret = -1;

        dma_close(dma_write);
        dma_close(dma_read);
        vSemaphoreDelete(event_read);
        vSemaphoreDelete(event_write);
    }
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    return ret;
}

void k_spi_driver::fill(k_spi_device_driver &device, uint32_t instruction, uint32_t address, uint32_t value, size_t count)
{
    COMMON_ENTRY;
    setup_device(device);

    uintptr_t dma_write = dma_open_free();
    dma_set_request_source(dma_write, dma_req_ + 1);

    set_bit_mask(&spi_.ctrlr0, TMOD_MASK, TMOD_VALUE(1));
    spi_.dmacr = 0x2;
    spi_.ssienr = 0x01;

    const uint8_t *buffer = (const uint8_t *)&instruction;
    write_inst_addr(spi_.dr, &buffer, device.inst_width_);
    buffer = (const uint8_t *)&address;
    write_inst_addr(spi_.dr, &buffer, device.addr_width_);

    SemaphoreHandle_t event_write = xSemaphoreCreateBinary();
    dma_transmit_async(dma_write, &value, &spi_.dr[0], 0, 0, sizeof(uint32_t), count, 4, event_write);

    spi_.ser = device.chip_select_mask_;
    int dma_ret = _wait_DMA_transfer(event_write, NULL, dma_write, 0, SPI_DMA_BLOCK_TIME);
    configASSERT(dma_ret >= 0);

    dma_close(dma_write);
    vSemaphoreDelete(event_write);

    while ((spi_.sr & 0x05) != 0x04)
        ;
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;
}

void k_spi_driver::setup_device(k_spi_device_driver &device)
{
    spi_.baudr = device.baud_rate_;
    spi_.imr = 0x00;
    spi_.dmacr = 0x00;
    spi_.dmatdlr = 0x10;
    spi_.dmardlr = 0x0;
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.ctrlr0 = (device.mode_ << mod_off_) | (device.frame_format_ << frf_off_) | ((device.data_bit_length_ - 1) << dfs_off_);
    spi_.spi_ctrlr0 = 0;

    if (device.frame_format_ != SPI_FF_STANDARD)
    {
        configASSERT(device.wait_cycles_ < (1 << 5));

        uint32_t inst_l = 0;
        switch (device.instruction_length_)
        {
        case 0:
            inst_l = 0;
            break;
        case 4:
            inst_l = 1;
            break;
        case 8:
            inst_l = 2;
            break;
        case 16:
            inst_l = 3;
            break;
        default:
            configASSERT("Invalid instruction length");
            break;
        }

        uint32_t trans = 0;
        switch (device.trans_mode_)
        {
        case SPI_AITM_STANDARD:
            trans = 0;
            break;
        case SPI_AITM_ADDR_STANDARD:
            trans = 1;
            break;
        case SPI_AITM_AS_FRAME_FORMAT:
            trans = 2;
            break;
        default:
            configASSERT("Invalid trans mode");
            break;
        }

        configASSERT(device.address_length_ % 4 == 0 && device.address_length_ <= 60);
        uint32_t addr_l = device.address_length_ / 4;

        spi_.spi_ctrlr0 = (device.wait_cycles_ << 11) | (inst_l << 8) | (addr_l << 2) | trans;
        spi_.endian = device.endian_;
    }
}

static k_spi_driver dev0_driver(SPI0_BASE_ADDR, SYSCTL_CLOCK_SPI0, SYSCTL_DMA_SELECT_SSI0_RX_REQ, 6, 16, 8, 21);
static k_spi_driver dev1_driver(SPI1_BASE_ADDR, SYSCTL_CLOCK_SPI1, SYSCTL_DMA_SELECT_SSI1_RX_REQ, 6, 16, 8, 21);
static k_spi_driver dev_slave_driver(SPI_SLAVE_BASE_ADDR, SYSCTL_CLOCK_SPI2, SYSCTL_DMA_SELECT_SSI2_RX_REQ, 6, 16, 8, 21);
static k_spi_driver dev3_driver(SPI3_BASE_ADDR, SYSCTL_CLOCK_SPI3, SYSCTL_DMA_SELECT_SSI3_RX_REQ, 8, 0, 10, 22);

driver &g_spi_driver_spi0 = dev0_driver;
driver &g_spi_driver_spi1 = dev1_driver;
driver &g_spi_driver_spi_slave = dev_slave_driver;
driver &g_spi_driver_spi3 = dev3_driver;
