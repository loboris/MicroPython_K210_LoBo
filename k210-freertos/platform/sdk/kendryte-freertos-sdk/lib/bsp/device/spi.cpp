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

extern uint64_t dmac_intstatus;

using namespace sys;

// transmission length (frames) bellow which DMA transfer is not used
// used only by SPI master
// during the non-DMA transfer interrupts are disabled !
#define SPI_TRANSMISSION_THRESHOLD  0x800UL
#define SPI_SLAVE_INFO              "K210 v1.2" // !must be exactly 9 bytes!

/* SPI Controller */

#define TMOD_MASK (3 << tmod_off_)
#define TMOD_VALUE(value) (value << tmod_off_)
#define COMMON_ENTRY \
    semaphore_lock locker(free_mutex_);

typedef enum
{
    IDLE,
    COMMAND,
    TRANSFER,
    DEINIT,
} spi_slave_status_e;

typedef struct _spi_slave_instance
{
    size_t data_bit_length;
    volatile spi_slave_status_e status;
    volatile spi_slave_command_t command;
    volatile spi_slave_command_t last_command;
    volatile uint8_t *databuff_ptr;
    uint32_t databuff_size;
    uint32_t databuff_ro_size;
    int32_t  mosi;
    int32_t  miso;

    uintptr_t dma;
    SemaphoreHandle_t dma_event;
    SemaphoreHandle_t slave_event;
    TaskHandle_t slave_task_handle;
    spi_slave_receive_callback_t callback;
    spi_slave_csum_callback_t csum_callback;
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
        pic_set_irq_enable(IRQN_SPI_SLAVE_INTERRUPT, 0);
        atomic_set(&slave_instance_.status, DEINIT);
        int tmo = 5000;
        while (slave_instance_.slave_task_handle != NULL) {
            xSemaphoreGive(slave_instance_.slave_event);
            vTaskDelay(5);
            tmo -= 5;
            if (tmo == 0) {
                LOGE(TAG, "Slave task not stopped");
                break;
            }
        }
        spi_.dmacr = 0x00;
        spi_.ssienr = 0x00;
        sysctl_clock_disable(SYSCTL_CLOCK_SPI2);
        vSemaphoreDelete(slave_instance_.dma_event);
        vSemaphoreDelete(slave_instance_.slave_event);
        dma_close(slave_instance_.dma);
    }

    //---------------
    void slave_init()
    {

    }

    //-----------------------------------------------------------------------------------------------
    void slave_config(size_t data_bit_length, uint8_t *data, uint32_t len, uint32_t ro_len,
                      spi_slave_receive_callback_t callback, spi_slave_csum_callback_t csum_callback,
                      int priority, int mosi, int miso)
    {
        slave_instance_.status = IDLE;
        slave_instance_.databuff_ptr = data;
        slave_instance_.databuff_size = len;
        slave_instance_.databuff_ro_size = ro_len;
        slave_instance_.data_bit_length = data_bit_length;
        slave_instance_.dma = dma_open_free();
        slave_instance_.dma_event = xSemaphoreCreateBinary();
        slave_instance_.slave_event = xSemaphoreCreateBinary();
        slave_instance_.csum_callback = csum_callback;
        slave_instance_.callback = callback;
        slave_instance_.mosi = mosi;
        slave_instance_.miso = miso;
        uint8_t slv_oe = 10;
        sysctl_reset(SYSCTL_RESET_SPI2);
        sysctl_clock_enable(SYSCTL_CLOCK_SPI2);
        sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI2, 4);
        LOGD(SLAVE_TAG, "SPI Clk: %u", sysctl_clock_get_freq(SYSCTL_CLOCK_SPI2));

        uint32_t data_width = data_bit_length / 8;

        spi_.ssienr = 0x00;
        spi_.ctrlr0 = (0x0 << mod_off_) | (0x1 << slv_oe) | ((data_bit_length - 1) << dfs_off_);
        spi_.dmatdlr = 0x04;
        spi_.dmardlr = 0x03;
        spi_.dmacr = 0x00;
        spi_.txftlr = 0x00;
        spi_.rxftlr = 0x08 / data_width - 1;

        pic_set_irq_priority(IRQN_SPI_SLAVE_INTERRUPT, 4);
        pic_set_irq_enable(IRQN_SPI_SLAVE_INTERRUPT, 1);
        pic_set_irq_handler(IRQN_SPI_SLAVE_INTERRUPT, spi_slave_irq, this);

        slave_instance_.slave_task_handle = NULL;
        auto ret = xTaskCreate(spi_slave_irq_thread, "spi_slave_irq", configMINIMAL_STACK_SIZE, this, priority+2, &slave_instance_.slave_task_handle);
        configASSERT(ret == pdTRUE);
        spi_.imr = 0x10;
        spi_.ssienr = 0x01;
    }

private:
    //---------------------------------------------
    void setup_device(k_spi_device_driver &device);

    // SPI Slave command processing task
    // Runs with the priority higher than the main task
    //----------------------------------------------
    static void spi_slave_irq_thread(void *userdata)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        while (1)
        {
            if (xSemaphoreTake(driver.slave_instance_.slave_event, portMAX_DELAY) == pdTRUE)
            {
                // slave interrupt occurred
                spi_slave_status_e status = atomic_read(&driver.slave_instance_.status);
                if (status == IDLE) {
                    atomic_cas((volatile int *)&driver.slave_instance_.status, IDLE, COMMAND);
                    spi_slave_command_mode(userdata);
                }
                else if (status == DEINIT) {
                    break;
                }
            }
        }
        driver.slave_instance_.slave_task_handle = NULL;
        vTaskDelete(NULL);
    }

    // SPI Slave ISR
    //----------------------------------
    static void spi_slave_irq(void *ctx)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        auto &driver = *reinterpret_cast<k_spi_driver *>(ctx);
        auto &spi_handle = driver.spi();
        spi_handle.imr = 0x00;
        *reinterpret_cast<volatile uint32_t *>(spi_handle.icr);
        // inform the slave task about the event
        xSemaphoreGiveFromISR(driver.slave_instance_.slave_event, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }

    // Setup the slave idle mode, wait for 8-byte command
    //----------------------------------------------------------
    static void spi_slave_idle_mode(void *userdata, bool irq_en)
    {
        uint8_t slv_oe = 10;
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();
        uint32_t data_width = driver.slave_instance_.data_bit_length / 8;
        driver.slave_instance_.status = IDLE;
        spi_handle.ssienr = 0x00;
        spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x1 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
        spi_handle.rxftlr = 0x08 / data_width - 1; // expecting 8-byte command

        spi_handle.dmacr = 0x00;
        spi_handle.imr = 0x10;
        if (irq_en) spi_handle.ssienr = 0x01;

        LOGD(SLAVE_TAG, "In IDLE mode");
    }

    // Wait for DMA transfer iniated from command handler to finish
    //-----------------------------------------------
    static bool _wait_transfer_finish(void *userdata)
    {
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        //auto &spi_handle = driver.spi();

        // timeout in ms
        uint64_t tstart = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
        uint64_t tend = tstart;
        bool f = true;
        int tmo = 200;
        while (1) {
            if (xSemaphoreTake(driver.slave_instance_.dma_event, 2 / portTICK_PERIOD_MS) == pdTRUE) break;
            tmo -= 2;
            if (tmo < 0) {
                f = false;
                break;
            }
        }
        tend = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
        if ((dmac_intstatus & 0x02) == 0) f = false;
        if (!f) driver.slave_instance_.command.err = SPI_CMD_ERR_FATAL;
        LOGD(SLAVE_TAG, "Transfer time: %lu us)", tend - tstart);
        return f;
    }

    // SPI SLAVE uses only one pin (mosi) for both receive and transper (FUNC_SPI_SLAVE_D0)
    // If 'miso' pin is defined, we are switching SPI slave pin to it during sending data
    //--------------------------------------------
    static void switch_to_miso(int mosi, int miso)
    {
        if (miso >= 0) {
            // Set 'miso' for FUNC_SPI_SLAVE_D0
            fpioa_set_function(mosi, FUNC_RESV0);
            fpioa_set_io(miso, &FUNC_SPI_SLAVE_MISO);
        }
    }

    //--------------------------------------------
    static void switch_to_mosi(int mosi, int miso)
    {
        if (miso >= 0) {
            // Set 'mosi' for FUNC_SPI_SLAVE_D0
            // wait 10 us, allow all data to be transfered
            uint64_t time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            uint64_t time_end = time_us + 8;
            while (time_us < time_end) {
                time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            }
            fpioa_set_function(miso, FUNC_RESV0);
            fpioa_set_function(mosi, FUNC_SPI_SLAVE_D0);
        }
    }

    // 8-byte command from master is received
    //------------------------------------------------
    static void spi_slave_command_mode(void *userdata)
    {
        // Start of master transaction
        auto &driver = *reinterpret_cast<k_spi_driver *>(userdata);
        auto &spi_handle = driver.spi();
        uint64_t time_us, end_time_us;
        uint8_t cmd_data[8], sum = 0;
        uint8_t slv_oe = 10;
        memset((void *)&driver.slave_instance_.command, 0, sizeof(spi_slave_command_t));
        driver.slave_instance_.command.time = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);

        uint32_t data_width = (driver.slave_instance_.data_bit_length + 7) / 8;

        // === Read the 8-byte command structure ===
        vTaskEnterCritical();
        switch (data_width)
        {
        case 4:
            for (uint32_t i = 0; i < 8 / 4; i++)
                ((uint32_t *)cmd_data)[i] = spi_handle.dr[0];
            break;
        case 2:
            for (uint32_t i = 0; i < 8 / 2; i++)
                ((uint16_t *)cmd_data)[i] = spi_handle.dr[0];
            break;
        default:
            for (uint32_t i = 0; i < 8; i++)
                cmd_data[i] = spi_handle.dr[0];
            break;
        }
        vTaskExitCritical();
        time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
        end_time_us = time_us;

        // calculate csum
        for (uint32_t i = 0; i < 7; i++) {
            sum ^= cmd_data[i];
        }

        driver.slave_instance_.command.cmd = cmd_data[0];
        driver.slave_instance_.command.addr = cmd_data[1] | (cmd_data[2] << 8) | (cmd_data[3] << 16);
        driver.slave_instance_.command.len = cmd_data[4] | (cmd_data[5] << 8) | (cmd_data[6] << 16);

        if (cmd_data[7] != sum) {
            driver.slave_instance_.command.err =  SPI_CMD_ERR_CSUM;
            goto exit;
        }
        if ((driver.slave_instance_.command.len > driver.slave_instance_.databuff_size) || (driver.slave_instance_.command.len == 0)) {
            driver.slave_instance_.command.err =  SPI_CMD_ERR_LENGTH;
            goto exit;
        }

        // === Command ok, process it ===
        spi_handle.ssienr = 0x00;
        //------------------------------------------------------------------------------------------------------------------------------------------
        if ((driver.slave_instance_.command.cmd == SPI_CMD_READ_DATA_BLOCK) || (driver.slave_instance_.command.cmd == SPI_CMD_READ_DATA_BLOCK_CSUM))
        {
            // ---------------------------------------------------------------------------------
            // Send requested number of bytes from requested slave buffer address
            // In case of READ_DATA_BLOCK_CSUM command, append 2-bytes checksum to the sent data
            // ---------------------------------------------------------------------------------

            uint8_t csum_bkp[2] = {0};
            uint32_t data_len = driver.slave_instance_.command.len;
            // check if the requested data fits into slave buffer
            if (driver.slave_instance_.command.addr >= driver.slave_instance_.databuff_size) {
                driver.slave_instance_.command.err =  SPI_CMD_ERR_ADDRESS;
                goto exit;
            }
            if ((driver.slave_instance_.command.addr + data_len) > driver.slave_instance_.databuff_size) {
                data_len = driver.slave_instance_.databuff_size - driver.slave_instance_.command.addr;
            }

            if (driver.slave_instance_.command.cmd == SPI_CMD_READ_DATA_BLOCK_CSUM) {
                // 2-byte checksum will be appended to the data
                uint16_t csum = 0;
                if (driver.slave_instance_.csum_callback) {
                    csum = driver.slave_instance_.csum_callback((const uint8_t *)(driver.slave_instance_.databuff_ptr+driver.slave_instance_.command.addr), data_len);
                }
                csum_bkp[0] = *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len);
                csum_bkp[1] = *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len + 1);
                *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len) = csum & 0xff;
                *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len + 1) = (csum >> 8) & 0xff;
                data_len += 2;
            }
            driver.slave_instance_.command.err = SPI_CMD_ERR_OK;
            size_t tx_frames = (data_len / data_width); // + 8;
            spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x0 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
            set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);
            spi_handle.dmacr = 0x02;
            spi_handle.imr = 0x00;
            spi_handle.ssienr = 0x01;

            switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

            end_time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_ + 1);
            dma_transmit_async(driver.slave_instance_.dma, (void *)(driver.slave_instance_.databuff_ptr+driver.slave_instance_.command.addr), &spi_handle.dr[0], 1, 0, data_width, tx_frames, 1, driver.slave_instance_.dma_event);

            _wait_transfer_finish(userdata);

            switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);

            if (driver.slave_instance_.command.cmd == SPI_CMD_READ_DATA_BLOCK_CSUM) {
                // restore data buffer
                data_len -= 2;
                *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len) = csum_bkp[0];
                *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len + 1) = csum_bkp[1];
            }
        }
        //-------------------------------------------------------------------------------------------------------------------------------------------------
        else if ((driver.slave_instance_.command.cmd == SPI_CMD_WRITE_DATA_BLOCK) || (driver.slave_instance_.command.cmd == SPI_CMD_WRITE_DATA_BLOCK_CSUM))
        {
            // ---------------------------------------------------------------------
            // Receive requested number of bytes into requested slave buffer address
            // ---------------------------------------------------------------------

            uint8_t csum_bkp[2] = {0};
            int data_len = driver.slave_instance_.command.len;

            // check if the requested data address is inside slave buffer
            if (driver.slave_instance_.command.addr >= (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) {
                driver.slave_instance_.command.err = SPI_CMD_ERR_ADDRESS;
                goto exit;
            }
            // check data length
            if ( (data_len > (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) ||
                 (data_len < ((driver.slave_instance_.command.cmd == SPI_CMD_WRITE_DATA_BLOCK_CSUM) ? 3 : 1)) ||
                 ((driver.slave_instance_.command.addr + data_len) > (driver.slave_instance_.databuff_size - driver.slave_instance_.databuff_ro_size)) ) {
                driver.slave_instance_.command.err = SPI_CMD_ERR_LENGTH;
                goto exit;
            }
            if (driver.slave_instance_.command.cmd == SPI_CMD_WRITE_DATA_BLOCK_CSUM) {
                // backup two bytes which will be overwritten with checksum
                csum_bkp[0] = *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len - 2);
                csum_bkp[1] = *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len - 1);
            }
            // slave buffer position
            uint8_t *new_data = (uint8_t *)(driver.slave_instance_.databuff_ptr+driver.slave_instance_.command.addr);

            driver.slave_instance_.command.err = SPI_CMD_ERR_OK;
            size_t tx_frames = data_len / data_width;
            spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x1 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
            spi_handle.dmacr = 0x01;
            spi_handle.imr = 0x00;
            spi_handle.ssienr = 0x01;

            end_time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_);
            spi_handle.dmacr = 0x01;
            dma_transmit_async(driver.slave_instance_.dma, &spi_handle.dr[0], new_data, 0, 1, data_width, tx_frames, 4, driver.slave_instance_.dma_event);

            if (_wait_transfer_finish(userdata)) {
                // === data received ===
                if (driver.slave_instance_.command.cmd == SPI_CMD_WRITE_DATA_BLOCK_CSUM) {
                    // received data with checksum, test checksum
                    if (driver.slave_instance_.csum_callback) {
                        uint16_t *rcsum = (uint16_t *)(new_data + data_len-2); // received checksum
                        uint16_t csum = driver.slave_instance_.csum_callback((const uint8_t *)new_data, data_len-2); // calculated checksum
                        if (csum != *rcsum) driver.slave_instance_.command.err = SPI_CMD_ERR_DATA_CSUM;
                    }
                    // restore two bytes overwritten with checksum
                    *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len - 2) = csum_bkp[0];
                    *(driver.slave_instance_.databuff_ptr + driver.slave_instance_.command.addr + data_len - 1) = csum_bkp[1];
                }
            }
        }
        //------------------------------------------------------------------
        else if (driver.slave_instance_.command.cmd == SPI_CMD_TEST_COMMAND)
        {
            // ---------------------------------------------------------------------------------
            // Send the requested number of byte values specified in the command's address field
            // ---------------------------------------------------------------------------------

            driver.slave_instance_.command.err = SPI_CMD_ERR_OK;

            size_t tx_frames = (driver.slave_instance_.command.len / data_width); // + 8;
            spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x0 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
            set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);
            spi_handle.dmacr = 0x02;
            spi_handle.imr = 0x00;
            spi_handle.ssienr = 0x01;

            switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

            end_time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_ + 1);
            dma_transmit_async(driver.slave_instance_.dma, &driver.slave_instance_.command.addr, &spi_handle.dr[0], 0, 0, data_width, tx_frames, 1, driver.slave_instance_.dma_event);

            _wait_transfer_finish(userdata);

            switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
        }
        //---------------------------------------------------------------
        else if (driver.slave_instance_.command.cmd == SPI_CMD_READ_INFO)
        {
            // ------------------------------------------------------------------
            // Send the SPI slave info to the master (16 bytes + 2-byte checksum)
            // ------------------------------------------------------------------

            uint16_t csum = 0;
            volatile uint8_t slave_info[18] = { 0 };
            memcpy((uint8_t *)slave_info, SPI_SLAVE_INFO, 9);
            slave_info[10] = driver.slave_instance_.databuff_size & 0xff;
            slave_info[11] = (driver.slave_instance_.databuff_size >> 8) & 0xff;
            slave_info[12] = (driver.slave_instance_.databuff_size >> 16) & 0xff;
            slave_info[13] = driver.slave_instance_.databuff_ro_size & 0xff;
            slave_info[14] = (driver.slave_instance_.databuff_ro_size >> 8) & 0xff;
            slave_info[15] = (driver.slave_instance_.databuff_ro_size >> 16) & 0xff;
            if (driver.slave_instance_.csum_callback) {
                csum = driver.slave_instance_.csum_callback((const uint8_t *)(slave_info), 16);
            }
            slave_info[16] = csum & 0xff;
            slave_info[17] = (csum >> 8) & 0xff;

            driver.slave_instance_.command.err = SPI_CMD_ERR_OK;

            size_t tx_frames = (18 / data_width); // + 8;
            spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x0 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
            set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);
            spi_handle.dmacr = 0x02;
            spi_handle.imr = 0x00;
            spi_handle.ssienr = 0x01;

            switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

            end_time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_ + 1);
            dma_transmit_async(driver.slave_instance_.dma, slave_info, &spi_handle.dr[0], 1, 0, data_width, tx_frames, 1, driver.slave_instance_.dma_event);

            _wait_transfer_finish(userdata);

            switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
        }
        //-----------------------------------------------------------------
        else if (driver.slave_instance_.command.cmd == SPI_CMD_LAST_STATUS)
        {
            // ----------------------------------------------------
            // Send the SPI slave last command status to the master
            // ----------------------------------------------------

            uint16_t csum = 0;
            int data_len = sizeof(spi_slave_command_t);
            volatile uint8_t slave_status[data_len+2] = { 0 }; // 26 bytes
            memcpy((uint8_t *)slave_status, (void *)&driver.slave_instance_.last_command, data_len);
            if (driver.slave_instance_.csum_callback) {
                csum = driver.slave_instance_.csum_callback((const uint8_t *)(slave_status), sizeof(spi_slave_command_t));
            }
            slave_status[sizeof(spi_slave_command_t)] = csum & 0xff;
            slave_status[sizeof(spi_slave_command_t)+1] = (csum >> 8) & 0xff;

            driver.slave_instance_.command.err = SPI_CMD_ERR_OK;

            size_t tx_frames = (data_len+2) / data_width; // + 8;
            spi_handle.ctrlr0 = (0x0 << driver.mod_off_) | (0x0 << slv_oe) | ((driver.slave_instance_.data_bit_length - 1) << driver.dfs_off_);
            set_bit_mask(&spi_handle.ctrlr0, 3 << driver.tmod_off_, 1 << driver.tmod_off_);
            spi_handle.dmacr = 0x02;
            spi_handle.imr = 0x00;
            spi_handle.ssienr = 0x01;

            switch_to_miso(driver.slave_instance_.mosi, driver.slave_instance_.miso);

            end_time_us = read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
            dma_set_request_source(driver.slave_instance_.dma, driver.dma_req_ + 1);
            dma_transmit_async(driver.slave_instance_.dma, slave_status, &spi_handle.dr[0], 1, 0, data_width, tx_frames, 1, driver.slave_instance_.dma_event);

            _wait_transfer_finish(userdata);

            switch_to_mosi(driver.slave_instance_.mosi, driver.slave_instance_.miso);
        }
        //---- unhandled command, return to idle mode ----
        else
        {
            driver.slave_instance_.command.err = SPI_CMD_ERR_COMMAND;
        }

exit:
        // set command execution time
        driver.slave_instance_.command.time = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000)) - driver.slave_instance_.command.time;
        // backup the command
        if (driver.slave_instance_.command.cmd != SPI_CMD_LAST_STATUS)
            memcpy((void *)&driver.slave_instance_.last_command, (void *)&driver.slave_instance_.command, sizeof(spi_slave_command_t));
        // execute the callback function if set
        if (driver.slave_instance_.callback != NULL) driver.slave_instance_.callback((void *)&driver.slave_instance_.command);

        LOGD(SLAVE_TAG, "Prepare time: %lu us", end_time_us - time_us);
        spi_slave_idle_mode(userdata, (driver.slave_instance_.command.err != SPI_CMD_ERR_FATAL));
        return;
    }

    //-------------------
    volatile spi_t &spi()
    {
        return spi_;
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
    // ToDo: check if device is spi3
    if (device.frame_format_ != SPI_FF_QUAD) {
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
    device.spi_mosi_func_ = FUNC_MAX;
    device.mosi_ = mosi;
    device.miso_ = miso;
    if ((mosi >= 0) && (miso == -1)) {
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
    device.baud_rate_ = div;
    LOGD(TAG, "SPI_%d Bdr: clk=%u, div=%u, bdr=%u", clock_ - SYSCTL_CLOCK_SPI0, (uint32_t)clk, div, (uint32_t)(clk / div));
    return clk / div;
}

int k_spi_driver::read(k_spi_device_driver &device, gsl::span<uint8_t> buffer)
{
    COMMON_ENTRY;

    setup_device(device);

    if (device.spi_mosi_func_ < FUNC_MAX) {
        // In half-duplex mode use MOSI as input (MISO)
        fpioa_set_function(device.mosi_, (fpioa_function_t)(device.spi_mosi_func_+1));
    }

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
        configASSERT(xSemaphoreTake(event_read, portMAX_DELAY) == pdTRUE);
        dma_close(dma_read);
        vSemaphoreDelete(event_read);
    }

    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    if (device.spi_mosi_func_ < FUNC_MAX) {
        // In half-duplex mode use MOSI again as output
        fpioa_set_function(device.mosi_, (fpioa_function_t)device.spi_mosi_func_);
    }

    return buffer.size();
}

int k_spi_driver::write(k_spi_device_driver &device, gsl::span<const uint8_t> buffer)
{
    COMMON_ENTRY;

    setup_device(device);

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
        configASSERT(xSemaphoreTake(event_write, portMAX_DELAY) == pdTRUE);
        dma_close(dma_write);
        vSemaphoreDelete(event_write);
    }
    while ((spi_.sr & 0x05) != 0x04)
        ;
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    return buffer.size();
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
        // Delay before read
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
            // In half-duplex mode use MOSI as input (MISO)
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

        configASSERT(xSemaphoreTake(event_read, portMAX_DELAY) == pdTRUE && xSemaphoreTake(event_write, portMAX_DELAY) == pdTRUE);

        dma_close(dma_write);
        dma_close(dma_read);
        vSemaphoreDelete(event_read);
        vSemaphoreDelete(event_write);
    }
    spi_.ser = 0x00;
    spi_.ssienr = 0x00;
    spi_.dmacr = 0x00;

    return read_buffer.size();
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
    configASSERT(xSemaphoreTake(event_write, portMAX_DELAY) == pdTRUE);
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
