/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list
 *      of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form, except as embedded into a Nordic Semiconductor ASA
 *      integrated circuit in a product or a software update for such product, must reproduce
 *      the above copyright notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without specific prior
 *      written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Nordic Semiconductor ASA integrated circuit.
 *
 *   5. Any software provided in binary or object form under this license must not be reverse
 *      engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if DEVICE_SPI

#include "hal/spi_api.h"

#include "object_owners.h"
#include "pinmap_ex.h"
#include "PeripheralPins.h"

#if NRFX_CHECK(NRFX_SPI_ENABLED)
#include "nrfx_spi.h"
#endif
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
#include "nrfx_spim.h"
#endif
#if NRFX_CHECK(NRFX_SPIS_ENABLED)
#include "nrfx_spis.h"
#endif

#if 1           // TODO, return this to 0 once debug complete
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

#define NORDIC_NRF5_MAX_SPI_INST    4   /**< Maximum number of SPI instances (3 on NRF52832, 4 on NRF52840) */

/**< An enum of SPI instance types (e.g. SPI/SPIM/SPIS) */
enum spi_spim_spis_t
{
    NRFX_TYPE_NONE,                     /**< None/not configured SPI instance */
    NRFX_TYPE_SPI,                      /**< Nordic legacy SPI master */
    NRFX_TYPE_SPIM,                     /**< Nordic SPI master */
    NRFX_TYPE_SPIS                      /**< Nordic SPI slave */
};

/**< Struct to store a pointer to the SPI instance, and to store it's instance type (e.g. SPI/SPIM/SPIS) */
struct nordic_nrf5_spi_instance_t
{
    const void* ptr;                    /**< Pointer to the instance object */
    const enum spi_spim_spis_t type;    /**< Type of the SPI instance (e.g. SPI/SPIM/SPIS) */
    bool initialised;                   /**< Flag to keep track of which instances have been initialised */
};


// TODO, the following 'nrfx_spi_t' declarations may need to be 'static const'
// TODO, add #if error checking if multiple instances of same number are enabled in sdk_config.h

/* Pre-allocate either SPI, SPIM or SPIS instances, based on sdk_config.h. */
// SPI/SPIM/SPIS Instance 0
#if   NRFX_CHECK(NRFX_SPI0_ENABLED)
    static const nrfx_spi_t spi_inst0 = NRFX_SPI_INSTANCE(0);
    static const enum spi_spim_spis_t spi_inst0_type = NRFX_TYPE_SPI;
    static nrfx_spi_config_t spi_inst0_config;
#elif NRFX_CHECK(NRFX_SPIM0_ENABLED)
    static const nrfx_spim_t spi_inst0 = NRFX_SPIM_INSTANCE(0);
    static const enum spi_spim_spis_t spi_inst0_type = NRFX_TYPE_SPIM;
    static nrfx_spim_config_t spi_inst0_config;
#elif NRFX_CHECK(NRFX_SPIS0_ENABLED)
    static const nrfx_spis_t spi_inst0 = NRFX_SPIS_INSTANCE(0);
    static const enum spi_spim_spis_t spi_inst0_type = NRFX_TYPE_SPIS;
    static nrfx_spis_config_t spi_inst0_config;
#endif
// SPI/SPIM/SPIS Instance 1
#if   NRFX_CHECK(NRFX_SPI1_ENABLED)
    static const nrfx_spi_t spi_inst1 = NRFX_SPI_INSTANCE(1);
    static const enum spi_spim_spis_t spi_inst1_type = NRFX_TYPE_SPI;
    static nrfx_spi_config_t spi_inst1_config;
#elif NRFX_CHECK(NRFX_SPIM1_ENABLED)
    static const nrfx_spim_t spi_inst1 = NRFX_SPIM_INSTANCE(1);
    static const enum spi_spim_spis_t spi_inst1_type = NRFX_TYPE_SPIM;
    static nrfx_spim_config_t spi_inst1_config;
#elif NRFX_CHECK(NRFX_SPIS1_ENABLED)
    static const nrfx_spis_t spi_inst1 = NRFX_SPIS_INSTANCE(1);
    static const enum spi_spim_spis_t spi_inst1_type = NRFX_TYPE_SPIS;
    static nrfx_spis_config_t spi_inst1_config;
#endif
// SPI/SPIM/SPIS Instance 2
#if   NRFX_CHECK(NRFX_SPI2_ENABLED)
    static const nrfx_spi_t spi_inst2 = NRFX_SPI_INSTANCE(2);
    static const enum spi_spim_spis_t spi_inst2_type = NRFX_TYPE_SPI;
    static nrfx_spi_config_t spi_inst2_config;
#elif NRFX_CHECK(NRFX_SPIM2_ENABLED)
    static const nrfx_spim_t spi_inst2 = NRFX_SPIM_INSTANCE(2);
    static const enum spi_spim_spis_t spi_inst2_type = NRFX_TYPE_SPIM;
    static nrfx_spim_config_t spi_inst2_config;
#elif NRFX_CHECK(NRFX_SPIS2_ENABLED)
    static const nrfx_spis_t spi_inst2 = NRFX_SPIS_INSTANCE(2);
    static const enum spi_spim_spis_t spi_inst2_type = NRFX_TYPE_SPIS;
    static nrfx_spis_config_t spi_inst2_config;
#endif
// SPIM Instance 3 (NRF52840 only)
#if NRFX_CHECK(NRFX_SPIM3_ENABLED)
    static const nrfx_spim_t spi_inst3 = NRFX_SPIM_INSTANCE(3);
    static const enum spi_spim_spis_t spi_inst3_type = NRFX_TYPE_SPIM;
    static nrfx_spim_config_t spi_inst3_config;
#endif

/* Array of pointers of pre-allocated instances and their SPI types (e.g. SPI/SPIM/SPIS). */
static struct nordic_nrf5_spi_instance_t nordic_nrf5_spi_instance[NORDIC_NRF5_MAX_SPI_INST] = 
{
    { .ptr = &spi_inst0, .type = spi_inst0_type, .initialised = false },
    { .ptr = &spi_inst1, .type = spi_inst1_type, .initialised = false },
    { .ptr = &spi_inst2, .type = spi_inst2_type, .initialised = false },
#if NRFX_CHECK(NRFX_SPIM3_ENABLED)   
    { .ptr = &spi_inst3, .type = spi_inst3_type, .initialised = false }
#else
    { .ptr = NULL,       .type = NRFX_TYPE_NONE, .initialised = false }
#endif
};

// TODO, these may be able to be merged into a common event handler
/* Forware declare interrupt handlers. */
#if DEVICE_SPI_ASYNCH
#if NRFX_CHECK(NRFX_SPI_ENABLED)
static void nordic_nrf5_spi_event_handler(nrfx_spi_evt_t const *p_event, void *p_context);
#endif
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
static void nordic_nrf5_spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context);
#endif
#if NRFX_CHECK(NRFX_SPIS_ENABLED)
static void nordic_nrf5_spis_event_handler(nrfx_spis_evt_t const *p_event, void *p_context);
#endif
#endif  // DEVICE_SPI_ASYNCH

/* Forward declaration. These functions are implemented in the driver but not
 * set up in the NVIC due to it being relocated.
 */
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
void SPIM2_SPIS2_SPI2_IRQHandler(void);

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
/* Forward declaration. These functions are implemented in the driver but not
 * set up in the NVIC due to it being relocated.
 */
void SPIM3_IRQHandler(void);
#endif // NRFX_SPIM_ENABLED

/**
 * Brief       Reconfigure peripheral.
 *
 *             If the peripheral has changed ownership clear old configuration and
 *             re-initialize the peripheral with the new settings.
 *
 * Parameter   obj           The object
 * Parameter   handler       Optional callback handler.
 * Parameter   force_change  Force change regardless of ownership.
 */
static void spi_configure_driver_instance(spi_t *obj)
{
#if DEVICE_SPI_ASYNCH
    struct spi_s *spi_inst = &obj->spi;
#else
    struct spi_s *spi_inst = obj;
#endif

    int instance = spi_inst->instance;
    // TODO thow error if instance >= NORDIC_NRF5_MAX_SPI_INST

    /* Get pointer to object of the current owner of the peripheral. */
    void *current_owner = object_owner_spi2c_get(instance);

    /* Check if reconfiguration is actually necessary. */
    if ((obj != current_owner) || spi_inst->update) {

        /* Update applied, reset flag. */
        spi_inst->update = false;

        /* Clean up and uninitialize peripheral if already initialized. */
        if (nordic_nrf5_spi_instance[instance].initialised) {
            switch (nordic_nrf5_spi_instance[instance].type) {
#if NRFX_CHECK(NRFX_SPI_ENABLED)
                case NRFX_TYPE_SPI:
                    nrfx_spi_uninit(nordic_nrf5_spi_instance[instance].ptr);
                    break;
#endif
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
                case NRFX_TYPE_SPIM:
                    nrfx_spim_uninit(nordic_nrf5_spi_instance[instance].ptr);
                    break;
#endif
#if NRFX_CHECK(NRFX_SPI_ENABLED)
                case NRFX_TYPE_SPIS:
                    nrfx_spis_uninit(nordic_nrf5_spi_instance[instance].ptr);
                    break;
#endif
                default:
                break;
            }
        }

#if DEVICE_SPI_ASYNCH
            /* Set callback handler in asynchronous mode. */
            if (spi_inst->handler) {
                switch (nordic_nrf5_spi_instance[instance].type) {
#if NRFX_CHECK(NRFX_SPI_ENABLED)
                    case NRFX_TYPE_SPI:
                        nrfx_spi_init(nordic_nrf5_spi_instance[instance].ptr, &(spi_inst->master_config), nordic_nrf5_spi_event_handler, obj);
                        break;
#endif  // NRFX_CHECK(NRFX_SPI_ENABLED)
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
                    case NRFX_TYPE_SPIM:
                        nrfx_spim_init(nordic_nrf5_spi_instance[instance].ptr, &(spi_inst->master_config), nordic_nrf5_spim_event_handler, obj);
                        break;
#endif  // NRFX_CHECK(NRFX_SPIM_ENABLED)
#if NRFX_CHECK(NRFX_SPIS_ENABLED)
                    case NRFX_TYPE_SPIS:
                        nrfx_spis_init(nordic_nrf5_spi_instance[instance].ptr, &(spi_inst->slave_config), nordic_nrf5_spis_event_handler, obj);
                        break;
#endif  // NRFX_CHECK(NRFX_SPIS_ENABLED)
                    default:
                    break;
            }
        }
#else   // DEVICE_SPI_ASYNCH
        /* Set callback handler to NULL in synchronous mode. */
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
        nrfx_spim_init(&nordic_nrf5_spim_instance[instance], &(spi_inst->config), NULL, NULL);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
        nrfx_spi_init(&nordic_nrf5_spim_instance[instance], &(spi_inst->config), NULL, NULL);
#endif  // NRFX_CHECK(NRFX_SPIM_ENABLED) elif NRFX_CHECK(NRFX_SPI_ENABLED)

#endif  // DEVICE_SPI_ASYNCH

        /* Mark instance as initialized. */
        nordic_nrf5_spi_instance[instance].initialised = true;

        /* Claim ownership of peripheral. */
        object_owner_spi2c_set(instance, obj);
    }
}

void spi_get_capabilities(PinName ssel, bool slave, spi_capabilities_t *cap)
{
    if (slave) {
        cap->minimum_frequency = 200000;            // 200 kHz
        cap->maximum_frequency = 2000000;           // 2 MHz
        cap->word_length = 0x00000080;              // 8 bit symbols
        cap->support_slave_mode = false;            // to be determined later based on ssel
        cap->hw_cs_handle = false;                  // irrelevant in slave mode
        cap->slave_delay_between_symbols_ns = 2500; // 2.5 us
        cap->clk_modes = 0x0f;                      // all clock modes
        cap->tx_rx_buffers_equal_length = false;    // rx/tx buffers can have different sizes
#if DEVICE_SPI_ASYNCH
        cap->async_mode = true;
#else
        cap->async_mode = false;
#endif
    } else {
        cap->minimum_frequency = 200000;          // 200 kHz
        cap->maximum_frequency = 2000000;         // 2 MHz
        cap->word_length = 0x00000080;            // 8 bit symbols
        cap->support_slave_mode = false;          // to be determined later based on ssel
        cap->hw_cs_handle = false;                // to be determined later based on ssel
        cap->slave_delay_between_symbols_ns = 0;  // irrelevant in master mode
        cap->clk_modes = 0x0f;                    // all clock modes
        cap->tx_rx_buffers_equal_length = false;  // rx/tx buffers can have different sizes
#if DEVICE_SPI_ASYNCH
        cap->async_mode = true;
#else
        cap->async_mode = false;
#endif
    }

    // check if given ssel pin is in the cs pinmap
    const PinMap *cs_pins = spi_master_cs_pinmap();
    PinName pin = NC;
    while (cs_pins->pin != NC) {
        if (cs_pins->pin == ssel) {
#if DEVICE_SPISLAVE
            cap->support_slave_mode = true;
#endif
            cap->hw_cs_handle = true;
            break;
        }
        cs_pins++;
    }
}

/** Initialize the SPI peripheral
 *
 * Configures the pins used by SPI, sets a default format and frequency, and enables the peripheral
 * Parameter   obj  The SPI object to initialize
 * Parameter   mosi The pin to use for MOSI
 * Parameter   miso The pin to use for MISO
 * Parameter   sclk The pin to use for SCLK
 * Parameter   ssel The pin to use for SSEL
 */
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    struct spi_s *spi_inst = &obj->spi;     // point to the spi_s object owned by the SPI/SPISlave class

    /* Get instance based on requested pins. */
    spi_inst->instance = pin_instance_spi(mosi, miso, sclk);
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    MBED_ASSERT(spi_inst->instance < NRFX_SPIM_ENABLED_COUNT);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    MBED_ASSERT(spi_inst->instance < NRFX_SPI_ENABLED_COUNT);
#endif

    /* Store chip select separately for manual enabling. */
    spi_inst->cs = ssel;

    /* At this point we don't know whether we are initialising as master or slave, 
     so we prepare the config structs for both (only if both are enabled within sdk_config.h) */
#if NRFX_CHECK(NRFX_SPI_ENABLED) || NRFX_CHECK(NRFX_SPIM_CHECK)
    /* Store pins except chip select. */
    spi_inst->master_config.sck_pin        = sclk;
    spi_inst->master_config.mosi_pin       = mosi;
    spi_inst->master_config.miso_pin       = miso;
    spi_inst->master_config.irq_priority   = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_inst->master_config.orc            = SPI_FILL_CHAR;

#if NRFX_CHECK(NRFX_SPI_ENABLED)
    spi_inst->master_config.ss_pin         = NRFX_SPI_PIN_NOT_USED;
    spi_inst->master_config.frequency      = NRF_SPI_FREQ_4M;
    spi_inst->master_config.mode           = NRF_SPI_MODE_0;
    spi_inst->master_config.bit_order      = NRF_SPI_BIT_ORDER_MSB_FIRST;

#elif NRFX_CHECK(NRFX_SPIM_ENABLED)
    spi_inst->master_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_inst->master_config.frequency      = NRF_SPIM_FREQ_4M;
    spi_inst->master_config.mode           = NRF_SPIM_MODE_0;
    spi_inst->master_config.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;

#endif  // NRFX_CHECK(NRFX_SPI_ENABLED) elif NRFX_CHECK(NRFX_SPIM_ENABLED)
#endif  // NRFX_CHECK(NRFX_SPI_ENABLED) || NRFX_CHECK(NRFX_SPIM_CHECK)

#if NRFX_CHECK(NRFX_SPIS_ENABLED)
    /* Store pins except chip select. */
    spi_inst->slave_config.sck_pin        = sclk;
    spi_inst->slave_config.mosi_pin       = mosi;
    spi_inst->slave_config.miso_pin       = miso;
    spi_inst->slave_config.csn_pin        = ssel;       // TODO, this may need to be PIN_NOT_USED

    /* Use the default config. */
    spi_inst->slave_config.irq_priority   = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_inst->slave_config.orc            = SPI_FILL_CHAR;
    spi_inst->slave_config.def            = SPI_FILL_CHAR;
    spi_inst->slave_config.miso_drive     = NRFX_SPIS_DEFAULT_MISO_DRIVE;
    spi_inst->slave_config.csn_pullup     = NRFX_SPIS_DEFAULT_CSN_PULLUP;
    spi_inst->slave_config.bit_order      = NRF_SPIS_BIT_ORDER_MSB_FIRST;
    spi_inst->slave_config.mode           = NRF_SPIS_MODE_0;
#endif  // NRFX_CHECK(NRFX_SPIS_ENABLED)


#if DEVICE_SPI_ASYNCH
    /* Set default values for asynchronous variables. */
    spi_inst->handler = 0;
    spi_inst->mask = 0;
    spi_inst->event = 0;
#endif

    /* Configure peripheral. This is called on each init to ensure all pins are set correctly
     * according to the SPI mode before calling CS for the first time.
     */
    spi_configure_driver_instance(obj);

    /* Configure GPIO pin if chip select has been set. */
    if (ssel != NC) {
        nrf_gpio_pin_set(ssel);
        nrf_gpio_cfg_output(ssel);
    }

    static bool first_init = true;

    if (first_init) {
        first_init = false;

        /* Register interrupt handlers in driver with the NVIC. */
        NVIC_SetVector(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, (uint32_t) SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler);
        NVIC_SetVector(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, (uint32_t) SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler);
        NVIC_SetVector(SPIM2_SPIS2_SPI2_IRQn, (uint32_t) SPIM2_SPIS2_SPI2_IRQHandler);
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
        NVIC_SetVector(SPIM3_IRQn, (uint32_t) SPIM3_IRQHandler);
#endif
    }
}

/** Release a SPI object
 *
 * TODO: spi_free is currently unimplemented
 * This will require reference counting at the C++ level to be safe
 *
 * Return the pins owned by the SPI object to their reset state
 * Disable the SPI peripheral
 * Disable the SPI clock
 * Parameter  obj The SPI object to deinitialize
 */
void spi_free(spi_t *obj)
{
    struct spi_s *spi_inst = &obj->spi;

    int instance = spi_inst->instance;
    // TODO throw error if 'instance >= NORDIC_NRF5_MAX_SPI_INST'

    /* Use driver uninit to free instance. */
    switch (nordic_nrf5_spi_instance[instance].type) {
#if NRFX_CHECK(NRFX_SPI_ENABLED)
        case NRFX_TYPE_SPI:
            nrfx_spi_uninit(nordic_nrf5_spi_instance[instance].ptr);
            break;
#endif  // NRFX_CHECK(NRFX_SPI_ENABLED)
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
        case NRFX_TYPE_SPIM:
            nrfx_spim_uninit(nordic_nrf5_spi_instance[instance].ptr);
            break;
#endif  // NRFX_CHECK(NRFX_SPIM_ENABLED)
#if NRFX_CHECK(NRFX_SPIS_ENABLED)
        case NRFX_TYPE_SPIS:
            nrfx_spis_uninit(nordic_nrf5_spi_instance[instance].ptr);
            break;
#endif  // NRFX_CHECK(NRFX_SPIS_ENABLED)
        default:
        break;
    }

    /* Mark instance as uninitialized. */
    nordic_nrf5_spi_instance[instance].initialised = false;
}

/** Configure the SPI format
 *
 * Set the number of bits per frame, configure clock polarity and phase, shift order and master/slave mode.
 * The default bit order is MSB.
 * Parameter      obj   The SPI object to configure
 * Parameter      bits  The number of bits per frame
 * Parameter      mode  The SPI mode (clock polarity, phase, and shift direction)
 * Parameter      slave Zero for master mode or non-zero for slave mode
 */
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    /* SPI module only supports 8 bit transfers. */
    MBED_ASSERT(bits == 8);

    // if 'slave = 0', ensure that the spi instance being used is SPI/SPIM
    if (slave == 0)
    {
        /* If this assert fails, the obj->spi.instance number is not correctly enabled
        within sdk_config.h (e.g. spi.instance = 2, so NRFX_SPIM2_ENABLED 1 should be defined) */
        MBED_ASSERT((nordic_nrf5_spi_instance[obj->spi.instance].type == NRFX_TYPE_SPI) ||
                    (nordic_nrf5_spi_instance[obj->spi.instance].type == NRFX_TYPE_SPIM));
    }
    // else if 'slave = 1', ensure that the spi instance being used is SPIS
    else    // slave == 1
    {
        /* If this assert fails, the obj->spi.instance number is not correctly enabled
        within sdk_config.h (e.g. spi.instance = 2, so NRFX_SPIS2_ENABLED 1 should be defined) */
        MBED_ASSERT(nordic_nrf5_spi_instance[obj->spi.instance].type == NRFX_TYPE_SPIS);
    }

    // TODO only force update if config/mode/slave has changed
    /* Set flag to force update. */
    obj->spi.update = true;
    
    /* Configure peripheral if necessary. Must be called on each format to ensure the pins are set
     * correctly according to the SPI mode.
     */
    spi_configure_driver_instance(obj);
}

/** Set the SPI baud rate
 *
 * Actual frequency may differ from the desired frequency due to available dividers and bus clock
 * Configures the SPI peripheral's baud rate
 * Parameter      obj The SPI object to configure
 * Parameter      hz  The baud rate in Hz
 */
void spi_frequency(spi_t *obj, int hz)
{
#if DEVICE_SPI_ASYNCH
    struct spi_s *spi_inst = &obj->spi;
#else
    struct spi_s *spi_inst = obj;
#endif

#if NRFX_CHECK(NRFX_SPI_ENABLED)
    nrf_spi_frequency_t new_frequency = NRF_SPI_FREQ_1M;

    /* Convert frequency to Nordic enum type. */
    if (hz < 250000) {
        new_frequency = NRF_SPI_FREQ_125K;
    } else if (hz < 500000) {
        new_frequency = NRF_SPI_FREQ_250K;
    } else if (hz < 1000000) {
        new_frequency = NRF_SPI_FREQ_500K;
    } else if (hz < 2000000) {
        new_frequency = NRF_SPI_FREQ_1M;
    } else if (hz < 4000000) {
        new_frequency = NRF_SPI_FREQ_2M;
    } else if (hz < 8000000) {
        new_frequency = NRF_SPI_FREQ_4M;
    } else {
        new_frequency = NRF_SPI_FREQ_8M;
    }
#elif NRFX_CHECK(NRFX_SPIM_ENABLED)
    nrf_spim_frequency_t new_frequency = NRF_SPIM_FREQ_1M;

    /* Convert frequency to Nordic enum type. */
    if (hz < 250000) {
        new_frequency = NRF_SPIM_FREQ_125K;
    } else if (hz < 500000) {
        new_frequency = NRF_SPIM_FREQ_250K;
    } else if (hz < 1000000) {
        new_frequency = NRF_SPIM_FREQ_500K;
    } else if (hz < 2000000) {
        new_frequency = NRF_SPIM_FREQ_1M;
    } else if (hz < 4000000) {
        new_frequency = NRF_SPIM_FREQ_2M;
    } else if (hz < 8000000) {
        new_frequency = NRF_SPIM_FREQ_4M;
    } else if (hz < 16000000) {
        new_frequency = NRF_SPIM_FREQ_8M;
    } else if (hz < 32000000) {
        new_frequency = NRF_SPIM_FREQ_16M;
    } else {
        new_frequency = NRF_SPIM_FREQ_32M;
    }

#endif
    /* Check if configuration has changed. */
    if (spi_inst->master_config.frequency != new_frequency) {
        spi_inst->master_config.frequency = new_frequency;

        /* Set flag to force update. */
        spi_inst->update = true;
    }
}

/** Write a byte out in master mode and receive a value
 *
 * Parameter  obj   The SPI peripheral to use for sending
 * Parameter  value The value to send
 * Return     Returns the value received during send
 */
int spi_master_write(spi_t *obj, int value)
{
    nrfx_err_t ret;

#if DEVICE_SPI_ASYNCH
    struct spi_s *spi_inst = &obj->spi;
#else
    struct spi_s *spi_inst = obj;
#endif

    int instance = spi_inst->instance;

    /* Local variables used in transfer. */
    const uint8_t tx_buff = (uint8_t) value;
    uint8_t rx_buff;

    /* Configure peripheral if necessary. */
    spi_configure_driver_instance(obj);

    /* Manually clear chip select pin if defined. */
    if (spi_inst->cs != NC) {
        nrf_gpio_pin_clear(spi_inst->cs);
    }

    /* Transfer 1 byte. */
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
   nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TRX(&tx_buff, 1, &rx_buff, 1);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
   nrfx_spi_xfer_desc_t desc = NRFX_SPI_XFER_TRX(&tx_buff, 1, &rx_buff, 1);
#endif

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    ret = nrfx_spim_xfer(nordic_nrf5_spim_instance[instance].ptr, &desc, 0);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    ret = nrfx_spi_xfer(nordic_nrf5_spi_instance[instance].ptr, &desc, 0);
#endif

    if (ret != NRFX_SUCCESS) {
        DEBUG_PRINTF("%d error returned from nrf_spi_xfer\n\r", ret);
    }

    /* Manually set chip select pin if defined. */
    if (spi_inst->cs != NC) {
        nrf_gpio_pin_set(spi_inst->cs);
    }

    return rx_buff;
}

/** Write a block out in master mode and receive a value
 *
 *  The total number of bytes sent and recieved will be the maximum of
 *  tx_length and rx_length. The bytes written will be padded with the
 *  value 0xff.
 *
 * Parameter  obj        The SPI peripheral to use for sending
 * Parameter  tx_buffer  Pointer to the byte-array of data to write to the device
 * Parameter  tx_length  Number of bytes to write, may be zero
 * Parameter  rx_buffer  Pointer to the byte-array of data to read from the device
 * Parameter  rx_length  Number of bytes to read, may be zero
 * Parameter  write_fill Default data transmitted while performing a read
 * @returns
 *      The number of bytes written and read from the device. This is
 *      maximum of tx_length and rx_length.
 */
int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
#if DEVICE_SPI_ASYNCH
    struct spi_s *spi_inst = &obj->spi;
#else
    struct spi_s *spi_inst = obj;
#endif

    int instance = spi_inst->instance;

    /* Check if overflow character has changed. */
    if (spi_inst->master_config.orc != write_fill) {

        /* Store new overflow character and force reconfiguration. */
        spi_inst->update = true;
        spi_inst->master_config.orc = write_fill;
    }

    /* Configure peripheral if necessary. */
    spi_configure_driver_instance(obj);

    /* Manually clear chip select pin if defined. */
    if (spi_inst->cs != NC) {
        nrf_gpio_pin_clear(spi_inst->cs);
    }

    /* The Nordic SPI driver is only able to transfer 255 bytes at a time.
     * The following code will write/read the data 255 bytes at a time and
     * ensure that asymmetrical transfers are handled properly.
     */
    int tx_offset = 0;
    int rx_offset = 0;

    ret_code_t result = NRFX_SUCCESS;

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TRX(tx_buffer, tx_length, rx_buffer, rx_length);
    result = nrfx_spim_xfer(nordic_nrf5_spim_instance[instance].ptr, &desc, 0);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    /* Loop until all data is sent and received. */
    while (((tx_length > 0) || (rx_length > 0)) && (result == NRFX_SUCCESS)) {

        /* Check if tx_length is larger than 255 and if so, limit to 255. */
        int tx_actual_length = (tx_length > 255) ? 255 : tx_length;

        /* Set tx buffer pointer. Set to NULL if no data is going to be transmitted. */
        const uint8_t *tx_actual_buffer = (tx_actual_length > 0) ?
                                          (const uint8_t *)(tx_buffer + tx_offset) :
                                          NULL;

        /* Check if rx_length is larger than 255 and if so, limit to 255. */
        int rx_actual_length = (rx_length > 255) ? 255 : rx_length;

        /* Set rx buffer pointer. Set to NULL if no data is going to be received. */
        uint8_t *rx_actual_buffer = (rx_actual_length > 0) ?
                                    (uint8_t *)(rx_buffer + rx_offset) :
                                    NULL;

        /* Blocking transfer. */
        nrfx_spi_xfer_desc_t desc = NRFX_SPI_XFER_TRX(tx_actual_buffer, tx_actual_length, rx_actual_buffer, rx_actual_length);
        result = nrfx_spi_xfer(nordic_nrf5_spi_instance[instance].ptr,
                               &desc, 0);
        /* Update loop variables. */
        tx_length -= tx_actual_length;
        tx_offset += tx_actual_length;

        rx_length -= rx_actual_length;
        rx_offset += rx_actual_length;
    }
#endif

    /* Manually set chip select pin if defined. */
    if (spi_inst->cs != NC) {
        nrf_gpio_pin_set(spi_inst->cs);
    }

    return (rx_offset < tx_offset) ? tx_offset : rx_offset;
}

/** Checks if the specified SPI peripheral is in use
 *
 * Parameter  obj The SPI peripheral to check
 * Return     non-zero if the peripheral is currently transmitting
 */
int spi_busy(spi_t *obj)
{
    /* Legacy API call. Always return zero. */
    return 0;
}

/** Get the module number
 *
 * Parameter  obj The SPI peripheral to check
 * Return     The module number
 */
uint8_t spi_get_module(spi_t *obj)
{
#if DEVICE_SPI_ASYNCH
    struct spi_s *spi_inst = &obj->spi;
#else
    struct spi_s *spi_inst = obj;
#endif

    return spi_inst->instance;
}

const PinMap *spi_master_mosi_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_master_miso_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_master_clk_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_master_cs_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_slave_mosi_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_slave_miso_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_slave_clk_pinmap()
{
    return PinMap_SPI_testing;
}

const PinMap *spi_slave_cs_pinmap()
{
    return PinMap_SPI_testing;
}

#if DEVICE_SPISLAVE

/** Check if a value is available to read
 *
 * Parameter  obj The SPI peripheral to check
 * Return     non-zero if a value is available
 */
int spi_slave_receive(spi_t *obj)
{
    return 0;
}

/** Get a received value out of the SPI receive buffer in slave mode
 *
 * Blocks until a value is available
 * Parameter  obj The SPI peripheral to read
 * Return     The value received
 */
int spi_slave_read(spi_t *obj)
{
    return 0;
}

/** Write a value to the SPI peripheral in slave mode
 *
 * Blocks until the SPI peripheral can be written to
 * Parameter  obj   The SPI peripheral to write
 * Parameter  value The value to write
 */
void spi_slave_write(spi_t *obj, int value)
{
    return;
}

#endif

#if DEVICE_SPI_ASYNCH

/***
 *                                               _____ _____
 *         /\                              /\   |  __ \_   _|
 *        /  \   ___ _   _ _ __   ___     /  \  | |__) || |
 *       / /\ \ / __| | | | '_ \ / __|   / /\ \ |  ___/ | |
 *      / ____ \\__ \ |_| | | | | (__   / ____ \| |    _| |_
 *     /_/    \_\___/\__, |_| |_|\___| /_/    \_\_|   |_____|
 *                    __/ |
 *                   |___/
 */

static ret_code_t spi_master_transfer_async_continue(spi_t *obj)
{
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    nrfx_spim_xfer_desc_t desc;
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    nrfx_spi_xfer_desc_t desc;
#endif
    /* Remaining data to be transferred. */
    size_t tx_length = obj->tx_buff.length - obj->tx_buff.pos;
    size_t rx_length = obj->rx_buff.length - obj->rx_buff.pos;

    /* Cap TX length to 255 bytes. */
    if (tx_length > 255) {
        tx_length = 255;
    }

    /* Cap RX length to 255 bytes. */
    if (rx_length > 255) {
        rx_length = 255;
    }

    desc.p_tx_buffer = ((const uint8_t *)(obj->tx_buff.buffer) + obj->tx_buff.pos);
    desc.p_rx_buffer = ((uint8_t *)(obj->rx_buff.buffer) + obj->rx_buff.pos);
    desc.tx_length = tx_length;
    desc.rx_length = rx_length;

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    ret_code_t result = nrfx_spim_xfer(nordic_nrf5_spim_instance[obj->spi.instance].ptr, &desc, 0);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    ret_code_t result = nrfx_spi_xfer(nordic_nrf5_spi_instance[obj->spi.instance].ptr, &desc, 0);
#endif
    return result;
}


#if NRFX_CHECK(NRFX_SPIS_ENABLED)
static void nordic_nrf5_spis_event_handler(nrfx_spis_evt_t const *p_event, void *p_context)
{
    // TODO, the SPIS event handling

    // TODO, could this just be merged into a common SPI/SPIM/SPIS event handler?
}
#endif

/* Callback function for driver calls. This is called from ISR context. */
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
static void nordic_nrf5_spi_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
static void nordic_nrf5_spi_event_handler(nrfx_spi_evt_t const *p_event, void *p_context)
#endif
{
    // Only safe to use with mbed-printf.
    //DEBUG_PRINTF("nordic_nrf5_twi_event_handler: %d %p\r\n", p_event->type, p_context);

    bool signal_complete = false;
    bool signal_error = false;

    spi_t *obj = (spi_t *) p_context;
    struct spi_s *spi_inst = &obj->spi;

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    if (p_event->type == NRFX_SPIM_EVENT_DONE) {
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    if (p_event->type == NRFX_SPI_EVENT_DONE) {
#endif

        /* Update buffers with new positions. */
        obj->tx_buff.pos += p_event->xfer_desc.tx_length;
        obj->rx_buff.pos += p_event->xfer_desc.rx_length;

        /* Setup a new transfer if more data is pending. */
        if ((obj->tx_buff.pos < obj->tx_buff.length) || (obj->rx_buff.pos < obj->tx_buff.length)) {

            /* Initiate SPI transfer. */
            ret_code_t result = spi_master_transfer_async_continue(obj);

            /* Abort if transfer wasn't accepted. */
            if (result != NRFX_SUCCESS) {

                /* Signal callback handler that transfer failed. */
                signal_error = true;
            }

        } else {

            /* Signal callback handler that transfer is complete. */
            signal_complete = true;
        }
    } else {

        /* Unexpected event, signal callback handler that transfer failed. */
        signal_error = true;
    }

    /* Transfer complete, signal success if mask is set.*/
    if (signal_complete) {

        /* Signal success if event mask matches and event handler is set. */
        if ((spi_inst->mask & SPI_EVENT_COMPLETE) && spi_inst->handler) {

            /* Cast handler to callback function pointer. */
            void (*callback)(void) = (void (*)(void)) spi_inst->handler;

            /* Reset object. */
            spi_inst->handler = 0;
            spi_inst->update = true;

            /* Store event value so it can be read back. */
            spi_inst->event = SPI_EVENT_COMPLETE;

            /* Signal callback handler. */
            callback();
        }

        /* Transfer failed, signal error if mask is set. */
    } else if (signal_error) {

        /* Signal error if event mask matches and event handler is set. */
        if ((spi_inst->mask & SPI_EVENT_ERROR) && spi_inst->handler) {

            /* Cast handler to callback function pointer. */
            void (*callback)(void) = (void (*)(void)) spi_inst->handler;

            /* Reset object. */
            spi_inst->handler = 0;
            spi_inst->update = true;

            /* Store event value so it can be read back. */
            spi_inst->event = SPI_EVENT_ERROR;

            /* Signal callback handler. */
            callback();
        }
    }

    /* Transfer completed one way or another. Set chip select manually if defined. */
    if (signal_complete || signal_error) {

        if (spi_inst->cs != NC) {
            nrf_gpio_pin_set(spi_inst->cs);
        }
    }
}

/** Begin the SPI transfer. Buffer pointers and lengths are specified in tx_buff and rx_buff
 *
 * Parameter  obj       The SPI object that holds the transfer information
 * Parameter  tx        The transmit buffer
 * Parameter  tx_length The number of bytes to transmit
 * Parameter  rx        The receive buffer
 * Parameter  rx_length The number of bytes to receive
 * Parameter  bit_width The bit width of buffer words
 * Parameter  event     The logical OR of events to be registered
 * Parameter  handler   SPI interrupt handler
 * Parameter  hint      A suggestion for how to use DMA with this transfer
 */
void spi_master_transfer(spi_t *obj,
                         const void *tx,
                         size_t tx_length,
                         void *rx,
                         size_t rx_length,
                         uint8_t bit_width,
                         uint32_t handler,
                         uint32_t mask,
                         DMAUsage hint)
{
    /* SPI peripheral only supports 8 bit transfers. */
    MBED_ASSERT(bit_width == 8);

    /* Setup buffers for transfer. */
    struct buffer_s *buffer_pointer;

    buffer_pointer = &obj->tx_buff;
    buffer_pointer->buffer = (void *) tx;
    buffer_pointer->length = tx_length;
    buffer_pointer->pos    = 0;
    buffer_pointer->width  = 8;

    buffer_pointer = &obj->rx_buff;
    buffer_pointer->buffer = rx;
    buffer_pointer->length = rx_length;
    buffer_pointer->pos    = 0;
    buffer_pointer->width  = 8;

    /* Save event handler and event mask so they can be called from interrupt handler. */
    struct spi_s *spi_inst = &obj->spi;
    spi_inst->handler = handler;
    spi_inst->mask = mask;

    /* Clear event flag. */
    spi_inst->event = 0;

    /* Force reconfiguration. */
    spi_inst->update = true;

    /* Configure peripheral if necessary. */
    spi_configure_driver_instance(obj);

    /* Manually clear chip select pin if defined. */
    if (spi_inst->cs != NC) {
        nrf_gpio_pin_clear(spi_inst->cs);
    }

    /* Initiate SPI transfer. */
    ret_code_t result = spi_master_transfer_async_continue(obj);

    /* Signal error if event mask matches and event handler is set. */
    if ((result != NRFX_SUCCESS) && (mask & SPI_EVENT_ERROR) && handler) {

        /* Cast handler to callback function pointer. */
        void (*callback)(void) = (void (*)(void)) handler;

        /* Reset object. */
        spi_inst->handler = 0;
        spi_inst->update = true;

        /* Store event value so it can be read back. */
        spi_inst->event = SPI_EVENT_ERROR;

        /* Signal callback handler. */
        callback();
    }
}

/** The asynchronous IRQ handler
 *
 * Reads the received values out of the RX FIFO, writes values into the TX FIFO and checks for transfer termination
 * conditions, such as buffer overflows or transfer complete.
 * Parameter  obj     The SPI object that holds the transfer information
 * Return     Event flags if a transfer termination condition was met; otherwise 0.
 */
uint32_t spi_irq_handler_asynch(spi_t *obj)
{
    /* Return latest event. */
    return obj->spi.event;
}

/** Attempts to determine if the SPI peripheral is already in use
 *
 * If a temporary DMA channel has been allocated, peripheral is in use.
 * If a permanent DMA channel has been allocated, check if the DMA channel is in use.  If not, proceed as though no DMA
 * channel were allocated.
 * If no DMA channel is allocated, check whether tx and rx buffers have been assigned.  For each assigned buffer, check
 * if the corresponding buffer position is less than the buffer length.  If buffers do not indicate activity, check if
 * there are any bytes in the FIFOs.
 * Parameter  obj The SPI object to check for activity
 * Return     Non-zero if the SPI port is active or zero if it is not.
 */
uint8_t spi_active(spi_t *obj)
{
    /* Callback handler is non-zero when a transfer is in progress. */
    return (obj->spi.handler != 0);
}

/** Abort an SPI transfer
 *
 * Parameter  obj The SPI peripheral to stop
 */
void spi_abort_asynch(spi_t *obj)
{
    int instance = obj->spi.instance;

    /* Abort transfer. */
#if NRFX_CHECK(NRFX_SPIM_ENABLED)
    nrfx_spim_abort(nordic_nrf5_spim_instance[instance].ptr);
#elif NRFX_CHECK(NRFX_SPI_ENABLED)
    nrfx_spi_abort(nordic_nrf5_spi_instance[instance].ptr);
#endif
    /* Force reconfiguration. */
    object_owner_spi2c_set(instance, NULL);
}

#endif // DEVICE_SPI_ASYNCH
#endif // DEVICE_SPI
