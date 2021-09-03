#include <cryptoauthlib.h>

#include <assert.h>
#include <fcntl.h>
#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <bsp.h>
#if defined LIBBSP_ARM_ATSAM_BSP_H
#define GRISP_I2C_BUS_PATH ATSAM_I2C_0_BUS_PATH
#define GRISP_I2C_REGISTER() atsam_register_i2c_0()
#elif defined LIBBSP_ARM_IMX_BSP_H
#define GRISP_I2C_BUS_PATH "/dev/i2c-1"
#define GRISP_I2C_REGISTER() i2c_bus_register_imx(GRISP_I2C_BUS_PATH, "i2c0")
#endif

#include <dev/i2c/i2c.h>

#include "atca_hal.h"

typedef struct grisp_i2c_data_s {
  int  ref_ct;
} grisp_i2c_data_t;

/** \brief HAL implementation of I2C init
 *
 * This doesn't really initialize anything yet, the read and write calls
 * are doing the initialization every time to not block the bus.
 *
 *  \param[in] hal pointer to HAL specific data that is maintained by this HAL
 *  \param[in] cfg pointer to HAL specific configuration data that is used to initialize this HAL
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_init(ATCAIface iface, ATCAIfaceCfg* cfg)
{
    ATCA_STATUS ret = ATCA_BAD_PARAM;

    if (!iface || !cfg)
    {
        return ret;
    }

    if (iface->hal_data)
    {
        grisp_i2c_data_t * hal_data = (grisp_i2c_data_t*)iface->hal_data;
        hal_data->ref_ct++;
        ret = ATCA_SUCCESS;
    }
    else
    {
        grisp_i2c_data_t *hal_data = malloc(sizeof(grisp_i2c_data_t));

        if (hal_data)
        {
            hal_data->ref_ct = 1;
            iface->hal_data = hal_data;
            ret = ATCA_SUCCESS;
        }
        else
        {
            ret = ATCA_ALLOC_FAILURE;
        }
    }

    return ret;
}

/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
    ((void)iface);
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C send
 * \param[in] iface         instance
 * \param[in] word_address  device transaction type
 * \param[in] txdata        pointer to space to bytes to send
 * \param[in] txlength      number of bytes to send
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t address, uint8_t *txdata, int txlength)
{
    int rv;
    int fd;
    grisp_i2c_data_t *hal_data = (grisp_i2c_data_t*)atgetifacehaldat(iface);

    if (!hal_data)
    {
        return ATCA_NOT_INITIALIZED;
    }

    rv = GRISP_I2C_REGISTER();
    assert(rv == 0);
    if ( (fd = open(GRISP_I2C_BUS_PATH, O_RDWR)) < 0)
    {
        return ATCA_COMM_FAIL;
    }

    // Set Device Address
    if (ioctl(fd, I2C_SLAVE, address >> 1) < 0)
    {
        close(fd);
        return ATCA_COMM_FAIL;
    }

    // Send data
    if (write(fd, txdata, txlength) != txlength)
    {
        close(fd);
        return ATCA_COMM_FAIL;
    }

    close(fd);
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C receive function
 * \param[in]    iface          Device to interact with.
 * \param[in]    address        device address
 * \param[out]   rxdata         Data received will be returned here.
 * \param[in,out] rxlength      As input, the size of the rxdata buffer.
 *                              As output, the number of bytes received.
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t address, uint8_t *rxdata, uint16_t *rxlength)
{
    int rv;
    int fd;
    grisp_i2c_data_t *hal_data = (grisp_i2c_data_t*)atgetifacehaldat(iface);

    if (!hal_data)
    {
        return ATCA_NOT_INITIALIZED;
    }

    rv = GRISP_I2C_REGISTER();
    assert(rv == 0);
    if ( (fd = open(GRISP_I2C_BUS_PATH, O_RDWR)) < 0)
    {
        return ATCA_COMM_FAIL;
    }

    // Set Device Address
    if (ioctl(fd, I2C_SLAVE, address >> 1) < 0)
    {
        close(fd);
        return ATCA_COMM_FAIL;
    }

    if (read(fd, rxdata, *rxlength) != *rxlength)
    {
        close(fd);
        return ATCA_COMM_FAIL;
    }

    close(fd);
    return ATCA_SUCCESS;
}

/** \brief Perform control operations for the kit protocol
 * \param[in]     iface          Interface to interact with.
 * \param[in]     option         Control parameter identifier
 * \param[in]     param          Optional pointer to parameter value
 * \param[in]     paramlen       Length of the parameter
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_control(ATCAIface iface, uint8_t option, void* param, size_t paramlen)
{
    (void)option;
    (void)param;
    (void)paramlen;

    if (iface && iface->mIfaceCFG)
    {
        /* This HAL does not support any of the control functions */
        return ATCA_UNIMPLEMENTED;
    }
    return ATCA_BAD_PARAM;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_release(void *hal_data)
{
    grisp_i2c_data_t *hal_data = (grisp_i2c_data_t*)hal_data;

    if (hal_data && --(hal_data->ref_ct) <= 0)
    {
        free(hal_data);
    }

    return ATCA_SUCCESS;
}

/** @} */
