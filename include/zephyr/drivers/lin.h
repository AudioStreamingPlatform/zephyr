#ifndef ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_
#define ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */
#define LIN_ID_MASK  0x3FU
#define LIN_NUM_ID   64U
#define LIN_MAX_DLEN 8U
#define LIN_BREAK_BITS 13U
/** @endcond */

static const uint8_t lin_pid[LIN_NUM_ID] = {
    0x80, 0xc1, 0x42, 0x03, 0xc4, 0x85, 0x06, 0x47,
    0x08, 0x49, 0xca, 0x8b, 0x4c, 0x0d, 0x8e, 0xcf,
    0x50, 0x11, 0x92, 0xd3, 0x14, 0x55, 0xd6, 0x97,
    0xd8, 0x99, 0x1a, 0x5b, 0x9c, 0xdd, 0x5e, 0x1f,
    0x20, 0x61, 0xe2, 0xa3, 0x64, 0x25, 0xa6, 0xe7,
    0xa8, 0xe9, 0x6a, 0x2b, 0xec, 0xad, 0x2e, 0x6f,
    0xf0, 0xb1, 0x32, 0x73, 0xb4, 0xf5, 0x76, 0x37,
    0x78, 0x39, 0xba, 0xfb, 0x3c, 0x7d, 0xfe, 0xbf,
};

/**
 * Sync field: The commander sends a sync field, which 
 * is a predefined pattern (0x55). This allows all nodes 
 * to synchronize their baud rates with the commander. 
 */
#define LIN_SYNC_FIELD 0x55

/**
 * @brief LIN controller mode
 */
enum lin_mode {
    /** Commander mode */
    LIN_MODE_COMMANDER,
    /** Responder mode */
    LIN_MODE_RESPONDER,
};

/**
 * @brief LIN responder action in response to header
 */
enum lin_action {
    /** Ignore message */
    LIN_ACTION_NONE,
    /** Send response */
    LIN_ACTION_SEND,
    /** Receive message */
    LIN_ACTION_RECEIVE,
};

/**
 * @brief LIN checksum type
 */
enum lin_checksum {
    /** Classic */
    LIN_CHECKSUM_CLASSIC,
    /** Enhanced */
    LIN_CHECKSUM_ENHANCED,
    /** Auto (receive only, match either) */
    LIN_CHECKSUM_AUTO,
};

/**
 * @brief LIN frame structure
 */
struct lin_frame {
    /** LIN identifier (0-63) */
    uint8_t id;
    /** Checksum type. Use @a lin_checksum enum for assignment */
    uint8_t type;
    /** Data length */
    uint8_t len;
    /** @cond INTERNAL_HIDDEN */
    uint8_t res;  /* reserved/padding. */
    /** @endcond */
    /** Frame payload data */
    uint8_t data[LIN_MAX_DLEN];
};

/**
 * @brief Defines the application callback handler function signature for header reception
 *
 * Note this function may be called from an ISR context. Since this may delay the response being processed,
 * keep this function as short as possible. Only applicable to responder mode, only will be called on successful
 * header reception.
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param frame     Pointer to frame with the id field populated. If sending a response, populate
 *                  the type, len, and data fields.
 * @param user_data User data provided when the callback was set.
 * 
 * @retval Specifies how the controller should respond. See @a lin_action enum.
 */
typedef int (*lin_header_callback_t)(const struct device *dev, struct lin_frame *frame, void *user_data);

/**
 * @brief Defines the application callback handler function signature for frame sending
 *
 * Note this function may be called from an ISR context. Since this may delay the next packet being processed,
 * keep this function as short as possible. This is guaranteed to be called sometime after a call to @a lin_send()
 * in commander mode and after header reception in responder mode when sending.
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param error     Status of the performed send operation. See the list of return values for 
 *                  @a lin_send() for value descriptions.
 * @param user_data User data provided when the callback was set.
 */
typedef void (*lin_tx_callback_t)(const struct device *dev, int error, void *user_data);

/**
 * @brief Defines the application callback handler function signature for frame reception
 * 
 * Note this function may be called from an ISR context. Since this may delay the next packet being processed,
 * keep this function as short as possible. This is guaranteed to be called sometime after a call to @a lin_receive()
 * in commander mode and after header reception in responder mode when receiving.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param error     Status of the performed receive operation. See the list of return values for
 *                  @a lin_receive() for value descriptions.
 * @param frame     Received frame, valid only during callback.
 * @param user_data User data provided when the filter was added.
 */
typedef void (*lin_rx_callback_t)(const struct device *dev, int error, const struct lin_frame *frame, void *user_data);

struct lin_callbacks {
	lin_header_callback_t header;
	void *header_data;
	lin_tx_callback_t tx;
	void *tx_data;
	lin_rx_callback_t rx;
	void *rx_data;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
 */

/**
 * @brief API for setting LIN controller mode
 * See @a lin_set_mode() for argument description
 */
typedef int (*lin_set_mode_t)(const struct device *dev, enum lin_mode mode);

/**
 * @brief API for setting LIN controller bitrate
 * See @a lin_set_bitrate() for argument description
 */
typedef int (*lin_set_bitrate_t)(const struct device *dev, uint32_t bitrate);

/**
 * @brief API for setting LIN controller header callback
 * See @a lin_set_header_callback() for argument description
 */
typedef int (*lin_set_header_callback_t)(const struct device *dev, lin_header_callback_t callback, void *user_data);

/**
 * @brief API for setting LIN controller tx callback
 * See @a lin_set_tx_callback() for argument description
 */
typedef int (*lin_set_tx_callback_t)(const struct device *dev, lin_tx_callback_t callback, void *user_data);

/**
 * @brief API for setting LIN controller rx callback
 * See @a lin_set_rx_callback() for argument description
 */
typedef int (*lin_set_rx_callback_t)(const struct device *dev, lin_rx_callback_t callback, void *user_data);

/**
 * @brief API for sending a LIN frame as commander
 * See @a lin_send() for argument description
 */
typedef int (*lin_send_t)(const struct device *dev, const struct lin_frame *frame);

/**
 * @brief API for receiving a LIN frame as commander
 * See @a lin_receive() for argument description
 */
typedef int (*lin_receive_t)(const struct device *dev, uint8_t id, enum lin_checksum type, uint8_t len);

__subsystem struct lin_driver_api {
    lin_set_mode_t set_mode;
    lin_set_bitrate_t set_bitrate;
    lin_set_header_callback_t set_header_callback;
    lin_set_tx_callback_t set_tx_callback;
    lin_set_rx_callback_t set_rx_callback;
    lin_send_t send;
    lin_receive_t receive;
};

/** @endcond */

/**
 * @brief Set the LIN controller to the given operation mode
 * 
 * Sets the LIN controller to the given operation mode.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 * @param mode Operation mode.
 *
 * @retval 0 if successful.
 * @retval -EINVAL invalid mode
 */
__syscall int lin_set_mode(const struct device *dev, enum lin_mode mode);

static inline int z_impl_lin_set_mode(const struct device *dev, enum lin_mode mode) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_mode(dev, mode);
}

/**
 * @brief Set the LIN controller bitrate
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param bitrate Bitrate in bits/sec
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP bitrate not supported by LIN controller/transceiver combination
 * @retval -EINVAL bitrate cannot be met.
 */
__syscall int lin_set_bitrate(const struct device *dev, uint32_t bitrate);

static inline int z_impl_lin_set_bitrate(const struct device *dev, uint32_t bitrate) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_bitrate(dev, bitrate);
}

/**
 * @brief Set the LIN controller header callback
 * 
 * Sets the LIN controller callback for frame header reception.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param callback  The callback
 * @param user_data User data to pass to the callback function
 *
 * @retval 0 if successful.
 * @retval -EIO General input/output error, failed to configure device.
 */
__syscall int lin_set_header_callback(const struct device *dev, lin_header_callback_t callback, void *user_data);

static inline int z_impl_lin_set_header_callback(const struct device *dev, lin_header_callback_t callback, void *user_data) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_header_callback(dev, callback, user_data);
}

/**
 * @brief Set the LIN controller frame sending callback
 * 
 * Sets the LIN controller callback for when done sending frame.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param callback  The callback
 * @param user_data User data to pass to the callback function
 *
 * @retval 0 if successful.
 * @retval -EIO General input/output error, failed to configure device.
 */
__syscall int lin_set_tx_callback(const struct device *dev, lin_tx_callback_t callback, void *user_data);

static inline int z_impl_lin_set_tx_callback(const struct device *dev, lin_tx_callback_t callback, void *user_data) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_tx_callback(dev, callback, user_data);
}

/**
 * @brief Set the LIN controller frame reception callback
 * 
 * Sets the LIN controller callback for when done receiving frame.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param callback  The callback
 * @param user_data User data to pass to the callback function
 *
 * @retval 0 if successful.
 * @retval -EIO General input/output error, failed to configure device.
 */
__syscall int lin_set_rx_callback(const struct device *dev, lin_rx_callback_t callback, void *user_data);

static inline int z_impl_lin_set_rx_callback(const struct device *dev, lin_rx_callback_t callback, void *user_data) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_rx_callback(dev, callback, user_data);
}

/**
 * @brief For commander mode, initiate a LIN frame for transmission
 *
 * Only applicable for commander mode. If possible, sends the frame. Driver may implement a FIFO queue.
 * Callback from @a lin_set_tx_callback() will be called when frame is done sending or error.
 * 
 * @param dev   Pointer to the device structure for the driver instance.
 * @param frame LIN frame to transmit.
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP if not in correct controller mode
 * @retval -EINVAL if an invalid parameter was passed to the function.
 * @retval -EIO if a general transmit error occurred (e.g. sent/received byte mismatch, invalid checksum).
 */
__syscall int lin_send(const struct device *dev, const struct lin_frame *frame);

static inline int z_impl_lin_send(const struct device *dev, const struct lin_frame *frame) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->send(dev, frame);
}

/**
 * @brief For commander mode, initiate the reception of data from a LIN responder node.
 *
 * Only applicable for commander mode. If possible, initiates receiving a frame. Driver may implement a
 * FIFO queue for receiving multiple frames. Callback from @a lin_set_rx_callback will be called when frame
 * is received or error.
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param id        The ID to initiate the receive from.
 * @param type The type of checksum to look for.
 * @param len  The length of message to look for. Set to 0 for automatic length determination based on checksum.
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP if not supported or in incorrect mode.
 * @retval -EINVAL if invalid argument.
 * @retval -EIO if a general transmit error occurred (e.g. sent/received byte mismatch, invalid checksum, no response).
 */
__syscall int lin_receive(const struct device *dev, uint8_t id, enum lin_checksum type, uint8_t len);

static inline int z_impl_lin_receive(const struct device *dev, uint8_t id, enum lin_checksum type, uint8_t len) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->receive(dev, id, type, len);
}

/* FIXME: add documentation */
static inline uint8_t lin_pid_from_index(uint8_t index)
{
	__ASSERT(index < LIN_NUM_ID,
		 "index exceeds lin_num_id (64)");
	return lin_pid[index];
}

static inline uint8_t lin_calculate_checksum(const struct lin_frame *frame, enum lin_checksum type)
{
	uint16_t checksum = type == LIN_CHECKSUM_ENHANCED ? lin_pid_from_index(frame->id) : 0;
	for (uint8_t i = 0; i < frame->len; i++) {
		checksum += frame->data[i];
		if (checksum > 0xff) {
			checksum -= 0xff;
		}
	}
	return (uint8_t)(255 - checksum);
}

static inline bool lin_verify_checksum(const struct lin_frame *frame, uint8_t expected)
{
	if (frame->type == LIN_CHECKSUM_AUTO) {
		const uint8_t classic = lin_calculate_checksum(frame, LIN_CHECKSUM_CLASSIC);
		const uint8_t enhanced = lin_calculate_checksum(frame, LIN_CHECKSUM_ENHANCED);

		return (classic == expected) || (enhanced == expected);
	}

	return lin_calculate_checksum(frame, frame->type) == expected;
}

static inline k_timeout_t lin_break_time(const uint32_t baud_rate)
{
	return K_USEC(((float)LIN_BREAK_BITS / baud_rate) * Z_HZ_us);
}

#ifdef __cplusplus
}
#endif

#include <syscalls/lin.h>

#endif // ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_
