

#ifndef _PBIO_ERROR_H_
#define _PBIO_ERROR_H_

#include <pbio/config.h>

/**
 * Error code.
 */
typedef enum {
    PBIO_SUCCESS,               /**< No error */
    PBIO_ERROR_FAILED,          /**< Unspecified error */
    PBIO_ERROR_INVALID_ARG,     /**< Invalid argument */
    PBIO_ERROR_INVALID_PORT,    /**< Invalid port identifier */
    PBIO_ERROR_IO,              /**< General I/O error */
    PBIO_ERROR_NO_DEV,          /**< Device is not connected */
} pbio_error_t;

#endif // _PBIO_ERROR_H_
