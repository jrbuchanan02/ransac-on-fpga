`ifndef RANSAC_RANSAC_PKG_SVH
`define RANSAC_RANSAC_PKG_SVH

package ransac;

    typedef enum logic [1:0] {
        // the plane checking unit successfully ran an iteration of RANSAC
        PLANE_CHECKING_UNIT_STATUS_SUCCESS,
        // the plane checking unit encountered an error while deriving the
        // plane requested of it.
        PLANE_CHECKING_UNIT_STATUS_DERIVE_PLANE_ERROR,
        // the plane checking unit encountered a bus error (specifically
        // resp signal not success)
        PLANE_CHECKING_UNIT_STATUS_BUS_ERROR,
        // the plane checking unit waited for SO LONG that the plane checking
        // unit decided that that point just isn't coming.
        PLANE_CHECKING_UNIT_STATUS_BUS_TIMEOUT
    } plane_checking_unit_status_e;

    typedef enum logic [31:0] {
        RANSAC_UNIT_STATUS_IDLE_WITHOUT_ERROR,
        RANSAC_UNIT_STATUS_RUNNING,
        RANSAC_UNIT_STATUS_IDLE_AFTER_DERIVE_PLANE_ERRORS,
        RANSAC_UNIT_STATUS_IDLE_AFTER_BUS_ERRORS,
        RANSAC_UNIT_STATUS_IDLE_AFTER_BUS_TIMEOUTS
    } ransac_unit_status_e;

    typedef enum logic [1:0] {
        AXI_RESP_OKAY,
        AXI_RESP_EXOKAY,
        AXI_RESP_SLVERR,
        AXI_RESP_DECERR
    } axi_resp_value_e;

endpackage : ransac

`endif // ifndef RANSAC_RANSAC_PKG_SVH