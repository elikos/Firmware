#define SCIP2_CMD_MODE 		"SCIP2.0"
#define SCIP2_CMD_GET_RANGE2 	"MS"
#define SCIP2_CMD_GET_RANGE3 	"MD"
#define SCIP2_CMD_GET_LATEST2 	"GS"
#define SCIP2_CMD_GET_LATEST2 	"GD"
#define SCIP2_CMD_LASER_ON 	"BM"
#define SCIP2_CMD_LASER_OFF 	"QT"
#define SCIP2_CMD_RESET 	"RS"
#define SCIP2_CMD_SET_RATE 	"SS"
#define SCIP2_CMD_SET_MSPEED	"CR"
#define SCIP2_CMD_VERSION	"VV"
#define SCIP2_CMD_SPECS		"PP"
#define SCIP2_CMD_STATE 	"II"

/*
 * Calculate scip checksum of a byte array
 */
static char scip_checksum(const char buffer[], int size)
{
    unsigned char sum = 0x00;
    int i;

    for (i = 0; i < size; ++i) {
        sum += buffer[i];
    }

    return (sum & 0x3f) + 0x30;
}