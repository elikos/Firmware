#define SCIP2_CMD_MODE 		"SCIP2.0"
#define SCIP2_CMD_GET_RANGE2 	"MS"
#define SCIP2_CMD_GET_RANGE3 	"MD"
#define SCIP2_CMD_GET_LATEST2 	"GS"
#define SCIP2_CMD_GET_LATEST3 	"GD"
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

static int byteDecode3(uint8_t first, uint8_t second, uint8_t third)
{
	const uint8_t mask = 0x003f;	//keep first 6 bits
	first	-= 0x30;
	second	-= 0x30;
	third	-= 0x30;

	//get first 6 bits
	uint32_t _first = first	& mask;
	uint32_t _second = second & mask;
	uint32_t _third = third	& mask;

	int result = 0;
	result = (first << 12) | (_second << 6) | _third;
}