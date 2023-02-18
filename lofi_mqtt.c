//
// lofi_mqtt.c:
//	Receive and process lofi security packets.
//	on odroid or Raspberry Pi
//	on odroid spi not working correctly so I added a bit-bang spi
//	when using bit-bang spi don't add the following modules, otherwise do add them...
//	spicc
//	spidev
//	aml_i2c
//


#include <libgen.h>
#include <sys/signalfd.h>
#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mosquitto.h>   // sudo apt install libmosquitto-dev


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <strings.h>
#include <fcntl.h>
#include <netdb.h>

// Server side C program to demonstrate HTTP Server programming
#include <netinet/in.h>
#include <string.h>


#define NRFIRQ			5
#define nrfCSN			10
#define nrfCE			6

#define SPI_BIT_BANG	1

#if SPI_BIT_BANG
#define MOSI_PIN		12
#define MISO_PIN		13
#define SCLK_PIN		14
#endif

typedef struct {
	uint8_t		nodeId;
	uint8_t		lastState	:1;
	uint8_t		closed		:1;
	uint8_t		seq			:2;
	uint8_t		sensorId	:4;
	uint8_t		hi			:4;
	uint8_t		rsvd		:4;
	uint8_t		mid;
	uint8_t		low;
} sensor_t;

#define PAYLOAD_LEN		sizeof(sensor_t)

typedef struct {
	uint	nbrMsgs;
	int		sw1;
	float	vcc;
	float	atemp;
	float	ahumd;
} node_status_t;


#define handle_error(msg) \
	do { perror(msg); /*exit(EXIT_FAILURE);*/ } while (0)

// define the NRF24l01+ register addresses
#define NRF_CONFIG			0x00
#define NRF_EN_AA			0x01
#define NRF_EN_RXADDR		0x02
#define NRF_SETUP_AW		0x03
#define NRF_SETUP_RETR		0x04
#define NRF_RF_CH			0x05
#define NRF_RF_SETUP		0x06
#define NRF_STATUS			0x07
#define NRF_OBSERVE_TX		0x08
#define NRF_CD				0x09
#define NRF_RX_ADDR_P0		0x0A
#define NRF_RX_ADDR_P1		0x0B
#define NRF_RX_ADDR_P2		0x0C
#define NRF_RX_ADDR_P3		0x0D
#define NRF_RX_ADDR_P4		0x0E
#define NRF_RX_ADDR_P5		0x0F
#define NRF_TX_ADDR			0x10
#define NRF_RX_PW_P0		0x11
#define NRF_RX_PW_P1		0x12
#define NRF_RX_PW_P2		0x13
#define NRF_RX_PW_P3		0x14
#define NRF_RX_PW_P4		0x15
#define NRF_RX_PW_P5		0x16
#define NRF_FIFO_STATUS		0x17
#define NRF_DYNPD			0x1C
#define NRF_FEATURE			0x1D

#if (!defined(TRUE))
#define TRUE 1
#endif
	
#if (!defined(FALSE))
#define FALSE 1
#endif

#define SECS_IN_DAY	(24*60*60)

typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR,
	SENID_REV,
	SENID_ATEMP,
	SENID_AHUMD
} senId_t;


typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;

//unsigned char payload[PAYLOAD_LEN];
int mqttStr = 0;
int printPayload = 0;
int printSeq = 0;
int en_shockburst = 1;
char *pgmName = NULL;
speed_t speed = speed_250K;
int rf_chan = 84;
int maxNodeRcvd = 0;
int verbose = 0;
int printTime = 0;
int en_epoch = 0;
int nrfIrq = NRFIRQ;
int en_CRC1 = 0;
uint8_t	nrf_config = 0x3C;
// p0Addr[0] is LSByte which should be written first
uint8_t p0Addr[] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
// p15Addr[0] is LSByte which should be written first
uint8_t p15Addr[] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };
uint8_t pxAddr[] = { 0xC3, 0xC4, 0xC5, 0xC6 };
uint8_t nrfStatus;

const char *swState[] = { "OPEN", "SHUT", " -- " };

#if !SPI_BIT_BANG
static int spiFd;
#endif


struct	mosquitto	*mosq = NULL;
char	*mosq_host = "omv";
int		mosq_port = 1883;
int		mosq_keepalive = 60;
char	*mosq_user = (char*)NULL;
char	*mosq_pass = (char*)NULL;
bool	mosq_clean_session = true;

#if 0
char *nodeMap[] = {
	"node/0",			// node/0, there never will be a node 0
	"node/1",			// node/1, retired
	"node/2",			// node/2, retired
	"node/3",			// node/3, retired
	"node/4",			// node/4, retired
	"door/GarageN",		// node/5
	"node/6",			// node/6, retired
	"node/7",
	"door/Hall",		// node/8,
	"node/9",			// node/9, retired
	"node/10",			// node/10, retired
	"node/11",			// node/11, active - testing
	"node/12",			// node/12, retired
	"node/13",			// node/13, retired
	"door/Garage",		// node/14,
	"node/15",			// node/15, retired
	"node/16",			// node/16, retired
	"node/17",			// node/17, active - testing
	"node/18",			// node/18, retired
	"door/Back",		// node/19
	"node/20",
	"door/ToyRm",		// node/21
	"window/LivingE",	// node/22, rev 0.1 PWB, Aliexpress NRF25l01+, RF_SETUP[0]=0, crappy reed switch???
	"node/23",
	"window/OfficeS",	// node/24, rev 0.2 PWB, Aliexpress Si24R1,    RF_SETUP[0]=0
	"door/GarageS",		// node/25, rev 0.2 PWB, Aliexpress Si24R1,    RF_SETUP[0]=1
	"window/MasterE",	// node/26
	"node/27",			// node/27, rev 0.3 PWB, Aliexpress Si24R1,    RF_SETUP[0]=0
	"door/Front",		// node/28, rev 0.3 PWB, RFM75,                RF_SETUP[0]=1
	"node/29",			// node/29, rev 0.3 PWB, Aliexpress NRF24l01+, RF_SETUP[0]=0
	"door/GuestBr",		// node/30, rev 0.4 PWB, Aliexpress NRF24l01+, RF_SETUP[0]=0
	"node/31",			// node/31, rev 0.4 PWB, RFM75,                RF_SETUP[0]=0
	"door/Sliding",		// node/32, rev 0.2 PWB, Aliexpress NRF24l01+, RF_SETUP[0]=1
	"node/33",			// node/33, rev 0.5 PWB, RFM75
	"window/LivingW",	// node/34, rev 0.5 PWB, RFM75
	"node/35",			// node/35, rev 0.4 PWB, RFM75, moded to use Aliexpress AHT10 PWB
	"node/36",			// node/36, rev 0.6 PWB, RFM75, AHT10
	"node/37",
	"node/38",
	"node/99"
};
#endif

#define maxNodes  60
//const int maxNodes = sizeof(nodeMap)/sizeof(char*);

volatile node_status_t nodeStatus[maxNodes];

sem_t count_sem;


//************  Forward Declarations
//int parse_payload( uint8_t *payload );
PI_THREAD (parse_payload);
PI_THREAD (http_server);
void spiSetup( int speed );
int spiXfer( uint8_t *buf, int cnt );
uint8_t nrfRegRead( int reg );
uint8_t nrfRegWrite( int reg, int val );
void nrfPrintDetails(void);
uint8_t nrfAvailable( uint8_t *pipe_num );
uint8_t nrfRegsWrite( int reg, uint8_t *buf, int len);
int nrfRead( uint8_t *payload, int len );
uint8_t nrfFlushTx( void );
uint8_t nrfFlushRx( void );
uint8_t nrfAddrRead( uint8_t reg, uint8_t *buf, int len );
uint8_t nrfReadRxPayloadLen(void);
void mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str);


void mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
	// Print all log messages regardless of level.
  
	switch (level) {
	//case MOSQ_LOG_DEBUG:
	//case MOSQ_LOG_INFO:
	//case MOSQ_LOG_NOTICE:
	case MOSQ_LOG_WARNING:
	case MOSQ_LOG_ERR: 
		printf("%i:%s\n", level, str);
		break;
	default:
		break;
	}
}

// must be a power of two
#define	PLDBUF_SIZE		8
uint32_t	pldBuf[PLDBUF_SIZE];
uint32_t	pldBufWr, pldBufRd;

#if 1
void nrfIntrHandler(void)
{
//	uint8_t pipeNum __attribute__ ((unused));
//	uint8_t payLen __attribute__ ((unused));
//	unsigned char payload[PAYLOAD_LEN];

////	nrfRegWrite( NRF_STATUS, 0x70);
	if (sem_post(&count_sem) == -1) {
		perror("sem_post");
	}
//	payLen = nrfReadRxPayloadLen();
//	if (payLen != PAYLOAD_LEN) {
//		fprintf(stderr, "PAYLOAD LEN: %d\n", payLen);
//		fflush(stderr);
//	}

//	nrfRead( payload, payLen );

//	pldBuf[pldBufWr] = *(uint32_t *)payload;
//	pldBufWr = (pldBufWr + 1) & (PLDBUF_SIZE-1);

//	parse_payload( payload );
}
#else
void nrfIntrHandler(void)
{
	uint8_t pipeNum __attribute__ ((unused));
	uint8_t payLen __attribute__ ((unused));
	unsigned char payload[PAYLOAD_LEN];

	payLen = nrfReadRxPayloadLen();
	if (payLen != PAYLOAD_LEN) {
		fprintf(stderr, "PAYLOAD LEN: %d\n", payLen);
		fflush(stderr);
	}

	nrfRead( payload, payLen );

	pldBuf[pldBufWr] = *(uint32_t *)payload;
	pldBufWr = (pldBufWr + 1) & (PLDBUF_SIZE-1);

//	parse_payload( payload );
}
#endif

void sig_handler( int sig )
{
	if (sig == SIGINT) {
		printf("\nPowering down receiver...\n");
    	digitalWrite(nrfCE, LOW);
		printf("Disconnecting from MQTT broker\n");
		mosquitto_disconnect(mosq);
		mosquitto_lib_cleanup();
		exit(0);
	}
}


int Usage(void)
{
	fprintf(stderr, "Usage: %s [-hvbmpsSteqC] [-P n] [-H s] [-U s] [-W s] [-c chan] [-g pin] [-x \"1,3-5\"] [-f \"1,3-5\"]\n", pgmName);
	fprintf(stderr, "  -h	this message\n");
	fprintf(stderr, "  -v	verbose\n");
	fprintf(stderr, "  -b	disable shockBurst mode\n");
	fprintf(stderr, "  -m	print output in MQTT format\n");
	fprintf(stderr, "  -p	print out payload in hex\n");
	fprintf(stderr, "  -s	set receive RF bit rate to 1M (default is 250kbps)\n");
	fprintf(stderr, "  -S	set receive RF bit rate to 2M (default is 250kbps)\n");
	fprintf(stderr, "  -t	print timestamp\n");
	fprintf(stderr, "  -e	print timestamp in epoc time\n");
	fprintf(stderr, "  -q	print packet sequence nbr\n");
	fprintf(stderr, "  -C	use 2 byte CRC (default is 1 byte)\n");
	fprintf(stderr, "  -P n	set MQTT port to 'n' (default is 1883)\n");
	fprintf(stderr, "  -H s	set MQTT host to 's' (default is 'omv')\n");
	fprintf(stderr, "  -U s	set MQTT username to 's' (default is not used)\n");
	fprintf(stderr, "  -W s	set MQTT password to 's' (default is not used)\n");
	fprintf(stderr, "  -g n	use GPIO pin 'n' for IRQ input (default is pin 5)\n");
	fprintf(stderr, "  -c n	set RF receive channel to 'n' (default is 84)\n");
	fprintf(stderr, "  -x s	exclude nodes: e.g. s = \"1,2,3-5,7\"\n");
	fprintf(stderr, "  -f s	include nodes: e.g. s = \"1,2,3-5,7\"\n");
	return 0;
}


//
// main
//
int main(int argc, char *argv[])
{
	int i __attribute__ ((unused));
	uint8_t val8 __attribute__ ((unused));
	int opt;

	pgmName = basename(argv[0]);

	while ((opt = getopt(argc, argv, "hvCWmpsSteqc:x:f:g:P:H:")) != -1) {
		switch (opt) {
		case 'h':
			Usage();
			exit(0);
			break;
		case 'm':
			mqttStr = 1;
			break;
		case 'C':
			en_CRC1 = 1;
			break;
		case 'b':
			en_shockburst = 0;
			break;
		case 'p':
			printPayload = 1;
			break;
		case 'q':
			printSeq = 1;
			break;
		case 'g':
			nrfIrq = atoi(optarg);
			break;
		case 's':
			speed = speed_1M;
			break;
		case 'S':
			speed = speed_2M;
			break;
		case 't':
			printTime = 1;
			en_epoch = 0;
			break;
		case 'e':
			printTime = 1;
			en_epoch = 1;
			break;
		case 'H':
			mosq_host = optarg;
			break;
		case 'P':
			mosq_port = atoi(optarg);
			break;
		case 'U':
			mosq_user = optarg;
			break;
		case 'W':
			mosq_pass = optarg;
			break;
		case 'c':
			rf_chan = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		default:
			Usage();
			exit(0);
			break;
		}
	}

	for (int i = 0; i < maxNodes; i++) {
		nodeStatus[i].nbrMsgs = 0;
		nodeStatus[i].sw1 = 2;
		nodeStatus[i].vcc = -50.0;
		nodeStatus[i].atemp = -50.0;
		nodeStatus[i].ahumd = -50.0;
	}

	if (sem_init(&count_sem, 0, 0) == -1) {
		perror ("sem_init"); exit(1);
	}

//	atexit(printStats);
	if (signal(SIGINT, sig_handler) == SIG_ERR)
		fprintf(stderr, "Can't catch SIGINT\n");

    // setup mqtt for publishing
	mosquitto_lib_init();
	mosq = mosquitto_new(NULL, mosq_clean_session, NULL);
	if (!mosq) {
		fprintf(stderr, "Error: Out of memory.\n");
		exit(1);
	}
  
	mosquitto_log_callback_set(mosq, mosq_log_callback);

	if (mosq_user != (char*)NULL) {
		if (mosquitto_username_pw_set(mosq, mosq_user, mosq_pass)) {
			fprintf(stderr, "Unable to set mosq username or password");
			exit(1);
		}
	}
  
	if (mosquitto_connect(mosq, mosq_host, mosq_port, mosq_keepalive)) {
		fprintf(stderr, "Unable to connect.\n");
		exit(1);
	}
	int loop = mosquitto_loop_start(mosq);
	if (loop != MOSQ_ERR_SUCCESS) {
		fprintf(stderr, "Unable to start loop: %i\n", loop);
		exit(1);
	}
  
	wiringPiSetup();

	pinMode(nrfCSN, OUTPUT);
    digitalWrite(nrfCSN, HIGH);
	pinMode(nrfCE, OUTPUT);
    digitalWrite(nrfCE, LOW);

#if SPI_BIT_BANG
	if (verbose)
		printf("BIT BANG SPI\n");
	pinMode(MOSI_PIN, OUTPUT);
	digitalWrite(MOSI_PIN, LOW);
	pinMode(SCLK_PIN, OUTPUT);
	digitalWrite(SCLK_PIN, LOW);
	pinMode(MISO_PIN, INPUT);
#else
    spiSetup(1000000);
#endif

	// NRF setup
	// enable 8-bit CRC; mask TX_DS and MAX_RT
	if (en_CRC1)
		nrf_config &= ~(1<<2);
	nrfRegWrite( NRF_CONFIG, nrf_config );

	if (en_shockburst) {
		// set nbr of retries and delay
		//	nrfRegWrite( NRF_SETUP_RETR, 0x5F );
		nrfRegWrite( NRF_SETUP_RETR, 0x77 );
		nrfRegWrite( NRF_EN_AA, 0x3F ); // enable auto ack
	} else {
		nrfRegWrite( NRF_SETUP_RETR, 0 );
		nrfRegWrite( NRF_EN_AA, 0 );
	}

	nrfRegsWrite(NRF_TX_ADDR, p0Addr, 5);
	nrfRegsWrite(NRF_RX_ADDR_P0, p0Addr, 5);
	nrfRegsWrite(NRF_RX_ADDR_P1, p15Addr, 5);
	nrfRegWrite(NRF_RX_ADDR_P2, pxAddr[0]);
	nrfRegWrite(NRF_RX_ADDR_P3, pxAddr[1]);
	nrfRegWrite(NRF_RX_ADDR_P4, pxAddr[2]);
	nrfRegWrite(NRF_RX_ADDR_P5, pxAddr[3]);

	// Disable dynamic payload
	nrfRegWrite( NRF_FEATURE, 0);
	nrfRegWrite( NRF_DYNPD, 0);

	// Reset STATUS
	nrfRegWrite( NRF_STATUS, 0x70 );

	nrfRegWrite( NRF_EN_RXADDR, 0x3F ); //3);
	nrfRegWrite( NRF_RX_PW_P0, PAYLOAD_LEN );

	nrfRegWrite( NRF_RX_PW_P1, PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P2, PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P3, PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P4, PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P5, PAYLOAD_LEN );

	// Set up channel
	nrfRegWrite( NRF_RF_CH, rf_chan );

	switch (speed) {
	case speed_1M:
		val8 = 0x06;
		break;
	case speed_2M:
		val8 = 0x0e;
		break;
	case speed_250K:
		val8 = 0x26;
		break;
	default:
		val8 = 0x26;
		break;
	}
	nrfRegWrite( NRF_RF_SETUP, val8); // | 1 );

	nrfFlushTx();
	nrfFlushRx();

//	piHiPri(50);

	int x = piThreadCreate (http_server);
	if (x != 0)
		fprintf(stderr, "http_server didn't start...\n");

	x = piThreadCreate (parse_payload);
	if (x != 0)
		fprintf(stderr, "it didn't start...\n");

	wiringPiISR(nrfIrq, INT_EDGE_FALLING, &nrfIntrHandler);

	// Power up radio and delay 5ms
	nrf_config |= 0x2;
	nrfRegWrite( NRF_CONFIG, nrf_config );
//	nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x02 );
	delay(5);

	// Enable PRIME RX (PRX)
	nrf_config |= 0x01;
	nrfRegWrite( NRF_CONFIG, nrf_config );
	//nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x01 );

	if (verbose)
		nrfPrintDetails();

    digitalWrite(nrfCE, HIGH);

//	nrfRegWrite( NRF_EN_RXADDR, 3 );

	nrfFlushRx();

	for (;;) {
		sleep(10);
		//delay(10000);
	}

	return 0;
}

int showPayload( uint8_t *payload )
{
	printf("Payload:");
	for (int i = 0; i < PAYLOAD_LEN; i++)
		printf(" %02X", payload[i]);
	printf("\n");
	return 0;
}

//void parse_payload( void )
PI_THREAD (parse_payload)
{
	struct timespec ts;
	struct tm mt;
	unsigned short val;
	int val1;
	int	rv;
	uint8_t	sensorId;
	uint8_t nodeId;
	char tbuf[128];
	char topic[40];
	char topicVal[40];
	int topicIdx = 0;
	int		tbufIdx = 0;
	int		seq = 0;
	uint8_t	payload[8];
	int pkt_avail = false;
	sensor_t *pl = (sensor_t *)payload;


	for (;;) {

		if (!pkt_avail) {
			if (sem_wait(&count_sem) == -1) {
				perror("sem_wait");
			}
		}

		clock_gettime(CLOCK_REALTIME, &ts);
		localtime_r(&ts.tv_sec, &mt);

		rv = nrfRead( payload, PAYLOAD_LEN );

		if (rv) {
#if 0
			if (printTime) {
				//printf("%d  ", (int)ts.tv_sec);
				if (en_epoch)
					printf("%4d.%03d  ",
						(int) (ts.tv_sec % 10000),
						(int) (ts.tv_nsec/1000000)); // .%03ld
				else
					printf("%02d:%02d:%02d.%03d  ",
						mt.tm_hour, mt.tm_min, mt.tm_sec,
						(int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
			}

			printf("Bad: %d\n", rv);
#endif
			pkt_avail = false;
			continue;
		}

	//fprintf(stderr, "FIFO_STATUS: %02X\n", nrfRegRead(NRF_FIFO_STATUS));
	//fflush(stderr);



	//*(uint32_t *)payload	= pldBuf[pldBufRd];
	//pldBufRd = (pldBufRd + 1) & (PLDBUF_SIZE-1);

	topicIdx = 0;
	tbufIdx = 0;
	tbuf[0] = '\0';
//	sbuf[0] = '\0';
	topic[0] = '\0';


//	nodeId = payload[0];
	nodeId = pl->nodeId;

	if (nodeId < 1 || nodeId >= maxNodes) {
		if (printTime) {
			//printf("%d  ", (int)ts.tv_sec);
			if (en_epoch)
				printf("%4d.%03d  ",
					(int) (ts.tv_sec % 10000),
					(int) (ts.tv_nsec/1000000)); // .%03ld
			else
				printf("%02d:%02d:%02d.%03d  ",
					mt.tm_hour, mt.tm_min, mt.tm_sec,
					(int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
		}
		printf("Bad nodeId: %d\n", nodeId);
		showPayload(payload);
//		printf("Payload: %02X %02X %02X\n", payload[0], payload[1], payload[2]);
		fflush(stdout);
		pkt_avail = false;
		continue;
	}


//	topicIdx = snprintf(&topic[topicIdx], 127-topicIdx, "lofi/%s", nodeMap[nodeId]);
	topicIdx = snprintf(&topic[topicIdx], 127-topicIdx, "lofi/node/%d", nodeId);

	if (!mqttStr && printTime) {
		if (en_epoch)
			printf("%4d.%03d  ",
				(int) (ts.tv_sec % 10000),
				(int) (ts.tv_nsec/1000000)); // .%03ld
//				(int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
		else
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "%02d:%02d:%02d.%03d  ",
				mt.tm_hour, mt.tm_min, mt.tm_sec,
			  (int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
	}

	if (!mqttStr && printPayload) {
		tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "Payload: %02X %02X %02X %02X %02X",
			payload[0], payload[1], payload[2], payload[3], payload[4]);
	}

	if (!mqttStr) {
		tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "Id: %2d ", nodeId);
	}

#if 1
	if (pl->sensorId == 0 || pl->sensorId > SENID_AHUMD) {
		printf("Bad Sensor Id: %d %s\n", pl->sensorId, tbuf);
		fflush(stdout);
		pkt_avail = false;
		continue;
	}
#else
	if (payload[1] == 0) {
		printf("%s\n", tbuf);
		fflush(stdout);
		pkt_avail = false;
		continue;
	}
#endif

//	sensorId = (payload[1]>>4) & 0xF;
//	seq = (payload[1] >> 2) & 0x3;
	sensorId = pl->sensorId;
	seq = pl->seq;

	if (!mqttStr && printSeq) {
		tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
	}

//	val = payload[1] & 0x03;
	val = pl->hi;
	val <<= 8;
//	val += payload[2];
	val += pl->low;

	val1 = pl->hi;
	val1 = (val1<<8) | pl->mid;
	val1 = (val1<<8) | pl->low;

	switch (sensorId) {
	case SENID_REV:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/rev");
		sprintf(topicVal, "%d.%d", val1/256, val1&0xff);
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Rev: %d.%d", val1/256, val1&0xff);
		}
		nodeStatus[nodeId].nbrMsgs++;
		break;
	case SENID_CTR:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/ctr");
		sprintf(topicVal, "%d", val);
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Ctr: %4d", val);
		}
		nodeStatus[nodeId].nbrMsgs++;
		break;
	case SENID_SW1:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/sw1");
		//sprintf(topicVal, (payload[1] & 0x02) ? "OPEN" : "SHUT");
		sprintf(topicVal, "{\"state\":\"%s\",\"trig\":\"%s\"}", (payload[1] & 0x02) ? "OPEN" : "SHUT", (payload[1] & 0x01) ? "PC" : "WD");
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  SW1: %s", (payload[1] & 0x02) ? "OPEN" : "SHUT");
		}
		nodeStatus[nodeId].nbrMsgs++;
		nodeStatus[nodeId].sw1 = (payload[1] & 0x02) ? 0 : 1;
		break;
	case SENID_SW2:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/sw2");
		sprintf(topicVal, (payload[1] & 0x02) ? "OPEN" : "SHUT");
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  SW2: %s", (payload[1] & 0x02) ? "OPEN" : "SHUT");
		}
		nodeStatus[nodeId].nbrMsgs++;
		break;
	case SENID_VCC:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/vcc");
		sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
		}
		nodeStatus[nodeId].nbrMsgs++;
		nodeStatus[nodeId].vcc = (1.1 * 1024.0)/(float)val;
		break;
	case SENID_TEMP:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/temp");
		sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Temp: %4.2f",1.0 * (float)val - 260.0);
		}
		nodeStatus[nodeId].nbrMsgs++;
		break;
	case SENID_ATEMP:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/atemp");
		float ftemp = ((float)(val1*200)/0x100000) - 50.0;
		ftemp = (ftemp * 9.0/5.0 ) + 32.05;
		if (ftemp < -30.0) ftemp = -30.0;
		if (ftemp > 120.0) ftemp = 120.0;
//		if (ftemp < -30.0 || ftemp > 140.0) goto check;
		sprintf(topicVal, "%4.1f", ftemp);
//		sprintf(topicVal, "%4.2f", (((float)(val1*200))/0x100000) - 50.0);
		if (!mqttStr) {
//			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Temp: %4.2f",((float)(val1*200)/0x100000) - 50.0);
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Temp: %4.1f", ftemp);
		}
		nodeStatus[nodeId].nbrMsgs++;
		nodeStatus[nodeId].atemp = ftemp;
		break;
	case SENID_AHUMD:
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/ahumd");
		float fhumd = ((float)(val1*100))/0x100000;
		if (fhumd < 0.0) fhumd = 0.0;
		if (fhumd > 100.0) fhumd = 100.0;
		fhumd += 0.05;
		sprintf(topicVal, "%4.1f", fhumd);
		if (!mqttStr) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Temp: %4.1f", fhumd);
		}
		nodeStatus[nodeId].nbrMsgs++;
		nodeStatus[nodeId].ahumd = fhumd;
		break;
	default:
		printf("Bad SensorId: %d\n", sensorId);
		fflush(stdout);
		pkt_avail = false;
		continue;
	}

	mosquitto_publish(mosq, NULL, topic, strlen(topicVal), topicVal, 0, 0);

	if (!mqttStr) {
		printf("%s", tbuf);
	} else {
#if 1
		if (printTime) {
			//printf("%d  ", (int)ts.tv_sec);
			if (en_epoch)
				printf("%4d.%03d  ",
					(int) (ts.tv_sec % 10000),
					(int) (ts.tv_nsec/1000000)); // .%03ld
//					(int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
			else
				printf("%02d:%02d:%02d.%03d  ",
					mt.tm_hour, mt.tm_min, mt.tm_sec,
					(int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
		}
#else
		if (printTime) {
			//printf("%d  ", (int)ts.tv_sec);
			printf("%02d:%02d:%02d.%03d  ",
		      (int) ((ts.tv_sec % SECS_IN_DAY) / 3600) - 7,
		      (int) (ts.tv_sec % 3600) / 60,
		      (int) ts.tv_sec % 60,
			  (int) ((ts.tv_nsec/100000)+5)/10); // .%03ld
		}
#endif	
		printf("Id: %2d ", nodeId);
		printf("Seq: %d ", seq);
		printf("%s", topic);
		//printf("%s %s", topic, topicVal);
		if (sensorId == SENID_SW1)
			printf(" %s %s", (payload[1] & 0x02) ? "OPEN" : "SHUT", (payload[1] & 0x01) ? "PC" : "");
		else
			printf(" %s", topicVal);
	}


//	if (sensorId == SENID_SW1)
//		printf(" %s\n", (payload[1] & 0x01) ? "PC" : "");
//	else
		printf("\n");
				
	fflush(stdout);

		//payLen = nrfReadRxPayloadLen();
		//if (payLen != PAYLOAD_LEN) {
			//fprintf(stderr, "PAYLOAD1 LEN: %d\n", payLen);
			//fprintf(stderr, "FIFO_STATUS: %02X\n", nrfRegRead(NRF_FIFO_STATUS));
			//fflush(stderr);
		//}

#if 1
		pkt_avail = ((nrfRegRead(NRF_FIFO_STATUS) & 1) == 0);
#else
		pkt_avail = (nrfRegRead( NRF_STATUS ) & 0x40) == 0x40;
#endif
	}
	return 0;
}

#if !SPI_BIT_BANG
void spiSetup(int speed)
{
	if ((spiFd = wiringPiSPISetup(0, speed)) < 0) {
		fprintf(stderr, "Can't open the SPI bus: %s\n", strerror (errno));
		exit (EXIT_FAILURE);
	}
}
#endif

#if SPI_BIT_BANG
int spiXfer(uint8_t *buf, int cnt)
{
	uint8_t tmpOut, tmpIn;
	int i;

	digitalWrite(nrfCSN, LOW);
	while (cnt--) {
		tmpIn = 0;
		tmpOut = *buf;
		for (i = 0; i < 8; i++) {
			tmpIn<<=1;
			// write MOSI
			if (tmpOut & (1<<(7-i)))
				digitalWrite(MOSI_PIN, HIGH);
			else
				digitalWrite(MOSI_PIN, LOW);
			// write SCLK
			digitalWrite(SCLK_PIN, HIGH);
			// read MISO
			if (digitalRead(MISO_PIN) == HIGH)
				tmpIn |= 1;
			digitalWrite(SCLK_PIN, LOW);
		}
		*buf++ = tmpIn;;
	}
//	digitalWrite(nrfCSN, LOW);
//	digitalWrite(nrfCSN, LOW);
	digitalWrite(nrfCSN, HIGH);
	return 0;
}
#else
int spiXfer(uint8_t *buf, int cnt)
{
	int rv;

    digitalWrite(nrfCSN, LOW);
	rv = wiringPiSPIDataRW(0, buf, cnt);
    digitalWrite(nrfCSN, HIGH);
	if (rv == -1) {
		fprintf(stderr, "Error in spiXfer: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}
#endif

uint8_t nrfAddrRead( uint8_t reg, uint8_t *buf, int len )
{
	uint8_t spiBuf[24];

	spiBuf[0] = reg & 0x1f;
	spiXfer(spiBuf, len+1);
	for (int i = 0; i < len; i++)
		buf[i] = spiBuf[i+1];
	nrfStatus = spiBuf[0];
	return 0;
}


uint8_t nrfFlushRx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe2;
	spiXfer(spiBuf, 1);
	nrfStatus = spiBuf[0];
	return nrfStatus;
}

uint8_t nrfFlushTx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe1;
	spiXfer(spiBuf, 1);
	nrfStatus = spiBuf[0];
	return nrfStatus;
}

uint8_t nrfRegWrite( int reg, int val)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	spiBuf[1] = val;
	spiXfer(spiBuf, 2);
	nrfStatus = spiBuf[0];
	return nrfStatus;
}

#if 1
uint8_t nrfRegsWrite( int reg, uint8_t *buf, int len)
{
	uint8_t spiBuf[20];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	for (int i = 0; i < len; i++)
		spiBuf[i+1] = buf[i];
	spiXfer(spiBuf, len+1);
	nrfStatus = spiBuf[0];
	return nrfStatus;
}
#else
uint8_t nrfRegsWrite( int reg, uint8_t *buf, int len)
{
	uint8_t spiBuf[20];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	for (int i=4,j=1; i >= 0; i--,j++)
		spiBuf[j] = buf[i];
	spiXfer(spiBuf, len+1);
	nrfStatus = spiBuf[0];
	return nrfStatus;
}
#endif

uint8_t nrfRegRead( int reg )
{
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	nrfStatus = spiBuf[0];
	return spiBuf[1];
}

uint8_t nrfReadRxPayloadLen(void)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x60;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	nrfStatus = spiBuf[0];
	return spiBuf[1];
}

uint8_t nrfAvailable( uint8_t *pipe_num )
{
	nrfStatus = nrfRegRead( NRF_STATUS );
	if (nrfStatus & 0x40 ) {
		if ( pipe_num ) {
			*pipe_num = ((nrfStatus>>1) & 0x7);
		}
	}
	return nrfStatus;
}

#if 1
int nrfRead( uint8_t *payload, int len )
{
	uint8_t spiBuf[33];
	int i;
//	uint8_t rxPayloadLen;
	uint8_t pipe = 0;
	uint8_t fifoStatus;
	int rv = 0;

	fifoStatus = nrfRegRead( NRF_FIFO_STATUS );
	if (fifoStatus & 1) return -1;
	printf("%02X ", fifoStatus);
//	rxPayloadLen = nrfReadRxPayloadLen();
//	printf("rxPLen: %d ", rxPayloadLen);
	nrfAvailable(&pipe);
	printf("pipe: %d ", pipe); fflush(stdout);

	spiBuf[0] = 0x61;
	for (i = 1; i < len+1; i++)
		spiBuf[i] = 0;
	spiXfer(spiBuf, len+1);
	fifoStatus = nrfRegRead( NRF_FIFO_STATUS );
	printf("%02X ", fifoStatus);

//nrfFlushRx();
	nrfStatus = spiBuf[0];
	if (payload)
		for (i = 1; i < len+1; i++)
			payload[i-1] = spiBuf[i];

	nrfRegWrite( NRF_STATUS, 0x40 );
	nrfStatus = nrfRegRead( NRF_STATUS );

	if (nrfStatus & 0x40) {
		printf("More Data: %02X ", spiBuf[1]); fflush(stdout);
	}
	return rv;
}
#else
int nrfRead( uint8_t *payload, int len )
{
	uint8_t spiBuf[33];
	int i;

	if (len > 32)
		return -1;
	if (len < 1)
		return -1;

	spiBuf[0] = 0x61;
	for (i = 1; i < len+1; i++)
		spiBuf[i] = 0;
	spiXfer(spiBuf, len+1);
	if (payload)
		for (i = 1; i < len+1; i++)
			payload[i-1] = spiBuf[i];
	
	nrfRegWrite( NRF_STATUS, 0x40 );

	return 0;
}
#endif

void nrfPrintDetails(void)
{
	uint8_t		buf[6];

	printf("================ SPI Configuration ================\n" );
	printf("CSN Pin  \t = Custom GPIO%d\n", nrfCSN  );
	printf("CE Pin  \t = Custom GPIO%d\n", nrfCE );
	printf("Clock Speed\t = " );
	printf("1 Mhz");
	printf("\n================ NRF Configuration ================\n");
 

	printf("STATUS: %02X\n", nrfRegRead( NRF_STATUS ));
	nrfAddrRead( NRF_RX_ADDR_P0, buf, 5 );
	printf("RX_ADDR_P0: %02X%02X%02X%02X%02X\n", buf[4], buf[3], buf[2], buf[1], buf[0]);
//	printf("RX_ADDR_P0: %02X\n", nrfRegRead( NRF_RX_ADDR_P0 ));
	nrfAddrRead( NRF_RX_ADDR_P1, buf, 5 );
	printf("RX_ADDR_P1: %02X%02X%02X%02X%02X\n", buf[4], buf[3], buf[2], buf[1], buf[0]);
//	printf("RX_ADDR_P1: %02X\n", nrfRegRead( NRF_RX_ADDR_P1 ));
	printf("RX_ADDR_P2: %02X\n", nrfRegRead( NRF_RX_ADDR_P2 ));
	printf("RX_ADDR_P3: %02X\n", nrfRegRead( NRF_RX_ADDR_P3 ));
	printf("RX_ADDR_P4: %02X\n", nrfRegRead( NRF_RX_ADDR_P4 ));
	printf("RX_ADDR_P5: %02X\n", nrfRegRead( NRF_RX_ADDR_P5 ));
//	printf("TX_ADDR: %02X\n", nrfRegRead( NRF_TX_ADDR ));
	nrfAddrRead( NRF_TX_ADDR, buf, 5 );
	printf("TX_ADDR: %02X%02X%02X%02X%02X\n", buf[4], buf[3], buf[2], buf[1], buf[0]);

//  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
	printf("EN_AA: %02X\n", nrfRegRead( NRF_EN_AA ));
	printf("EN_RXADDR: %02X\n", nrfRegRead( NRF_EN_RXADDR ));
	printf("RF_CH: %02X\n", nrfRegRead( NRF_RF_CH ));
	printf("RF_SETUP: %02X\n", nrfRegRead( NRF_RF_SETUP ));
	printf("RX_PW_P0: %02X\n", nrfRegRead( NRF_RX_PW_P0 ));
	printf("RX_PW_P1: %02X\n", nrfRegRead( NRF_RX_PW_P1 ));
	printf("RX_PW_P2: %02X\n", nrfRegRead( NRF_RX_PW_P2 ));
	printf("RX_PW_P3: %02X\n", nrfRegRead( NRF_RX_PW_P3 ));
	printf("RX_PW_P4: %02X\n", nrfRegRead( NRF_RX_PW_P4 ));
	printf("RX_PW_P5: %02X\n", nrfRegRead( NRF_RX_PW_P5 ));
	printf("CONFIG: %02X\n", nrfRegRead( NRF_CONFIG ));
	printf("CD: %02X\n", nrfRegRead( NRF_CD ));
	printf("SETUP_AW: %02X\n", nrfRegRead( NRF_SETUP_AW ));
	printf("SETUP_RETR: %02X\n", nrfRegRead( NRF_SETUP_RETR ));
	printf("DYNPD: %02X\n", nrfRegRead( NRF_DYNPD ));
	printf("FEATURE: %02X\n", nrfRegRead( NRF_FEATURE ));

#if 1
	if (speed == speed_1M)
		printf("Data Rate\t = %s\n", "1Mbps" );
	else if (speed == speed_250K)
		printf("Data Rate\t = %s\n", "250Kbps" );
	else
		printf("Data Rate\t = %s\n", "2Mbps" );

	printf("Model\t\t = %s\n", "nRF24L01+"  );
	printf("CRC Length\t = %s\n", "8 bits");
	printf("PA Power\t = %s\n", "PA_MAX" );
#endif
	fflush(stdout);
}


#define PORT 8080
PI_THREAD (http_server)
{
    int server_fd, new_socket;
//   	long valread;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("In socket");
        exit(EXIT_FAILURE);
    }
    

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
    
    memset(address.sin_zero, '\0', sizeof address.sin_zero);
    
    
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("In bind");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 10) < 0) {
        perror("In listen");
        exit(EXIT_FAILURE);
    }

    // Only this line has been changed. Everything is same.
    //char *hello = "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: ";
    
    while(1) {
        char tbuffer[5000] = {0};
        char buffer[30000] = {0};
		char tcontent[80];
		char vcc[10], atemp[10], ahumd[10];

//        printf("\n+++++++ Waiting for new connection ++++++++\n\n");
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
            perror("In accept");
            exit(EXIT_FAILURE);
        }
        
        /*valread = */ read( new_socket , buffer, 30000);

		sprintf(tbuffer, "Node  #Msgs    sw1     Vcc    aTemp   aHumd\n");
		for (int i = 1; i < maxNodes; i++) {
			if (nodeStatus[i].nbrMsgs) {
				strcpy(vcc, "      ");
				if (nodeStatus[i].vcc >= 0.0)
					sprintf(vcc, "%6.2f", nodeStatus[i].vcc);
				strcpy(atemp, "      ");
				if (nodeStatus[i].atemp >= -40.0)
					sprintf(atemp, "%6.2f", nodeStatus[i].atemp);
				strcpy(ahumd, "      ");
				if (nodeStatus[i].ahumd >= 0.0)
					sprintf(ahumd, "%6.2f", nodeStatus[i].ahumd);

				sprintf(tcontent, " %2d  %6d    %s  %s  %s  %s\n", i, nodeStatus[i].nbrMsgs,
						swState[nodeStatus[i].sw1], vcc, atemp, ahumd);
				strcat(tbuffer, tcontent);
			}
		}
		sprintf(buffer, "%s%d\n\n", "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: ", strlen(tbuffer));
		strcat(buffer, tbuffer);


//        printf("%s\n",buffer );
        write(new_socket , buffer , strlen(buffer));
		//fflush(new_socket);
//        printf("------------------Hello message sent-------------------");
        close(new_socket);
    }
    return 0;
}

