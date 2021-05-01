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


#define NRFIRQ			5
#define nrfCSN			10
#define nrfCE			6
#define SPI_BIT_BANG	1
#if SPI_BIT_BANG
#define MOSI_PIN		12
#define MISO_PIN		13
#define SCLK_PIN		14
#endif

#define PAYLOAD_LEN		3

//#define MAX_NODES		20

#define handle_error(msg) \
	do { perror(msg); /*exit(EXIT_FAILURE);*/ } while (0)

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


typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR
} senId_t;


typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;

//unsigned char payload[PAYLOAD_LEN];
int longStr = 0;
int printPayload = 0;
int printSeq = 0;
int en_shockburst = 1;
char *pgmName = NULL;
speed_t speed = speed_2M;
int rf_chan = 2;
int maxNodeRcvd = 0;
int verbose = 0;
int printTime = 0;
int nrfIrq = NRFIRQ;
//static int mainThreadPid;
#if !SPI_BIT_BANG
static int spiFd;
#endif

#if 0
char rmt_host[256] = "odp";
int rmt_port = 9900;
int sockFd;
int connected = FALSE;
int remote = 0;
#endif

struct	mosquitto	*mosq = NULL;
//char	*mosq_topic = "/testtopic";
char	*mosq_host = "omv";
int		mosq_port = 1883;
int		mosq_keepalive = 60;
bool	mosq_clean_session = true;

char *nodeMap[] = {
	"node/0",
	"node/1",
	"node/2",
	"node/3",
	"node/4",
	"door/5",
	"node/6",
	"node/7",
	"door/Hall",
	"node/9",
	"node/10",
	"node/11",
	"node/12",
	"node/13",
	"door/Garage",
	"node/15",
	"node/16",
	"door/GarageS",
	"door/Sliding",
	"door/Back",
	"node/20",
	"window/officeN",
	"door/Front",
	"node/23",
	"window/officeS",
	"window/masterW",
	"window/masterE",
	"door/GarageN",
	"node/28",
	"node/29",
	"node/30",
	"node/31",
	"node/32",
	"node/33",
	"node/34",
	"node/35",
	"node/36",
	"node/37",
	"node/38",
	"node/99"
};
int maxNodes = sizeof(nodeMap)/sizeof(char*);



//************  Forward Declarations
int parse_payload( uint8_t *payload );
void spiSetup( int speed );
int spiXfer( uint8_t *buf, int cnt );
uint8_t nrfRegRead( int reg );
int nrfRegWrite( int reg, int val );
void nrfPrintDetails(void);
int nrfAvailable( uint8_t *pipe_num );
int nrfRead( uint8_t *payload, int len );
int nrfFlushTx( void );
int nrfFlushRx( void );
int nrfAddrRead( uint8_t reg, uint8_t *buf, int len );
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


#if 0
int main(int argc, char *argv[])
{
	int i = -10;
	char *buf = malloc(64);


    // main loop
	for (int i = 0; i < 10; i++) {
		sprintf(buf, "i=%i", i);
		int snd = mosquitto_publish(mosq, NULL, topic, strlen(buf), buf, 0, 0);
		if (snd != 0) 
			printf("mqtt_send error=%i\n", snd);
		usleep(100000);
	}
	return 0;
}
#endif


//
// error - wrapper for perror
//
void error(char *msg) {
	perror(msg);
//	exit(0);
}


void nrfIntrHandler(void)
{
	uint8_t pipeNum __attribute__ ((unused));
	uint8_t payLen __attribute__ ((unused));
	unsigned char payload[PAYLOAD_LEN];

	payLen = nrfReadRxPayloadLen();
	if (payLen != PAYLOAD_LEN) {
//		if (verbose) fprintf(stderr, "PAYLOAD LEN: %d\n", payLen);
		fprintf(stderr, "PAYLOAD LEN: %d\n", payLen);
	}

	nrfRead( payload, payLen );
	parse_payload( payload );
}


void sig_handler( int sig )
{
	if (sig == SIGINT) {
		printf("\nPowering down receiver...\n");
    	digitalWrite(nrfCE, LOW);
//		printf("Closing socket\n");
//		close(sockFd);
		printf("Disconnecting from MQTT broker\n");
		mosquitto_disconnect(mosq);
		mosquitto_lib_cleanup();
		exit(0);
	}
}


int Usage(void)
{
	fprintf(stderr, "Usage: %s [-hvlpsStq] [-P n] [-H s] [-c chan] [-g pin] [-x \"1,3-5\"] [-f \"1,3-5\"]\n", pgmName);
	fprintf(stderr, "  -h	this message\n");
	fprintf(stderr, "  -v	verbose\n");
	fprintf(stderr, "  -W	disable shockBurst mode\n");
	fprintf(stderr, "  -l	print output in long string format\n");
	fprintf(stderr, "  -p	print out payload in hex\n");
	fprintf(stderr, "  -s	set receive RF bit rate to 1M (default is 2M)\n");
	fprintf(stderr, "  -S	set receive RF bit rate to 250K (default is 2M)\n");
	fprintf(stderr, "  -t	print timestamp\n");
	fprintf(stderr, "  -q	print packet sequence nbr\n");
	fprintf(stderr, "  -P n	set MQTT port to 'n'\n");
	fprintf(stderr, "  -H s	set MQTT host to 's'\n");
	fprintf(stderr, "  -g n	use GPIO pin 'n' for IRQ input\n");
	fprintf(stderr, "  -c n	set RF receive channel to 'n'\n");
	fprintf(stderr, "  -x s	exclude nodes: e.g. s = \"1,2,3-5,7\"\n");
	fprintf(stderr, "  -f s	include nodes: e.g. s = \"1,2,3-5,7\"\n");
	return 0;
}

#if 0
int tcpSend(const char *msg)
{
	int n;
	struct sockaddr_in serveraddr;
	struct hostent *server;

	if (!connected) {
		sockFd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockFd < 0) {
			fprintf(stderr, "cannot open socket\n");
			return FALSE;
		}
	
		server = gethostbyname(rmt_host);
		if (server == NULL) {
			fprintf(stderr, "ERROR, no such host as %s\n", rmt_host);
			return FALSE;
		}

		bzero((char*)&serveraddr, sizeof(serveraddr));
		serveraddr.sin_family = AF_INET;
		bcopy((char*)server->h_addr, (char*)&serveraddr.sin_addr.s_addr, server->h_length);
		serveraddr.sin_port = htons(rmt_port);

		if (connect(sockFd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
			fprintf(stderr, "ERROR connecting\n");
			return FALSE;
		}
		connected = TRUE;
	}

	n = write(sockFd, msg, strlen(msg));
	if (n < 0) {
		fprintf(stderr, "ERROR writing to socket\n");
		close(sockFd);
		sockFd = -1;
		connected = FALSE;
		return FALSE;
	}
	return TRUE;
}
#endif

//
// main
//
int main(int argc, char *argv[])
{
	int i __attribute__ ((unused));
	uint8_t val8 __attribute__ ((unused));
	int opt;

	pgmName = basename(argv[0]);

//	printf("maxNodes: %d\n", maxNodes); fflush(stdout);
#if 0
	if (strcmp(pgmName, "lofi_rmt") == 0)
		remote = 1;
	else
		remote = 0;
#endif

	while ((opt = getopt(argc, argv, "hvWlpsStqc:x:f:g:P:H:")) != -1) {
		switch (opt) {
		case 'h':
			Usage();
			exit(0);
			break;
		case 'l':
			longStr = 1;
			break;
		case 'W':
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
			speed = speed_250K;
			break;
		case 't':
			printTime = 1;
			break;
		case 'H':
			mosq_host = optarg;
			//strncpy(mosq_host, optarg, 255);
			//mosq_host[255] = '\0';
			break;
		case 'P':
			mosq_port = atoi(optarg);
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

#if 0
	if (remote) {
		printf("PORT: %d\n", rmt_port);
		printf("HOST: %s\n", rmt_host);
	}
#endif

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
	nrfRegWrite( NRF_CONFIG, 0x38 );

	if (en_shockburst) {
		// set nbr of retries and delay
		//	nrfRegWrite( NRF_SETUP_RETR, 0x5F );
		nrfRegWrite( NRF_SETUP_RETR, 0x77 );
		nrfRegWrite( NRF_EN_AA, 3 ); // enable auto ack
	} else {
		nrfRegWrite( NRF_SETUP_RETR, 0 );
		nrfRegWrite( NRF_EN_AA, 0 );
	}

	// Disable dynamic payload
	nrfRegWrite( NRF_FEATURE, 0);
	nrfRegWrite( NRF_DYNPD, 0);

	// Reset STATUS
	nrfRegWrite( NRF_STATUS, 0x70 );

	nrfRegWrite( NRF_EN_RXADDR, 1 ); //3);
	nrfRegWrite( NRF_RX_PW_P0, PAYLOAD_LEN );
#if 1
	nrfRegWrite( NRF_RX_PW_P1, 0 ); //PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P2, 0 );
	nrfRegWrite( NRF_RX_PW_P3, 0 );
	nrfRegWrite( NRF_RX_PW_P4, 0 );
	nrfRegWrite( NRF_RX_PW_P5, 0 );
#else
	nrfRegWrite( NRF_RX_PW_P1, 8 );
	nrfRegWrite( NRF_RX_PW_P2, 8 );
	nrfRegWrite( NRF_RX_PW_P3, 8 );
	nrfRegWrite( NRF_RX_PW_P4, 8 );
	nrfRegWrite( NRF_RX_PW_P5, 8 );
#endif

	// Set up channel
	nrfRegWrite( NRF_RF_CH, rf_chan );

	switch (speed) {
	case speed_1M:
		nrfRegWrite( NRF_RF_SETUP, 0x06 );
		break;
	case speed_2M:
		nrfRegWrite( NRF_RF_SETUP, 0x0e );
		break;
	case speed_250K:
		nrfRegWrite( NRF_RF_SETUP, 0x26 );
		break;
	default:
		nrfRegWrite( NRF_RF_SETUP, 0x0e );
		break;
	}

	nrfFlushTx();
	nrfFlushRx();

	wiringPiISR(nrfIrq, INT_EDGE_FALLING, &nrfIntrHandler);

	// Power up radio and delay 5ms
	nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x02 );
	delay(5);

	// Enable PRIME RX (PRX)
	nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x01 );

	if (verbose)
		nrfPrintDetails();

    digitalWrite(nrfCE, HIGH);

//	nrfRegWrite( NRF_EN_RXADDR, 3 );

#if 0
	sigemptyset(&mask);
	sigaddset(&mask, SIGQUIT);
	sigaddset(&mask, SIGUSR1);

	// Block signals so that they aren't handled
	// according to their default dispositions
	if (sigprocmask(SIG_BLOCK, &mask, NULL) == -1)
		handle_error("sigprocmask");

	sfd = signalfd(-1, &mask, 0);

	mainThreadPid = pthread_self();

	if (sfd == -1)
		handle_error("signalfd");
#endif

	for (;;) {
		delay(10000);
#if 0
		s = read(sfd, &fdsi, sizeof(struct signalfd_siginfo));
		if (s != sizeof(struct signalfd_siginfo)) {
			handle_error("read");
		}
printf("s = %d\n", s); fflush(stdout);

		if (fdsi.ssi_signo == SIGUSR1) {
			printf("Got SIGUSR1\n");
			fflush(stdout);
		} else if (fdsi.ssi_signo == SIGQUIT) {
			printf("Got SIGQUIT\n"); fflush(stdout);
		} else {
			printf("Read unexpected signal\n"); fflush(stdout);
		}
#endif
	}
#if 0
	for (;;) {

#if 0
		while (nrfAvailable(0)) {
			nrfRead( payload, 8 );
			parse_payload( payload );
		}
#endif
		usleep(50000);
	}
#endif

#if 0
#if 1
	uint8_t nrfConfigReg;
	nrfConfigReg = nrfRegRead(0);
	printf("CONFIG: %02X\n", nrfConfigReg);
#else
	spiBuf[0] = 0;
	spiBuf[1] = 0;

	rv = spiXfer(spiBuf, 2);
	if (rv < 0) {
		printf("spiXfer error\n");
	}
	printf("[0]:%02X  [1]:%02X\n", spiBuf[0], spiBuf[1]);
#endif
#endif

	return 0;


#if 0

	if (sfd == -1)
		handle_error("signalfd");

	for (;;) {
		s = read(sfd, &fdsi, sizeof(struct signalfd_siginfo));
		if (s != sizeof(struct signalfd_siginfo)) {
			handle_error("read");
		}
printf("s = %d\n", s); fflush(stdout);

		if (fdsi.ssi_signo == SIGUSR1) {
			printf("Got SIGUSR1\n");
			fflush(stdout);
		} else if (fdsi.ssi_signo == SIGQUIT) {
			printf("Got SIGQUIT\n"); fflush(stdout);
		} else {
			printf("Read unexpected signal\n"); fflush(stdout);
		}
	}
#endif
  return 0;
}

int showPayload( uint8_t *payload )
{
	printf("Payload: %02X %02X %02X\n", payload[0], payload[1], payload[2]);
	return 0;
}

int parse_payload( uint8_t *payload )
{
	struct timespec ts;
	int i;
	unsigned short val;
	uint8_t	sensorId;
	uint8_t nodeId;
	char tbuf[128];
//	char sbuf[80];
	char topic[128];
	char topicVal[20];
	int topicIdx = 0;
	int		tbufIdx = 0;
	int		seq = 0;

	tbuf[0] = '\0';
//	sbuf[0] = '\0';
	topic[0] = '\0';

	clock_gettime(CLOCK_REALTIME, &ts);
	nodeId = payload[0];

	if (nodeId < 1 || nodeId >= maxNodes) {
		fprintf(stderr, "Bad nodeId: %d\n", nodeId); fflush(stderr);
		return -1;
	}


	if (longStr) {
		if (printTime) {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "%d Id: %2d", (int)ts.tv_sec, nodeId);
		} else {
			tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "Id: %2d", nodeId);
		}
		//topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/lofi/node%d", nodeId);
		topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "lofi/%s", nodeMap[nodeId]);
	}

	if (printPayload) {
		tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, " Payload: %02X %02X %02X",
			payload[0], payload[1], payload[2]);
		printf("%s\n", tbuf);
		fflush(stdout);
	}

	for (i = 1; i < PAYLOAD_LEN; ) {

		if (payload[i] == 0) break;

		sensorId = (payload[i]>>4) & 0xF;

		switch (sensorId) {
		case SENID_CTR:
			seq = (payload[i] >> 2) & 0x3;
			if (longStr && printSeq) {
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
			}
			val = payload[i++] & 0x03;
			val <<= 8;
			val += payload[i++];
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Ctr: %4d", val);
				topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/ctr");
				sprintf(topicVal, "%d", val);
			} else {
				printf("%d NodeId: %2d  Ctr: %4d\n", (unsigned int)ts.tv_sec, nodeId, val);
			}
			break;
		case SENID_SW1:
			seq = (payload[i] >> 2) & 0x3;
			if (longStr) {
				if (printSeq) {
					tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
				}
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  SW1: %s", (payload[i] & 0x02) ? "OPEN" : "SHUT");
				topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/sw1");
				sprintf(topicVal, (payload[i] & 0x02) ? "OPEN" : "SHUT");
			} else {
				printf("%d NodeId: %2d  SW1: %s", (unsigned int)ts.tv_sec, nodeId, (payload[i] & 0x02) ? " OPEN\n" : " SHUT\n");
			}
			i++;
			break;
		case SENID_SW2:
			seq = (payload[i] >> 2) & 0x3;
			if (longStr) {
				if (printSeq) {
					tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
				}
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  SW2: %s", (payload[i] & 0x02) ? "OPEN" : "SHUT");
				topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/sw2");
				sprintf(topicVal, (payload[i] & 0x02) ? "OPEN" : "SHUT");
			} else {
				printf("%d NodeId: %2d  SW2: %s", (unsigned int)ts.tv_sec, nodeId, (payload[i] & 0x02) ? " OPEN\n" : " SHUT\n");
			}
			i++;
			break;
		case SENID_VCC:
			seq = (payload[i] >> 2) & 0x3;
			val = payload[i++] & 0x03;
			val <<= 8;
			val += payload[i++];
			if (longStr) {
				if (printSeq) {
					tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
				}
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
				topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/vcc");
				sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
			} else {
				printf("%d NodeId: %2d  Vcc: %4.2f\n", (unsigned int)ts.tv_sec, nodeId, (1.1 * 1024.0)/(float)val);
			}
			break;
		case SENID_TEMP:
			seq = (payload[i] >> 2) & 0x3;
			val = payload[i++] & 0x03;
			val <<= 8;
			val += payload[i++];
			if (longStr) {
				if (printSeq) {
					tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Seq: %1d", seq);
				}
				tbufIdx += snprintf(&tbuf[tbufIdx], 127-tbufIdx, "  Temp: %4.2f",1.0 * (float)val - 260.0);
				topicIdx += snprintf(&topic[topicIdx], 127-topicIdx, "/temp");
				sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
			} else {
				printf("%d NodeId: %2d  Vcc: %4.2f\n", (unsigned int)ts.tv_sec, nodeId, 1.0 * (float)val - 260.0);
			}
			break;
		default:
			fprintf(stderr, "Bad SensorId: %d\n", sensorId);
			return -1;
			break;
		}
	}

	if (longStr) {
		printf("%s %s\n", topic, topicVal);
		mosquitto_publish(mosq, NULL, topic, strlen(topicVal), topicVal, 0, 0);
#if 0
		printf("%s", tbuf);
		strcat(tbuf, "\n");
		if (remote)
			tcpSend(tbuf);
		if (sbuf[0] != '\0') {
			printf("%s", sbuf);
		}
		printf("\n");
#endif
	} else
		printf("\n");
				
	fflush(stdout);
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

int nrfAddrRead( uint8_t reg, uint8_t *buf, int len )
{
	if (buf && len > 1) {
		buf[0] = reg & 0x1f;
		spiXfer(buf, len+1);
		return buf[1];
	}
	return -1;
}


int nrfFlushRx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe2;
	return spiXfer(spiBuf, 1);
}

int nrfFlushTx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe1;
	return spiXfer(spiBuf, 1);
}

int nrfRegWrite( int reg, int val)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	spiBuf[1] = val;
	return spiXfer(spiBuf, 2);
}

uint8_t nrfRegRead( int reg )
{
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

uint8_t nrfReadRxPayloadLen(void)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x60;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

int nrfAvailable( uint8_t *pipe_num )
{
	uint8_t status;

	status = nrfRegRead( NRF_STATUS );
	if (status & 0x40 ) {
		if ( pipe_num ) {
			*pipe_num = ((status>>1) & 0x7);
		}
		return 1;
	}
	return 0;
}

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
	printf("RX_ADDR_P0: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P0: %02X\n", nrfRegRead( NRF_RX_ADDR_P0 ));
	nrfAddrRead( NRF_RX_ADDR_P1, buf, 5 );
	printf("RX_ADDR_P1: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P1: %02X\n", nrfRegRead( NRF_RX_ADDR_P1 ));
	printf("RX_ADDR_P2: %02X\n", nrfRegRead( NRF_RX_ADDR_P2 ));
	printf("RX_ADDR_P3: %02X\n", nrfRegRead( NRF_RX_ADDR_P3 ));
	printf("RX_ADDR_P4: %02X\n", nrfRegRead( NRF_RX_ADDR_P4 ));
	printf("RX_ADDR_P5: %02X\n", nrfRegRead( NRF_RX_ADDR_P5 ));
//	printf("TX_ADDR: %02X\n", nrfRegRead( NRF_TX_ADDR ));
	nrfAddrRead( NRF_TX_ADDR, buf, 5 );
	printf("TX_ADDR: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);

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
