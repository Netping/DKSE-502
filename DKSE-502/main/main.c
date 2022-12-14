/* Simple HTTP Server Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include "includes_base.h"
#include "http_var.h"
#include "update.h"
#include "smtp.h"
#include "snmp.h"
#include "LOGS.h"
#define TAG_boot "main start"
#include "app.h"
#include "../main/termo/app_owb.h"
#include "private_mib.h"
#include "config_pj.h"
#include "connect.h"
#include "sdkconfig.h"
#include "../components/snmp_mib2.h"
#include "../components/snmp_threadsync.h"
#include "../components/snmp_netconn.h"
#include "..\components\snmp_core.h"
#include "../components/snmp_traps.h"
#define DEVICE_USE_STATIC_IP	true
#define DEVICE_IP          "192.168.1.159"
#define DEVICE_GW          "192.168.1.1"
#define DEVICE_NETMASK     "255.255.255.0"

extern tcpip_adapter_ip_info_t ipInfo;
extern esp_netif_dns_info_t dns_info;

char SNMP_COMMUNITY[32];
char SNMP_COMMUNITY_WRITE[32];

u8_t *SNMP_SERVER_IP = (u8_t*) "192.168.1.186";

tcpip_adapter_ip_info_t ipInfo;
esp_netif_dns_info_t dns_info;

static const char *TAG_SNMP = "Snmp_agent";
/* ----- TODO: Global variables for SNMP Trap vvv
 * Define your own vars SNMP_SYSDESCR for System Description, SNMP_SYSCONTACT
 * for your contact mail, SNMP_SYSNAME for your system name, SNMP_SYSLOCATION
 * for your location. Also consider the size of each string in _LEN functions.
 */
static const struct snmp_mib *my_snmp_mibs[] = { &mib2, &mib_private,&mib_private5,&new_mib_private,&out_mib_private,
		&mib_np_private, &mib_termo_private };
//1.3.6.1.2.1.1.1.0
const u8_t *SNMP_SYSDESCR = (u8_t*) "Project 58";
const u16_t SNMP_SYSDESCR_LEN = sizeof("Project 58");
//1.3.6.1.2.1.1.4.0
u8_t *SNMP_SYSCONTACT = (u8_t*) "yourmail@contact.com";
u16_t SNMP_SYSCONTACT_LEN = sizeof("yourmail@contact.com");
//1.3.6.1.2.1.1.5.0
u8_t *SNMP_SYSNAME = (u8_t*) "ESP32_Core_board_V2";
u16_t SNMP_SYSNAME_LEN = sizeof("ESP32_Core_board_V2");
//1.3.6.1.2.1.1.6.0
u8_t *SNMP_SYSLOCATION = (u8_t*) "Your Institute or Company";
u16_t SNMP_SYSLOCATION_LEN = sizeof("Your Institute or Company");
/*
 * ----- TODO: Global variables for SNMP Trap ^^^
 */
char PAGE_BODY[256] = { 0 };

u16_t snmp_buffer = 64;
//TaskHandle_t xHandleNTP = NULL;

/*
 * TODO: Setup SNMP server
 * Define a specific address to send SNMP broadcast package.
 */

//----- ^^^
/* transport information to my_mib.c */
extern const struct snmp_mib gpio_mib;
//extern tcpip_adapter_ip_info_t ipInfo;

void initialize_snmp(void) {

	*SNMP_COMMUNITY = &(FW_data.snmp.V_COMMUNITY);
//    MEMCPY(SNMP_COMMUNITY, FW_data.snmp.V_COMMUNITY, strlen(FW_data.snmp.V_COMMUNITY));
	MEMCPY(SNMP_COMMUNITY_WRITE, FW_data.snmp.V_COMMUNITY_WRITE,
			strlen(FW_data.snmp.V_COMMUNITY_WRITE));
//	SNMP_SYSDESCR = FW_data.sys.
//	SNMP_SYSDESCR_LEN
	SNMP_SYSCONTACT = (u8_t*) FW_data.sys.V_CALL_DATA;
	SNMP_SYSCONTACT_LEN = strlen(FW_data.sys.V_CALL_DATA);
	SNMP_SYSNAME = (u8_t*) FW_data.sys.V_Name_dev;
	SNMP_SYSNAME_LEN = strlen(FW_data.sys.V_Name_dev);
	SNMP_SYSLOCATION = (u8_t*) FW_data.sys.V_GEOM_NAME;
	SNMP_SYSLOCATION_LEN = strlen(FW_data.sys.V_GEOM_NAME);
	lwip_privmib_init();

	snmp_threadsync_init(&snmp_mib2_lwip_locks, snmp_mib2_lwip_synchronizer);
	snmp_mib2_set_syscontact(SNMP_SYSCONTACT, &SNMP_SYSCONTACT_LEN,
			snmp_buffer);
	snmp_mib2_set_syslocation(SNMP_SYSLOCATION, &SNMP_SYSLOCATION_LEN,
			snmp_buffer);
	snmp_set_auth_traps_enabled(ENABLE);
	snmp_mib2_set_sysdescr(SNMP_SYSDESCR, &SNMP_SYSDESCR_LEN);
	snmp_mib2_set_sysname(SNMP_SYSNAME, &SNMP_SYSNAME_LEN, snmp_buffer);

	//    ipaddr_aton((char*)SNMP_SERVER_IP,&gw);
//	IP4_ADDR(&tipaddr, FW_data.snmp.V_IP_SNMP[0], FW_data.snmp.V_IP_SNMP[1],
//			FW_data.snmp.V_IP_SNMP[2], FW_data.snmp.V_IP_SNMP[3]);
//	snmp_trap_dst_ip_set(TRAP_DESTINATION_INDEX, &tipaddr);
//	snmp_trap_dst_enable(TRAP_DESTINATION_INDEX, ENABLE);
	snmp_set_mibs(my_snmp_mibs, LWIP_ARRAYSIZE(my_snmp_mibs));

	snmp_init();
	ESP_LOGI(TAG_SNMP, "initialize_snmp() finished.");

}

void app_main(void) {
	system("chcp 1251 > nul");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	load_struct_flash_data();
	const esp_partition_t *running = esp_ota_get_running_partition();
	ESP_LOGI(TAG_boot, "Current running partition: %s", running->label);
	esp_app_desc_t app_desc;
	esp_err_t ret = esp_ota_get_partition_description(running, &app_desc);
	if (ret == ESP_OK) {
		ESP_LOGI(TAG_boot, "Proj %s, ver %s, date %s, time %s",
				app_desc.project_name, app_desc.version, app_desc.date,
				app_desc.time);
	}
	static httpd_handle_t server = NULL;

	ESP_ERROR_CHECK(example_connect());
	if (((FW_data.net.V_DHCP == 1)||(DEF_DHCP!=0)
			|| ((FW_data.net.V_IP_CONFIG[0] == 0)
					&& (FW_data.net.V_IP_CONFIG[1] == 0)
					&& (FW_data.net.V_IP_CONFIG[2] == 0)
					&& (FW_data.net.V_IP_CONFIG[3] == 0))) == 0) {
		printf("Run DHCP GET \n\r");
		tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_ETH); // Don't run a DHCP client
		//	inet_pton(AF_INET, DEVICE_IP, &ipInfo.ip);
		//	inet_pton(AF_INET, DEVICE_GW, &ipInfo.gw);
		//	inet_pton(AF_INET, DEVICE_NETMASK, &ipInfo.netmask);
		IP4_ADDR(&ipInfo.ip, FW_data.net.V_IP_CONFIG[0],
				FW_data.net.V_IP_CONFIG[1], FW_data.net.V_IP_CONFIG[2],
				FW_data.net.V_IP_CONFIG[3]);
		IP4_ADDR(&ipInfo.gw, FW_data.net.V_IP_GET[0], FW_data.net.V_IP_GET[1],
				FW_data.net.V_IP_GET[2], FW_data.net.V_IP_GET[3]);
		IP4_ADDR(&ipInfo.netmask, FW_data.net.V_IP_MASK[0],
				FW_data.net.V_IP_MASK[1], FW_data.net.V_IP_MASK[2],
				FW_data.net.V_IP_MASK[3]);
		tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_ETH, &ipInfo);
	}
	else
	{
	printf("Run NOT DHCP GET \n\r");
	}
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    	    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    	    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    	#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    	    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    	    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
    	#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET



	/* Start the server for the first time */


    initialise_mdns();
	server = start_webserver();


	initialize_snmp();

	uint8_t mac_addr[8] = { 0 };

	xTaskCreate(&sett_task, "sett_task", 4096, NULL, 10, NULL);

	struct netif *_netif;

	ret = tcpip_adapter_get_netif(TCPIP_ADAPTER_IF_ETH,
				(void**) &_netif);

	gpio_set_direction(4, GPIO_MODE_OUTPUT);
	gpio_set_level(4, 1);

	ESP_LOGI("MAC=", "%02x.%02x.%02x.%02x.%02x.%02x", _netif->hwaddr[0],
			_netif->hwaddr[1], _netif->hwaddr[2], _netif->hwaddr[3],
			_netif->hwaddr[4], _netif->hwaddr[5]);
	while (1) {
		time(&now);
		localtime_r(&now, &timeinfo);
		//    sprintf(PAGE_BODY,"\n\r%d : %d : %d",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);

		gpio_set_level(BLINK_GPIO, 0);

		vTaskDelay(500 / portTICK_PERIOD_MS);

		gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
		FW_data.net.V_IP_CONFIG[0] = (0xff & (ipInfo.ip.addr));
		FW_data.net.V_IP_CONFIG[1] = (0xff & (ipInfo.ip.addr >> 8));
		FW_data.net.V_IP_CONFIG[2] = (0xff & (ipInfo.ip.addr >> 16));
		FW_data.net.V_IP_CONFIG[3] = (0xff & (ipInfo.ip.addr >> 24));

		FW_data.net.V_IP_MASK[0] = (0xff & (ipInfo.netmask.addr));
		FW_data.net.V_IP_MASK[1] = (0xff & (ipInfo.netmask.addr >> 8));
		FW_data.net.V_IP_MASK[2] = (0xff & (ipInfo.netmask.addr >> 16));
		FW_data.net.V_IP_MASK[3] = (0xff & (ipInfo.netmask.addr >> 24));

		FW_data.net.V_IP_GET[0] = (0xff & (ipInfo.gw.addr));
		FW_data.net.V_IP_GET[1] = (0xff & (ipInfo.gw.addr >> 8));
		FW_data.net.V_IP_GET[2] = (0xff & (ipInfo.gw.addr >> 16));
		FW_data.net.V_IP_GET[3] = (0xff & (ipInfo.gw.addr >> 24));

		if (FW_data.net.V_DHCP != 0)
		{
			tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_ETH, 0, &dns_info);

					FW_data.net.V_IP_DNS[0] = (0xff & (dns_info.ip.u_addr.ip4.addr));
					FW_data.net.V_IP_DNS[1] = (0xff & ((dns_info.ip.u_addr.ip4.addr) >> 8));
					FW_data.net.V_IP_DNS[2] = (0xff & ((dns_info.ip.u_addr.ip4.addr) >> 16));
					FW_data.net.V_IP_DNS[3] = (0xff & ((dns_info.ip.u_addr.ip4.addr) >> 24));
		}
		else
		{
			IP4_ADDR(&dns_info.ip.u_addr.ip4, FW_data.net.V_IP_DNS[0],
							FW_data.net.V_IP_DNS[1], FW_data.net.V_IP_DNS[2],
							FW_data.net.V_IP_DNS[3]);
			tcpip_adapter_set_dns_info(TCPIP_ADAPTER_IF_ETH, 0, &dns_info);

		}


		gpio_set_level(BLINK_GPIO, 1);
		tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ipInfo);

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}

}

const __attribute__((used)) __attribute__((section(".rodata_custom_desc"))) char updater_js[] =
		"75hd95kuDbvf8y3k"
				"function fw_is_compatible(oldver){return oldver.startsWith('v58.');}/* **************************** */"
				"48fe99uA6k88eSDa";

