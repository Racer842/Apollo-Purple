/*
 * M5 Core2 WiFi Test Application
 * Tests bidirectional WiFi communication
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(m5_wifi_test, LOG_LEVEL_DBG);

/* WiFi Configuration */
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PSK "YourWiFiPassword"

/* Test Server Configuration */
#define TEST_SERVER_IP "192.168.1.100"  // Your Mac IP
#define TEST_SERVER_PORT 8888
#define LOCAL_SERVER_PORT 9999

/* LED for status indication */
#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED_NODE, gpios, {0});

/* Network status */
static bool wifi_connected = false;
static struct net_mgmt_event_callback mgmt_cb;

/* Socket file descriptors */
static int client_sock = -1;
static int server_sock = -1;

/* Test counters */
static int messages_sent = 0;
static int messages_received = 0;
static int server_connections = 0;

/* WiFi Management Event Handler */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                   uint32_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_WIFI_CONNECT_RESULT:
        LOG_INF("WiFi connected successfully");
        wifi_connected = true;
        if (gpio_is_ready_dt(&led)) {
            gpio_pin_set_dt(&led, 1);
        }
        break;
    case NET_EVENT_WIFI_DISCONNECT_RESULT:
        LOG_INF("WiFi disconnected");
        wifi_connected = false;
        if (gpio_is_ready_dt(&led)) {
            gpio_pin_set_dt(&led, 0);
        }
        break;
    default:
        break;
    }
}

/* Connect to WiFi */
static int wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();
    struct wifi_connect_req_params params = {0};

    if (!iface) {
        LOG_ERR("No default network interface found");
        return -1;
    }

    params.ssid = WIFI_SSID;
    params.ssid_length = strlen(WIFI_SSID);
    params.psk = WIFI_PSK;
    params.psk_length = strlen(WIFI_PSK);
    params.channel = WIFI_CHANNEL_ANY;
    params.security = WIFI_SECURITY_TYPE_PSK;

    LOG_INF("Connecting to WiFi network: %s", WIFI_SSID);
    return net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
}

/* Get local IP address */
static void print_local_ip(void)
{
    struct net_if *iface = net_if_get_default();
    struct net_if_addr *addr;
    char ip_str[NET_IPV4_ADDR_LEN];

    if (!iface) {
        return;
    }

    addr = net_if_ipv4_addr_lookup(&iface->config.ip.ipv4->unicast, NULL);
    if (addr) {
        net_addr_ntop(AF_INET, &addr->address.in_addr, ip_str, sizeof(ip_str));
        LOG_INF("Local IP address: %s", ip_str);
    }
}

/* Create and connect client socket */
static int create_client_socket(void)
{
    struct sockaddr_in server_addr;
    int sock, ret;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create client socket: %d", errno);
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TEST_SERVER_PORT);
    ret = inet_pton(AF_INET, TEST_SERVER_IP, &server_addr.sin_addr);
    if (ret != 1) {
        LOG_ERR("Invalid server IP address");
        close(sock);
        return -1;
    }

    LOG_INF("Connecting to server %s:%d", TEST_SERVER_IP, TEST_SERVER_PORT);
    ret = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        LOG_ERR("Failed to connect to server: %d", errno);
        close(sock);
        return -1;
    }

    LOG_INF("Connected to test server successfully");
    return sock;
}

/* Create server socket */
static int create_server_socket(void)
{
    struct sockaddr_in server_addr;
    int sock, ret;
    int opt = 1;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create server socket: %d", errno);
        return -1;
    }

    /* Set socket options */
    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (ret < 0) {
        LOG_WRN("Failed to set SO_REUSEADDR: %d", errno);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(LOCAL_SERVER_PORT);

    ret = bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        LOG_ERR("Failed to bind server socket: %d", errno);
        close(sock);
        return -1;
    }

    ret = listen(sock, 5);
    if (ret < 0) {
        LOG_ERR("Failed to listen on server socket: %d", errno);
        close(sock);
        return -1;
    }

    LOG_INF("Server listening on port %d", LOCAL_SERVER_PORT);
    return sock;
}

/* Send test message to server */
static int send_test_message(int sock)
{
    char message[256];
    int ret;

    snprintf(message, sizeof(message),
             "{"
             "\"device\":\"M5Core2\","
             "\"test_id\":%d,"
             "\"message\":\"Hello from M5Core2\","
             "\"timestamp\":%u,"
             "\"wifi_status\":\"connected\""
             "}",
             messages_sent + 1, (uint32_t)(k_uptime_get_32() / 1000));

    ret = send(sock, message, strlen(message), 0);
    if (ret < 0) {
        LOG_ERR("Failed to send message: %d", errno);
        return -1;
    }

    messages_sent++;
    LOG_INF("Sent message #%d (%d bytes)", messages_sent, ret);
    return 0;
}

/* Receive and process message */
static int receive_message(int sock)
{
    char buffer[512];
    int ret;

    ret = recv(sock, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
    if (ret > 0) {
        buffer[ret] = '\0';
        messages_received++;
        LOG_INF("Received message #%d (%d bytes): %s", messages_received, ret, buffer);
        return ret;
    } else if (ret == 0) {
        LOG_INF("Connection closed by peer");
        return -1;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_ERR("Receive error: %d", errno);
        return -1;
    }
    
    return 0; /* No data available */
}

/* Client thread - sends messages to external server */
static void client_thread(void)
{
    while (1) {
        if (!wifi_connected) {
            k_sleep(K_MSEC(1000));
            continue;
        }

        /* Create client connection if needed */
        if (client_sock < 0) {
            client_sock = create_client_socket();
            if (client_sock < 0) {
                LOG_WRN("Failed to create client connection, retrying...");
                k_sleep(K_MSEC(5000));
                continue;
            }
        }

        /* Send test message */
        if (send_test_message(client_sock) < 0) {
            LOG_WRN("Failed to send message, reconnecting...");
            close(client_sock);
            client_sock = -1;
            k_sleep(K_MSEC(2000));
            continue;
        }

        /* Try to receive response */
        int ret = receive_message(client_sock);
        if (ret < 0) {
            LOG_WRN("Connection lost, reconnecting...");
            close(client_sock);
            client_sock = -1;
        }

        k_sleep(K_MSEC(10000)); /* Send every 10 seconds */
    }
}

/* Server thread - accepts incoming connections */
static void server_thread(void)
{
    int client_fd;
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    fd_set read_fds;
    struct timeval timeout;

    while (1) {
        if (!wifi_connected) {
            k_sleep(K_MSEC(1000));
            continue;
        }

        /* Create server socket if needed */
        if (server_sock < 0) {
            server_sock = create_server_socket();
            if (server_sock < 0) {
                LOG_ERR("Failed to create server socket, retrying...");
                k_sleep(K_MSEC(5000));
                continue;
            }
        }

        /* Use select to check for incoming connections */
        FD_ZERO(&read_fds);
        FD_SET(server_sock, &read_fds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(server_sock + 1, &read_fds, NULL, NULL, &timeout);
        if (ret > 0 && FD_ISSET(server_sock, &read_fds)) {
            /* Accept incoming connection */
            client_fd = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
            if (client_fd >= 0) {
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
                
                server_connections++;
                LOG_INF("Accepted connection #%d from %s:%d", 
                       server_connections, client_ip, ntohs(client_addr.sin_port));

                /* Handle client in a simple way - echo back a response */
                char response[256];
                snprintf(response, sizeof(response),
                         "{"
                         "\"device\":\"M5Core2_Server\","
                         "\"response\":\"Hello from M5Core2 Server\","
                         "\"connection_id\":%d,"
                         "\"timestamp\":%u"
                         "}",
                         server_connections, (uint32_t)(k_uptime_get_32() / 1000));
                
                send(client_fd, response, strlen(response), 0);
                
                /* Try to receive data from client */
                char recv_buffer[512];
                ret = recv(client_fd, recv_buffer, sizeof(recv_buffer) - 1, 0);
                if (ret > 0) {
                    recv_buffer[ret] = '\0';
                    LOG_INF("Server received from client: %s", recv_buffer);
                }
                
                close(client_fd);
                LOG_INF("Client connection closed");
            }
        } else if (ret < 0) {
            LOG_ERR("Select error: %d", errno);
            k_sleep(K_MSEC(1000));
        }
    }
}

/* Status reporting thread */
static void status_thread(void)
{
    while (1) {
        if (wifi_connected) {
            print_local_ip();
            LOG_INF("=== WiFi Test Status ===");
            LOG_INF("WiFi: %s", wifi_connected ? "Connected" : "Disconnected");
            LOG_INF("Messages sent: %d", messages_sent);
            LOG_INF("Messages received: %d", messages_received);
            LOG_INF("Server connections: %d", server_connections);
            LOG_INF("Uptime: %u seconds", (uint32_t)(k_uptime_get_32() / 1000));
            LOG_INF("=======================");
        }
        
        k_sleep(K_MSEC(30000)); /* Status every 30 seconds */
    }
}

/* Define threads */
K_THREAD_DEFINE(client_tid, 2048, client_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(server_tid, 2048, server_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(status_tid, 1024, status_thread, NULL, NULL, NULL, 8, 0, 0);

int main(void)
{
    int ret;

    LOG_INF("=================================");
    LOG_INF("M5 Core2 WiFi Test Application");
    LOG_INF("=================================");

    /* Initialize LED GPIO if available */
    if (gpio_is_ready_dt(&led)) {
        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure LED GPIO: %d", ret);
        } else {
            LOG_INF("LED GPIO initialized");
        }
    } else {
        LOG_WRN("LED GPIO not available");
    }

    /* Setup WiFi management callback */
    net_mgmt_init_event_callback(&mgmt_cb, wifi_mgmt_event_handler,
                                NET_EVENT_WIFI_CONNECT_RESULT |
                                NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&mgmt_cb);

    /* Connect to WiFi */
    ret = wifi_connect();
    if (ret) {
        LOG_ERR("WiFi connection request failed: %d", ret);
    }

    LOG_INF("Test configuration:");
    LOG_INF("- WiFi SSID: %s", WIFI_SSID);
    LOG_INF("- Test server: %s:%d", TEST_SERVER_IP, TEST_SERVER_PORT);
    LOG_INF("- Local server port: %d", LOCAL_SERVER_PORT);
    LOG_INF("System initialized, starting tests...");

    return 0;
}