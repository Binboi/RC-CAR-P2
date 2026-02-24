#include "usb_capture.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "usb/usb_host.h"

/* ─── tunables ─────────────────────────────────────────────────── */
#define USB_CAPTURE_DAEMON_STACK   4096
#define USB_CAPTURE_CLIENT_STACK   4096
#define USB_CAPTURE_TASK_PRIORITY  5

/* ─── internal event bits ───────────────────────────────────────── */
#define EVT_DAEMON_STARTED   BIT0    /* daemon task is running       */
#define EVT_CLIENT_STARTED   BIT1    /* client task is running       */
#define EVT_DEVICE_CONNECTED BIT2    /* a USB device was enumerated  */
#define EVT_STOP_REQUEST     BIT3    /* tear-down requested          */

static const char *TAG = "Captured data packet:";

/* ─── globals shared between the two tasks ─────────────────────── */
static EventGroupHandle_t s_evt_group = NULL;
static usb_host_client_handle_t s_client = NULL;
static usb_device_handle_t s_dev_hdl  = NULL;

/* transfer object — allocated once, reused for every IN poll */
static usb_transfer_t *s_transfer = NULL;

/* EP info filled after device open */
static uint8_t  moza_channel   = 0; //channel of the moza data
static uint16_t max_packet_size = 69;  //max packet size of the moza data

/* ─── forward declarations ──────────────────────────────────────── */
static void usb_daemon_task(void *arg);
static void usb_client_task(void *arg);
static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg);
static void transfer_cb(usb_transfer_t *transfer);
static bool find_interrupt_in_ep(usb_device_handle_t dev,
                                 uint8_t *ep_addr_out,
                                 uint16_t *ep_mps_out);

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t usb_capture_start(void)
{
    s_evt_group = xEventGroupCreate();
    if (!s_evt_group) return ESP_ERR_NO_MEM;

    /* Install the USB host library */
    usb_host_config_t host_cfg = {
        .skip_phy_setup       = false,
        .intr_flags           = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create daemon task (must exist before client task) */
    xTaskCreate(usb_daemon_task, "usb_daemon", USB_CAPTURE_DAEMON_STACK,
                NULL, USB_CAPTURE_TASK_PRIORITY, NULL);

    /* Wait until daemon is ready, then start client task */
    xEventGroupWaitBits(s_evt_group, EVT_DAEMON_STARTED,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    xTaskCreate(usb_client_task, "usb_client", USB_CAPTURE_CLIENT_STACK,
                NULL, USB_CAPTURE_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "USB capture started — plug in your MOZA device");
    return ESP_OK;
}

void usb_capture_stop(void)
{
    if (s_evt_group)
        xEventGroupSetBits(s_evt_group, EVT_STOP_REQUEST);
    /* tasks detect the bit and self-delete; give them time */
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* ═══════════════════════════════════════════════════════════════════
 * DAEMON TASK — handles USB host library events
 * ═══════════════════════════════════════════════════════════════════ */

static void usb_daemon_task(void *arg)
{
    ESP_LOGI(TAG, "daemon task started");
    xEventGroupSetBits(s_evt_group, EVT_DAEMON_STARTED);

    bool running = true;
    while (running) {
        /* Check for stop request */
        EventBits_t bits = xEventGroupGetBits(s_evt_group);
        if (bits & EVT_STOP_REQUEST) {
            usb_host_device_free_all();
            running = false;
            continue;
        }

        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(100), &event_flags);
        if (err == ESP_ERR_TIMEOUT) continue;

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGW(TAG, "No clients registered");
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All devices freed — uninstalling host lib");
            running = false;
        }
    }

    usb_host_uninstall();
    ESP_LOGI(TAG, "daemon task exiting");
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * CLIENT TASK — registers, waits for device, reads HID reports
 * ═══════════════════════════════════════════════════════════════════ */

static void usb_client_task(void *arg)
{
    ESP_LOGI(TAG, "client task started — waiting for device…");

    /* Register this task as a USB host client */
    usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg          = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &s_client));
    xEventGroupSetBits(s_evt_group, EVT_CLIENT_STARTED);

    while (true) {
        /* Check for stop request */
        EventBits_t bits = xEventGroupGetBits(s_evt_group);
        if (bits & EVT_STOP_REQUEST) break;

        /* Let the client library process its callbacks */
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(50));

        /* Once a device is connected and EP is found, keep submitting transfers */
        if ((bits & EVT_DEVICE_CONNECTED) && s_transfer && s_dev_hdl) {
            /* transfer_cb will re-submit automatically; only submit once here */
        }
    }

    /* Cleanup */
    if (s_transfer) {
        usb_host_transfer_free(s_transfer);
        s_transfer = NULL;
    }
    if (s_dev_hdl) {
        usb_host_device_close(s_client, s_dev_hdl);
        s_dev_hdl = NULL;
    }
    usb_host_client_deregister(s_client);
    s_client = NULL;

    ESP_LOGI(TAG, "client task exiting");
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * CLIENT EVENT CALLBACK
 * Called when a USB device connects or disconnects
 * ═══════════════════════════════════════════════════════════════════ */

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        uint8_t dev_addr = msg->new_dev.address;
        ESP_LOGI(TAG, "Device connected, address: %d", dev_addr);

        /* Open the device */
        esp_err_t err = usb_host_device_open(s_client, dev_addr, &s_dev_hdl);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "usb_host_device_open failed: %s", esp_err_to_name(err));
            return;
        }

        /* Print device descriptor info */
        const usb_device_desc_t *dev_desc;
        usb_host_get_device_descriptor(s_dev_hdl, &dev_desc);
        ESP_LOGI(TAG, "VID: 0x%04X  PID: 0x%04X", dev_desc->idVendor, dev_desc->idProduct);

        /* Find the interrupt IN endpoint (HID reports come from here) */
        if (!find_interrupt_in_ep(s_dev_hdl, &moza_channel, &max_packet_size)) {
            ESP_LOGW(TAG, "No interrupt IN endpoint found — trying EP 0x81 as fallback");
            moza_channel = 0x81;
            max_packet_size  = 64;
        }
        ESP_LOGI(TAG, "Using EP 0x%02X  MPS=%d", moza_channel, max_packet_size);

        /* Claim interface 0 so we can read from it */
        usb_host_interface_claim(s_client, s_dev_hdl, 0, 0);

        /* Allocate a transfer object and submit the first read */
        usb_host_transfer_alloc(max_packet_size, 0, &s_transfer);
        s_transfer->device_handle  = s_dev_hdl;
        s_transfer->bEndpointAddress = moza_channel;
        s_transfer->num_bytes      = max_packet_size;
        s_transfer->callback       = transfer_cb;
        s_transfer->context        = NULL;
        s_transfer->timeout_ms     = 1000;

        usb_host_transfer_submit(s_transfer);
        xEventGroupSetBits(s_evt_group, EVT_DEVICE_CONNECTED);

    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGW(TAG, "Device disconnected");
        xEventGroupClearBits(s_evt_group, EVT_DEVICE_CONNECTED);

        if (s_transfer) {
            usb_host_transfer_free(s_transfer);
            s_transfer = NULL;
        }
        if (s_dev_hdl) {
            usb_host_interface_release(s_client, s_dev_hdl, 0);
            usb_host_device_close(s_client, s_dev_hdl);
            s_dev_hdl = NULL;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * TRANSFER CALLBACK
 * Called every time a HID report arrives from the device
 * ═══════════════════════════════════════════════════════════════════ */

static void transfer_cb(usb_transfer_t *transfer)
{
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        /* Print raw bytes in hex */
        printf("[MOZA] %d bytes: ", transfer->actual_num_bytes);
        for (int i = 0; i < transfer->actual_num_bytes; i++) {
            printf("%02X ", transfer->data_buffer[i]);
        }
        printf("\n");

    } else {
        ESP_LOGW(TAG, "Transfer status %d - resubmitting",  transfer->status);
    }

    /* Re-submit for the next report (continuous stream) */
    if (s_dev_hdl) {
        usb_host_transfer_submit(transfer);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * HELPER — walk config descriptor to find interrupt IN endpoint
 * ═══════════════════════════════════════════════════════════════════ */

static bool find_interrupt_in_ep(usb_device_handle_t dev,
                                 uint8_t *ep_addr_out,
                                 uint16_t *ep_mps_out)
{
    const usb_config_desc_t *cfg_desc;
    usb_host_get_active_config_descriptor(dev, &cfg_desc);
    if (!cfg_desc) return false;

    int offset = 0;
    int total  = cfg_desc->wTotalLength;
    const uint8_t *raw = (const uint8_t *)cfg_desc;

    while (offset < total) {
        uint8_t len  = raw[offset];
        uint8_t type = raw[offset + 1];

        if (len == 0) break;

        if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)(raw + offset);
            /* Direction IN (bit7=1) and transfer type interrupt (bits 1:0 = 3) */
            if ((ep->bEndpointAddress & 0x80) &&
                (ep->bmAttributes & 0x03) == USB_BM_ATTRIBUTES_XFER_INT) {
                *ep_addr_out = ep->bEndpointAddress;
                *ep_mps_out  = ep->wMaxPacketSize;
                return true;
            }
        }
        offset += len;
    }
    return false;
}