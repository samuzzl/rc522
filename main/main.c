#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "HCF_RC22.h" // Sua biblioteca MFRC522

#define RFID_SPI_HOST SPI3_HOST // Ou SPI2_HOST

static const char *TAG = "RFID_DIAG";

spi_device_handle_t spi_handle_rfid;

void inicializar_spi_rfid_diag() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, // Definido em HCF_RC22.h
        .mosi_io_num = PIN_NUM_MOSI, // Definido em HCF_RC22.h
        .sclk_io_num = PIN_NUM_CLK,  // Definido em HCF_RC22.h
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz para estabilidade
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,       // Definido em HCF_RC22.h
        .queue_size = 4
    };

    esp_err_t ret;
    ret = spi_bus_initialize(RFID_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o barramento SPI: %s", esp_err_to_name(ret));
        return;
    }

    ret = spi_bus_add_device(RFID_SPI_HOST, &devcfg, &spi_handle_rfid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo MFRC522 ao barramento SPI: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI inicializado com sucesso.");
}

void tarefa_diagnostico_rfid(void *pvParameters) {
    Uid uidCartao;
    esp_err_t versao_status;

    // Inicializa o MFRC522 (PCD_ObterVersao está dentro de PCD_Inicializar)
    // PCD_Inicializar em sua lib não retorna status, mas imprime.
    // Vamos chamar PCD_ObterVersao separadamente para verificar se o chip está respondendo.
    versao_status = PCD_ObterVersao(spi_handle_rfid);
    if (versao_status != ESP_OK) {
        ESP_LOGE(TAG, "MFRC522 nao detectado ou versao incorreta. Abortando tarefa.");
        vTaskDelete(NULL); // Termina a tarefa
        return;
    }
    PCD_Inicializar(spi_handle_rfid); // Continua com o resto da inicialização
    ESP_LOGI(TAG, "MFRC522 pronto. Aproxime um cartao...");

    while (1) {
        ESP_LOGD(TAG, "Verificando novo cartao...");
        if (PICC_NovoCartaoPresente(spi_handle_rfid)) {
            ESP_LOGI(TAG, "-----------------------------------------");
            ESP_LOGI(TAG, "PICC_NovoCartaoPresente() retornou TRUE (Cartao detectado no campo).");

            ESP_LOGI(TAG, "Tentando ler o UID com PICC_LerSerialCartao()...");
            uint8_t status_leitura_uid = PICC_LerSerialCartao(spi_handle_rfid, &uidCartao);

            if (status_leitura_uid == STATUS_OK) {
                ESP_LOGI(TAG, "PICC_LerSerialCartao() retornou STATUS_OK.");
                // Imprime UID
                char uid_str[21] = {0}; // Max UID 10 bytes * 2 chars/byte + null
                for (uint8_t i = 0; i < uidCartao.size; i++) {
                    sprintf(&uid_str[i * 2], "%02X", uidCartao.uidByte[i]);
                }
                ESP_LOGI(TAG, "UID: %s (Tamanho: %d bytes)", uid_str, uidCartao.size);

                // Imprime tipo do cartão
                ESP_LOGI(TAG, "SAK: 0x%02X", uidCartao.sak);
                printf("Tipo do Cartao: ");
                PICC_ObterNomeTipo(PICC_ObterTipo(uidCartao.sak));
                printf("\n");

                // Comentado para simplificar o diagnóstico do timeout inicial
                /*
                if (PICC_ObterTipo(uidCartao.sak) == PICC_TYPE_MIFARE_UL) {
                    ESP_LOGI(TAG, "Detectado MIFARE Ultralight. Tentando ler paginas 0-3...");
                    uint8_t bufferLeituraUL[18];
                    uint8_t tamanhoBufferUL = sizeof(bufferLeituraUL);
                    uint8_t status_ul = MIFARE_Ler(spi_handle_rfid, 0, bufferLeituraUL, &tamanhoBufferUL);

                    if (status_ul == STATUS_OK && tamanhoBufferUL >= 16) {
                        printf("Pagina  Hex: 0  1  2  3\n");
                        for (int i = 0; i < 4; i++) {
                            printf("Pag %-2d: ", i);
                            for (int j = 0; j < 4; j++) {
                                printf("%02X ", bufferLeituraUL[i * 4 + j]);
                            }
                            printf("\n");
                        }
                    } else {
                        printf("Falha ao ler paginas do Ultralight. Status: ");
                        ObterNomeCodigoStatus(status_ul);
                        printf(" (Recebidos: %d bytes)\n", tamanhoBufferUL);
                    }
                }
                */

                ESP_LOGI(TAG, "Enviando comando HALT (PICC_PararA)...");
                uint8_t halt_status = PICC_PararA(spi_handle_rfid);
                ESP_LOGI(TAG, "PICC_PararA() status: 0x%02X", halt_status);
                ObterNomeCodigoStatus(halt_status); printf("\n");

                PCD_PararCripto1(spi_handle_rfid);
            } else {
                ESP_LOGW(TAG, "PICC_LerSerialCartao() FALHOU. Status: 0x%02X", status_leitura_uid);
                printf("Nome do status da falha: ");
                ObterNomeCodigoStatus(status_leitura_uid);
                printf("\n");
            }

            ESP_LOGI(TAG, "-----------------------------------------");
            ESP_LOGI(TAG, "Aguardando 2 segundos para remover o cartao...");
            vTaskDelay(pdMS_TO_TICKS(2000)); // Aumentar o delay para dar tempo de remover
        }

        vTaskDelay(pdMS_TO_TICKS(250)); // Verifica presença periodicamente
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Diagnostico RFID...");
    inicializar_spi_rfid_diag();

    // Verifica se o handle SPI foi inicializado antes de criar a tarefa
    if (spi_handle_rfid == NULL) {
        ESP_LOGE(TAG, "Falha critica na inicializacao do SPI. Handle eh NULL. Reinicie o dispositivo.");
        return; // Não continua se o SPI não inicializou
    }

    xTaskCreate(tarefa_diagnostico_rfid, "tarefa_diag_rfid", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tarefa de diagnostico RFID iniciada.");
}