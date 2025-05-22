#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h" // Para esp_err_t, se PCD_ObterVersao usa
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "HCF_RC22.h" // Inclui o header traduzido

// Protótipos de funções "internas" que não estão no .h como públicas
// mas são chamadas por outras funções na biblioteca.
static uint8_t PICC_RequisitarOuAcordarA(spi_device_handle_t spi, uint8_t comando, uint8_t *bufferATQA, uint8_t *tamanhoBuffer);
static uint8_t PCD_ComunicarComPICC(spi_device_handle_t spi, uint8_t comandoPCD, uint8_t esperarIRq, uint8_t *dadosEnviar, uint8_t comprimentoEnviar, uint8_t *dadosRecebidos, uint8_t *comprimentoRecebido, uint8_t *bitsValidos, uint8_t alinhamentoRx, bool verificarCRC);
static uint8_t PCD_TransceptarMIFARE(spi_device_handle_t spi, uint8_t *dadosEnviar, uint8_t comprimentoEnviar, bool aceitarTimeout);


/*
 * Função para obter a versão do MFRC522
 */
esp_err_t PCD_ObterVersao(spi_device_handle_t spi) {
    // VersionReg já está shiftado (ex: 0x37 << 1 = 0x6E)
    // Para ler, o byte SPI é (VersionReg | 0x80)
    uint8_t ver = PCD_LerRegistrador(spi, VersionReg); // PCD_LerRegistrador lida com o bit de leitura

    printf("Versao: 0x%02X\r\n", ver);
    if (ver == 0x92) {
        printf("MFRC522 Versao 2 detectada.\n");
        return ESP_OK;
    } else if (ver == 0x91) {
        printf("MFRC522 Versao 1 detectada.\n");
        return ESP_OK;
    } else if (ver == 0x00 || ver == 0xFF){ // Comum quando não há comunicação
        printf("Falha ao comunicar com MFRC522. Verifique a fiacao e conexoes.\n");
        return ESP_FAIL;
    }
     else {
        printf("O dispositivo conectado eh um MFRC522? Versao desconhecida: 0x%02X. Verifique a fiacao.\n", ver);
        return ESP_FAIL;
    }
}

/*
 * Função para escrever um único byte em um registrador do MFRC522
 */
void PCD_EscreverRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t valor) {
    esp_err_t ret;
    uint8_t tx_dados[2];
    // 'registrador' é o valor do enum PCD_Registrador (ex: CommandReg = 0x01 << 1 = 0x02)
    // Este valor já tem o bit R/W como 0 (implícito pelo shift e ser < 0x80)
    tx_dados[0] = registrador;
    tx_dados[1] = valor;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_dados;

#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 0);
#endif
    ret = spi_device_polling_transmit(spi, &t);
#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 1);
#endif
    assert(ret == ESP_OK);
}

/*
 * Função para escrever múltiplos bytes no MFRC522 (tipicamente no FIFO)
 */
void PCD_EscreverRegistradorMultiplos(spi_device_handle_t spi, uint8_t registrador, uint8_t contagem, uint8_t *valores) {
    if (contagem == 0) return;
    esp_err_t ret;

    uint8_t* buffer_tx_total = (uint8_t*)malloc(contagem + 1);
    if (!buffer_tx_total) {
        printf("E: Falha ao alocar memoria em PCD_EscreverRegistradorMultiplos\n");
        return;
    }
    buffer_tx_total[0] = registrador; // Endereço do registrador (já shiftado, MSB=0)
    memcpy(&buffer_tx_total[1], valores, contagem);

    spi_transaction_t t1;
    memset(&t1, 0, sizeof(t1));
    t1.length = 8 * (contagem + 1);
    t1.tx_buffer = buffer_tx_total;

#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 0);
#endif
    ret = spi_device_polling_transmit(spi, &t1);
#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 1);
#endif
    assert(ret == ESP_OK);

    free(buffer_tx_total);
}

/*
 * Função para ler um único byte de um registrador do MFRC522
 */
uint8_t PCD_LerRegistrador(spi_device_handle_t spi, uint8_t registrador) {
    esp_err_t ret;
    // 'registrador' é o valor do enum PCD_Registrador (ex: VersionReg = 0x37 << 1 = 0x6E)
    // Para ler, o byte de endereço SPI precisa ter o MSB (bit7) setado.
    uint8_t end_reg_leitura = registrador | 0x80;
    uint8_t payload_tx[2] = {end_reg_leitura, 0x00}; // Envia endereço de leitura, depois dummy
    uint8_t payload_rx[2] = {0x00, 0x00};

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = payload_tx;
    t.rx_buffer = payload_rx;

#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 0);
#endif
    ret = spi_device_polling_transmit(spi, &t);
#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 1);
#endif
    assert(ret == ESP_OK);

    return payload_rx[1];
}

/*
 * Função para ler múltiplos bytes do MFRC522 (tipicamente do FIFO)
 */
void PCD_LerRegistradorMultiplos(spi_device_handle_t spi,
                                 uint8_t registrador,
                                 uint8_t contagem,
                                 uint8_t *valores,
                                 uint8_t alinhamentoRx) { // alinhamentoRx é mais para o BitFramingReg
    if (contagem == 0) {
        return;
    }

    esp_err_t ret;
    uint8_t end_reg_leitura = registrador | 0x80;

    uint8_t* buffer_tx = (uint8_t*)malloc(contagem + 1);
    uint8_t* buffer_rx = (uint8_t*)malloc(contagem + 1);

    if (buffer_tx == NULL || buffer_rx == NULL) {
        printf("E: Falha ao alocar memoria em PCD_LerRegistradorMultiplos\n");
        if (buffer_tx) free(buffer_tx);
        if (buffer_rx) free(buffer_rx);
        return;
    }

    buffer_tx[0] = end_reg_leitura;
    for (int i = 0; i < contagem; i++) {
        buffer_tx[i + 1] = 0x00;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (contagem + 1) * 8;
    t.tx_buffer = buffer_tx;
    t.rx_buffer = buffer_rx;

#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 0);
#endif
    ret = spi_device_polling_transmit(spi, &t);
#ifdef MFRC522_MANUAL_CS
    gpio_set_level(PIN_NUM_CS, 1);
#endif
    assert(ret == ESP_OK);

    memcpy(valores, &buffer_rx[1], contagem);

    free(buffer_tx);
    free(buffer_rx);
}

void PCD_LimparMascaraBitsRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t mascara) {
    uint8_t tmp = PCD_LerRegistrador(spi, registrador);
    PCD_EscreverRegistrador(spi, registrador, tmp & (~mascara));
}

void PCD_DefinirMascaraBitsRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t mascara) {
    uint8_t tmp = PCD_LerRegistrador(spi, registrador);
    PCD_EscreverRegistrador(spi, registrador, tmp | mascara);
}

uint8_t PICC_PararA(spi_device_handle_t spi) {
    uint8_t resultado;
    uint8_t buffer[4];

    buffer[0] = PICC_CMD_HLTA; // 0x50
    buffer[1] = 0; // BCC (para este comando HLTA, é 0)
    resultado = PCD_CalcularCRC(spi, buffer, 2, &buffer[2]);
    if (resultado != STATUS_OK) {
        return resultado;
    }
    // Para HLTA, esperamos um TIMEOUT como sucesso (o cartão não deve responder)
    resultado = PCD_TransceptarDados(spi, buffer, 4, NULL, 0, NULL, 0, false);
    if (resultado == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (resultado == STATUS_OK) { // Resposta inesperada
        return STATUS_ERROR;
    }
    return resultado; // Outro erro
}

void PCD_PararCripto1(spi_device_handle_t spi) {
    PCD_LimparMascaraBitsRegistrador(spi, Status2Reg, 0x08); // Limpa MFCrypto1On
}

void PCD_Inicializar(spi_device_handle_t spi) {
    if (PCD_ObterVersao(spi) != ESP_OK) {
        printf("MFRC522 nao detectado ou versao incorreta. Interrompendo inicializacao.\n");
        return;
    }

    // Configura e pulsa o pino de Reset
    gpio_reset_pin(PIN_NUM_RST); // Usar esp_rom_gpio para compatibilidade ou gpio_pad_select_gpio
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2)); // Datasheet recomenda >100ns, 2ms é seguro
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Tempo para o MFRC522 estabilizar

    // Reset por software (opcional, mas bom para garantir estado conhecido)
    PCD_EscreverRegistrador(spi, CommandReg, PCD_SoftReset);
    vTaskDelay(pdMS_TO_TICKS(50)); // Aguarda o reset completar
    // O SoftReset pode limpar alguns registradores, então reconfiguramos
    // Aguarda o bit PowerDown ser limpo
    while (PCD_LerRegistrador(spi, CommandReg) & (1 << 4)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }


    PCD_EscreverRegistrador(spi, TxModeReg, 0x00);
    PCD_EscreverRegistrador(spi, RxModeReg, 0x00);
    PCD_EscreverRegistrador(spi, ModWidthReg, 0x26); // Valor padrão recomendado
    PCD_EscreverRegistrador(spi, TModeReg, 0x80);    // TAuto=1; Timer inicia automaticamente
    PCD_EscreverRegistrador(spi, TPrescalerReg, 0xA9); // Prescaler para ~40kHz (período de 25us)
    PCD_EscreverRegistrador(spi, TReloadRegH, 0x03);  // Timeout de ~25ms (1000 * 25us)
    PCD_EscreverRegistrador(spi, TReloadRegL, 0xE8);
    PCD_EscreverRegistrador(spi, TxASKReg, 0x40);     // Força 100% ASK modulation
    PCD_EscreverRegistrador(spi, ModeReg, 0x3D);      // CRC preset para 0x6363
    // PCD_EscreverRegistrador(spi, RFCfgReg, (0x07<<4)); // Ganho máximo do receptor (48dB) - opcional
    PCD_LigarAntena(spi);
    printf("PCD Inicializado com sucesso.\n");
}

void PCD_LigarAntena(spi_device_handle_t spi) {
    uint8_t valor = PCD_LerRegistrador(spi, TxControlReg);
    if ((valor & 0x03) != 0x03) {
        PCD_EscreverRegistrador(spi, TxControlReg, valor | 0x03);
    }
    // printf("Antena ligada (TxControlReg: 0x%02X).\n", PCD_LerRegistrador(spi, TxControlReg));
}

bool PICC_NovoCartaoPresente(spi_device_handle_t spi) {
    uint8_t bufferATQA[2];
    uint8_t tamanhoBuffer = sizeof(bufferATQA);
    uint8_t resultado = PICC_RequisitarA(spi, bufferATQA, &tamanhoBuffer);
    if(resultado == STATUS_OK)printf("novo cartão detectado.\n");
    return (resultado == STATUS_OK || resultado == STATUS_COLLISION);
}

void PICC_ObterNomeTipo(PICC_Type tipoPicc) {
    switch (tipoPicc) {
        case PICC_TYPE_ISO_14443_4:     printf("PICC compativel com ISO/IEC 14443-4"); break;
        case PICC_TYPE_ISO_18092:       printf("PICC compativel com ISO/IEC 18092 (NFC)"); break;
        case PICC_TYPE_MIFARE_MINI:     printf("MIFARE Mini, 320 bytes"); break;
        case PICC_TYPE_MIFARE_1K:       printf("MIFARE 1KB"); break;
        case PICC_TYPE_MIFARE_4K:       printf("MIFARE 4KB"); break;
        case PICC_TYPE_MIFARE_UL:       printf("MIFARE Ultralight ou Ultralight C"); break;
        case PICC_TYPE_MIFARE_PLUS:     printf("MIFARE Plus"); break;
        case PICC_TYPE_MIFARE_DESFIRE:  printf("MIFARE DESFire"); break;
        case PICC_TYPE_TNP3XXX:         printf("MIFARE TNP3XXX"); break;
        case PICC_TYPE_NOT_COMPLETE:    printf("SAK indica que o UID nao esta completo."); break;
        case PICC_TYPE_UNKNOWN:
        default:                        printf("Tipo desconhecido");
    }
    // printf("\n"); // Removido para ser adicionado pelo chamador se necessário
}

PICC_Type PICC_ObterTipo(uint8_t sak) {
    if (sak & 0x04) { // UID not complete
        return PICC_TYPE_NOT_COMPLETE;
    }
    switch (sak & ~0x04) { // Ignora o bit de UID incompleto para o resto da checagem
        case 0x09:  return PICC_TYPE_MIFARE_MINI;
        case 0x08:  return PICC_TYPE_MIFARE_1K;
        case 0x18:  return PICC_TYPE_MIFARE_4K;
        case 0x00:  return PICC_TYPE_MIFARE_UL;
        case 0x10:
        case 0x11:  return PICC_TYPE_MIFARE_PLUS; // Pode precisar de mais checagens para distinguir
        case 0x01:  return PICC_TYPE_TNP3XXX;     // Geralmente P3P (SmartMX)
        case 0x20:  return PICC_TYPE_ISO_14443_4; // Se SAK bit 5 é 1, então é ISO/IEC 14443-4
        case 0x40:  return PICC_TYPE_ISO_18092;   // Se SAK bit 6 é 1, então é ISO/IEC 18092
        default:    return PICC_TYPE_UNKNOWN;
    }
}

uint8_t PICC_RequisitarA(spi_device_handle_t spi, uint8_t *bufferATQA, uint8_t *tamanhoBuffer) {
    return PICC_RequisitarOuAcordarA(spi, PICC_CMD_REQA, bufferATQA, tamanhoBuffer);
}

// Função interna, não precisa estar no .h como pública
static uint8_t PICC_RequisitarOuAcordarA(spi_device_handle_t spi, uint8_t comando, uint8_t *bufferATQA, uint8_t *tamanhoBuffer) {
    if (bufferATQA == NULL || *tamanhoBuffer < 2) {
        return STATUS_NO_ROOM;
    }
    PCD_LimparMascaraBitsRegistrador(spi, CollReg, 0x80); // ValuesAfterColl=1 -> Limpa bits após colisão
    PCD_EscreverRegistrador(spi, BitFramingReg, 0x07); // TxLastBits = 7 -> Envia 7 bits
    
    uint8_t status = PCD_TransceptarDados(spi, &comando, 1, bufferATQA, tamanhoBuffer, NULL, 0x00, false);
    
    if (status != STATUS_OK) {
        return status;
    }
    if (*tamanhoBuffer != 2) { // ATQA deve ter 2 bytes (16 bits)
        return STATUS_ERROR;
    }
    // O MFRC522 já lida com a recepção de frames curtos.
    // A verificação de bitsValidos na resposta do ATQA é implícita pelo sucesso da transcepção
    // e pelo tamanho correto da resposta.
    return STATUS_OK;
}

uint8_t PCD_TransceptarDados(spi_device_handle_t spi,
                           uint8_t *dadosEnviar, uint8_t comprimentoEnviar,
                           uint8_t *dadosRecebidos, uint8_t *comprimentoRecebido,
                           uint8_t *bitsValidos, uint8_t alinhamentoRx, // alinhamentoRx é parte do BitFramingReg
                           bool verificarCRC) {
    uint8_t esperarIRq = ComIrqReg_RxIRq | ComIrqReg_IdleIRq; // Constantes de bits do ComIrqReg
    return PCD_ComunicarComPICC(spi, PCD_Transceive, esperarIRq, dadosEnviar, comprimentoEnviar, dadosRecebidos, comprimentoRecebido, bitsValidos, alinhamentoRx, verificarCRC);
}

// Função interna, não precisa estar no .h como pública
static uint8_t PCD_ComunicarComPICC(spi_device_handle_t spi,
                                uint8_t comandoPCD, uint8_t esperarIRq,
                                uint8_t *dadosEnviar, uint8_t comprimentoEnviar,
                                uint8_t *dadosRecebidos, uint8_t *comprimentoRecebido,
                                uint8_t *bitsValidosRx, uint8_t alinhamentoRx, // alinhamentoRx é BitFramingReg[6:4]
                                bool verificarCRC) {
    // Configura BitFramingReg antes do comando se 'bitsValidosTx' (para o último byte enviado) for relevante
    // Para a maioria dos comandos, bitsValidosTx é 0 (byte completo).
    // Para REQA/WUPA, é 7, mas já foi setado por PICC_RequisitarOuAcordarA.
    // O parâmetro 'bitsValidosRx' é para a SAÍDA (o que foi recebido).
    // O parâmetro 'alinhamentoRx' é para configurar BitFramingReg[6:4] para a recepção.
    uint8_t ultimosBitsTx = 0; // Padrão: envia bytes completos. Modificado por REQA/WUPA.
    if (comandoPCD == PCD_Transceive && comprimentoEnviar == 1 &&
        (dadosEnviar[0] == PICC_CMD_REQA || dadosEnviar[0] == PICC_CMD_WUPA)) {
        ultimosBitsTx = 7;
    }
    uint8_t enquadramentoBits = (alinhamentoRx << 4) | ultimosBitsTx;


    PCD_EscreverRegistrador(spi, CommandReg, PCD_Idle);      // Para comando anterior
    PCD_EscreverRegistrador(spi, ComIrqReg, 0x7F);           // Limpa todos os bits de interrupção
    PCD_DefinirMascaraBitsRegistrador(spi, FIFOLevelReg, 0x80); // FlushBuffer = 1
    
    if (comprimentoEnviar > 0) {
        PCD_EscreverRegistradorMultiplos(spi, FIFODataReg, comprimentoEnviar, dadosEnviar);
    }

    PCD_EscreverRegistrador(spi, BitFramingReg, enquadramentoBits); // Ajustes de bit
    PCD_EscreverRegistrador(spi, CommandReg, comandoPCD);      // Executa o comando

    if (comandoPCD == PCD_Transceive) {
        PCD_DefinirMascaraBitsRegistrador(spi, BitFramingReg, 0x80); // StartSend=1
    }

    // Espera o comando completar
    uint8_t n_irq;
    TickType_t ticks_inicio = xTaskGetTickCount();
    TickType_t ticks_timeout_cmd_mfrc522 = pdMS_TO_TICKS(100); // Aumentado um pouco (era 20ms)

    while (1) {
        n_irq = PCD_LerRegistrador(spi, ComIrqReg);
        if (n_irq & esperarIRq) { // Um dos IRQs esperados ocorreu
            break;
        }
        if (n_irq & ComIrqReg_TimerIRq) { // Timeout do timer interno do MFRC522 (para comunicação com PICC)
            printf("A\n");
            return STATUS_TIMEOUT;
            
        }
        if ((xTaskGetTickCount() - ticks_inicio) > ticks_timeout_cmd_mfrc522) {
            // O próprio MFRC522 não sinalizou conclusão/erro/timeout do PICC. Pode ser problema de comunicação com MFRC522.
            // printf("E: Timeout interno do MFRC522 (ComIrqReg polling)\n");
            PCD_EscreverRegistrador(spi, CommandReg, PCD_Idle); // Tenta parar qualquer comando pendente
            return STATUS_ERROR; // Ou um status mais específico
        }
        // vTaskDelay(pdMS_TO_TICKS(1)); // Opcional, para ceder CPU. Pode adicionar latência.
    }
    PCD_LimparMascaraBitsRegistrador(spi, BitFramingReg, 0x80); // Limpa StartSend

    uint8_t valorErroReg = PCD_LerRegistrador(spi, ErrorReg);
    if (valorErroReg & (ErrorReg_BufferOvfl | ErrorReg_ParityErr | ErrorReg_ProtocolErr)) {
        // printf("E: PCD_ComunicarComPICC ErrorReg: 0x%02X\n", valorErroReg);
        return STATUS_ERROR;
    }

    uint8_t _bitsValidosRecebidos = 0;
    if (dadosRecebidos && comprimentoRecebido) {
        uint8_t nivelFifo = PCD_LerRegistrador(spi, FIFOLevelReg);
        if (nivelFifo > *comprimentoRecebido && *comprimentoRecebido > 0) { // Se *comprimentoRecebido é 0, significa que o chamador não se importa com o tamanho exato, apenas com os dados
            // printf("E: STATUS_NO_ROOM, FIFO: %d, Buffer: %d\n", nivelFifo, *comprimentoRecebido);
            return STATUS_NO_ROOM;
        }
        if (*comprimentoRecebido == 0) *comprimentoRecebido = nivelFifo; // Se 0, usa o que tem no FIFO
        else if (nivelFifo < *comprimentoRecebido) *comprimentoRecebido = nivelFifo; // Se buffer maior, usa o que tem no FIFO


        if (*comprimentoRecebido > 0) {
             PCD_LerRegistradorMultiplos(spi, FIFODataReg, *comprimentoRecebido, dadosRecebidos, alinhamentoRx);
        }
        _bitsValidosRecebidos = PCD_LerRegistrador(spi, ControlReg) & 0x07; // RxLastBits
        if (bitsValidosRx) {
            *bitsValidosRx = _bitsValidosRecebidos;
        }
    }

    if (valorErroReg & ErrorReg_CollErr) {
        return STATUS_COLLISION;
    }

    if (dadosRecebidos && comprimentoRecebido && verificarCRC && *comprimentoRecebido > 0) {
        if (*comprimentoRecebido == 1 && _bitsValidosRecebidos == 4) {
            // É um ACK/NAK de 4 bits, o chamador deve verificar o valor
            // Se o chamador esperava dados + CRC, então isto é um erro de protocolo ou NAK.
            // Para MIFARE_Transceive, isso é tratado lá. Aqui, retornamos MIFARE_NACK.
            if( (dadosRecebidos[0] & 0x0F) != MF_ACK) return STATUS_MIFARE_NACK;
            else return STATUS_OK; // Era um ACK
        }
        if (*comprimentoRecebido < 2 || _bitsValidosRecebidos != 0) {
            // printf("E: CRC_ERRADO - Comprimento/bits invalidos para CRC. compRec=%d, _bitsValidos=%d\n", *comprimentoRecebido, _bitsValidosRecebidos);
            return STATUS_CRC_WRONG;
        }
        uint8_t bufferControle[2];
        uint8_t status_crc = PCD_CalcularCRC(spi, dadosRecebidos, *comprimentoRecebido - 2, bufferControle);
        if (status_crc != STATUS_OK) {
            return status_crc;
        }
        if ((dadosRecebidos[*comprimentoRecebido - 2] != bufferControle[0]) || (dadosRecebidos[*comprimentoRecebido - 1] != bufferControle[1])) {
            // printf("E: STATUS_CRC_ERRADO - Divergencia. Rec: %02X%02X, Calc: %02X%02X\n",
            //     dadosRecebidos[*comprimentoRecebido - 2], dadosRecebidos[*comprimentoRecebido - 1], bufferControle[0], bufferControle[1]);
            return STATUS_CRC_WRONG;
        }
    }
    return STATUS_OK;
}

uint8_t PICC_LerSerialCartao(spi_device_handle_t spi, Uid *uidCartao) {
    if (uidCartao == NULL) {
        return STATUS_INVALID;
        printf("ferrou!\n");
    }
    return PICC_Selecionar(spi, uidCartao, 0);
}

uint8_t PICC_Selecionar(spi_device_handle_t spi, Uid *uid, uint8_t bitsValidosConhecidos) {
    bool uidCompleto = false;
    uint8_t nivelCascata = 1;
    uint8_t resultado;
    printf("1\n");
    // Limpa o UID e SAK antes de começar
    memset(uid->uidByte, 0, sizeof(uid->uidByte));
    uid->size = 0;
    uid->sak = 0;

    PCD_LimparMascaraBitsRegistrador(spi, CollReg, 0x80); // ValuesAfterColl=1
    printf("2\n");
    while (!uidCompleto) {
        uint8_t comandoSelecao;
        uint8_t indiceUidAtual = 0; // Primeiro byte do UID para este nível de cascata
        bool usarTagCascataEsteNivel = false;

        switch (nivelCascata) {
            case 1:
                comandoSelecao = PICC_CMD_SEL_CL1;
                indiceUidAtual = 0;
                // Determina se o UID conhecido (se houver) sugere que este cartão tem mais de 4 bytes.
                // Se bitsValidosConhecidos > 32, significa que já passamos por CL1.
                usarTagCascataEsteNivel = (bitsValidosConhecidos > 32);
                printf("3\n");
                break;
            case 2:
                comandoSelecao = PICC_CMD_SEL_CL2;
                indiceUidAtual = 3; // UID bytes 3, 4, 5, (6 se CT)
                 // Se bitsValidosConhecidos > 56 (32 de CL1 + 24 de CL2 sem CT), sugere CL3
                usarTagCascataEsteNivel = (bitsValidosConhecidos > 56);
                printf("4\n");
                break;
            case 3:
                comandoSelecao = PICC_CMD_SEL_CL3;
                indiceUidAtual = 6; // UID bytes 6, 7, 8, 9
                usarTagCascataEsteNivel = false; // CL3 nunca usa CT
                printf("5\n");
                break;
            default:
                return STATUS_INTERNAL_ERROR;
        }

        uint8_t bufferComando[9]; // Max: SEL + NVB + CT + UIDn + UIDn+1 + UIDn+2 + BCC + CRC_A(2)
        uint8_t bytesUidNesteNivel = 0;
        uint8_t bitsConhecidosNesteNivel = 0;
        printf("6\n");
        if (bitsValidosConhecidos > (indiceUidAtual * 8)) {
            bitsConhecidosNesteNivel = bitsValidosConhecidos - (indiceUidAtual * 8);
            if (usarTagCascataEsteNivel) {
                 if (bitsConhecidosNesteNivel > 8) bitsConhecidosNesteNivel -=8; // Subtrai bits do CT se já conhecidos
                 else bitsConhecidosNesteNivel = 0;
            }
            if (bitsConhecidosNesteNivel > 32) bitsConhecidosNesteNivel = 32; // Max 4 bytes de UID por nível
        }


        bufferComando[0] = comandoSelecao;
        uint8_t indiceBuffer = 2; // Posição para UID/CT

        if (usarTagCascataEsteNivel) {
            bufferComando[indiceBuffer++] = PICC_CMD_CT;
        }

        // Copia os bits conhecidos do UID para este nível
        uint8_t numBytesUidParaCopiar = (bitsConhecidosNesteNivel + 7) / 8;
        for (uint8_t i = 0; i < numBytesUidParaCopiar; ++i) {
            if ((indiceBuffer - 2 - (usarTagCascataEsteNivel ? 1:0)) < 4) { // Não exceder 4 bytes de UID (ou 3 se CT)
                bufferComando[indiceBuffer++] = uid->uidByte[indiceUidAtual + i];
            }
        }
        bytesUidNesteNivel = indiceBuffer - 2 - (usarTagCascataEsteNivel ? 1:0);


        // Loop de anti-colisão para este nível de cascata
        bool selecaoNivelCompleta = false;
        uint8_t transmitirUltimosBits = bitsConhecidosNesteNivel % 8;
        printf("7\n");
        while (!selecaoNivelCompleta) {
            uint8_t nvb; // Number of Valid Bits byte
            uint8_t comprimentoEnvio;
            uint8_t bufferResposta[5]; // Max: UID parcial (até 4) + BCC
            uint8_t comprimentoResposta = sizeof(bufferResposta);
            uint8_t bitsValidosResposta = 0;
            printf("8\n");

            if ( (usarTagCascataEsteNivel && bytesUidNesteNivel == 3 && transmitirUltimosBits == 0) ||
                 (!usarTagCascataEsteNivel && bytesUidNesteNivel == 4 && transmitirUltimosBits == 0) ) {
                // Temos todos os 4 bytes (ou 3+CT) para este nível -> SELECT
                nvb = 0x70; // SEL + NVB + (CT)UID(3/4) + BCC = 7 bytes
                bufferComando[1] = nvb;
                // Calcula BCC sobre (CT)UID0 UID1 UID2 (UID3)
                bufferComando[2 + (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel] = 0; // BCC
                for(uint8_t i=0; i < (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel; ++i) {
                    bufferComando[2 + (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel] ^= bufferComando[2+i];
                }
                comprimentoEnvio = 2 + (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel + 1; // +1 for BCC
                resultado = PCD_CalcularCRC(spi, bufferComando, comprimentoEnvio, &bufferComando[comprimentoEnvio]);
                if (resultado != STATUS_OK) {printf("9\n");return resultado;}
                comprimentoEnvio += 2; // +2 for CRC
                transmitirUltimosBits = 0; // Envia bytes completos
            } else {
                // Anti-colisão: envia o que temos até agora
                printf("10\n");
                nvb = ( (2 + (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel) << 4) | transmitirUltimosBits;
                bufferComando[1] = nvb;
                comprimentoEnvio = 2 + (usarTagCascataEsteNivel?1:0) + bytesUidNesteNivel + (transmitirUltimosBits > 0 ? 1 : 0);
                // CRC não é enviado em anti-colisão
            }

            // Configura BitFramingReg para transmissão e recepção
            // RxAlign = 0 (padrão). TxLastBits = transmitirUltimosBits.
            PCD_EscreverRegistrador(spi, BitFramingReg, (0x00 << 4) | transmitirUltimosBits);
            printf("11\n");
            resultado = PCD_TransceptarDados(spi, bufferComando, comprimentoEnvio,
                                           bufferResposta, &comprimentoResposta, &bitsValidosResposta, 0x00, false);

            if (resultado == STATUS_COLLISION) {
                printf("12\n");
                uint8_t collReg = PCD_LerRegistrador(spi, CollReg);
                if (collReg & CollReg_CollPosNotValid) {
                    return STATUS_COLLISION; // Não é possível resolver
                }
                uint8_t collisionPos = collReg & 0x1F; // Bits 0-4 dão a posição do bit (1-31, 0 significa 32)
                if (collisionPos == 0) collisionPos = 32;

                // A posição de colisão é relativa ao início dos dados do UID (após SEL e NVB)
                // Se CT, collisionPos é relativa ao CT.
                uint8_t bitsEnviadosUidCt = (bytesUidNesteNivel * 8) + transmitirUltimosBits;
                if (usarTagCascataEsteNivel) bitsEnviadosUidCt +=8;


                if (collisionPos > bitsEnviadosUidCt) { // Colisão em um bit que não enviamos completamente?
                    return STATUS_INTERNAL_ERROR; // Teórico, não deveria acontecer
                }

                // Adiciona o bit de colisão à nossa tentativa e tenta novamente
                uint8_t byteIdxNoComando = 2; // Início do CT/UID no bufferComando
                uint8_t bitNoTotal = collisionPos -1; // 0-indexed

                if (usarTagCascataEsteNivel) {
                    if (bitNoTotal < 8) { /* Colisão no CT, não deveria acontecer se CT é fixo */ return STATUS_INTERNAL_ERROR;}
                    bitNoTotal -= 8; // Ajusta para ser relativo ao UID
                }
                
                byteIdxNoComando += bitNoTotal / 8;
                uint8_t bitIdxNoByte = bitNoTotal % 8;

                bufferComando[byteIdxNoComando] |= (1 << bitIdxNoByte); // Seta o bit (tenta o '1')

                // Atualiza quantos bits/bytes estamos enviando agora
                bitsConhecidosNesteNivel = bitNoTotal +1; // Agora conhecemos até este bit (inclusive)
                bytesUidNesteNivel = (bitsConhecidosNesteNivel + 7) / 8;
                transmitirUltimosBits = bitsConhecidosNesteNivel % 8;


            } else if (resultado != STATUS_OK) {
                return resultado; // Outro erro
                printf("13\n");
            } else {
                printf("14\n");
                // SUCESSO (OK)
                if ( (usarTagCascataEsteNivel && bytesUidNesteNivel == 3 && transmitirUltimosBits == 0) ||
                     (!usarTagCascataEsteNivel && bytesUidNesteNivel == 4 && transmitirUltimosBits == 0) ) {
                    // Este foi um SELECT, e tivemos uma resposta (SAK)
                    // bufferResposta agora contém SAK[0] CRC[1] CRC[2]
                    if (comprimentoResposta != 3 || bitsValidosResposta != 0) {
                        return STATUS_ERROR; // Resposta SAK inválida
                    }
                    // Verifica CRC do SAK
                    uint8_t crcCheck[2];
                    PCD_CalcularCRC(spi, &bufferResposta[0], 1, crcCheck);
                    if (crcCheck[0] != bufferResposta[1] || crcCheck[1] != bufferResposta[2]) {
                        return STATUS_CRC_WRONG;
                    }
                    uid->sak = bufferResposta[0];
                    selecaoNivelCompleta = true; // Saímos do loop de anti-colisão para este nível
                } else {
                    printf("15\n");
                    // Este foi um ANTICOLLISION, e tivemos uma resposta (parte do UID + BCC)
                    // bufferResposta contém UID bytes + BCC
                    // Precisamos copiar os bytes do UID para o nosso bufferComando e continuar
                    if (comprimentoResposta == 0) return STATUS_ERROR; // Sem resposta
                    
                    uint8_t bccRecebido = bufferResposta[comprimentoResposta-1];
                    uint8_t bccCalculado = 0;
                    uint8_t bytesUidRecebidos = comprimentoResposta -1;

                    uint8_t k = (usarTagCascataEsteNivel?1:0); // offset se CT
                    for(uint8_t i=0; i < bytesUidRecebidos; ++i) {
                        if (bytesUidNesteNivel + i < 4) { // Não exceder 4 bytes de UID
                             bufferComando[2 + k + bytesUidNesteNivel + i] = bufferResposta[i];
                             bccCalculado ^= bufferResposta[i];
                        }
                    }
                    // O BCC do MFRC522 é sobre os 5 bytes (CT+UID ou UID1-5).
                    // A resposta do PICC é sobre os bytes UID que ele enviou.
                    // Re-calcular BCC com base no que temos no bufferComando
                    bccCalculado = bufferComando[2]; // Primeiro byte (CT ou UID0)
                    for(uint8_t i=1; i < (usarTagCascataEsteNivel?4:5); ++i) { // Até 4 ou 5 bytes totais
                        if (2+i < sizeof(bufferComando)) bccCalculado ^= bufferComando[2+i]; else break;
                    }

                    // Como o BCC é retornado pelo PICC para os bytes UID que ele transmitiu,
                    // e a biblioteca original parece assumir que já temos 4 bytes + CT,
                    // esta parte da anti-colisão é complexa.
                    // A implementação mais simples de anti-colisão é quando o PICC responde com seu UID completo
                    // para aquele nível de cascata.
                    // Simplificação: assumimos que a anti-colisão nos deu todos os bytes para este nível.
                    for(uint8_t i=0; i<bytesUidRecebidos; ++i) {
                         if ( (usarTagCascataEsteNivel && bytesUidNesteNivel + i < 3) ||
                              (!usarTagCascataEsteNivel && bytesUidNesteNivel + i < 4) ) {
                             uid->uidByte[indiceUidAtual + bytesUidNesteNivel + i] = bufferResposta[i];
                         }
                    }
                    bytesUidNesteNivel += bytesUidRecebidos;
                    if (usarTagCascataEsteNivel && bytesUidNesteNivel > 3) bytesUidNesteNivel = 3;
                    if (!usarTagCascataEsteNivel && bytesUidNesteNivel > 4) bytesUidNesteNivel = 4;

                    transmitirUltimosBits = 0; // Agora conhecemos bytes completos
                }
            }
        } // Fim do while (!selecaoNivelCompleta) -> Loop de anti-colisão/seleção para este nível
        printf("16\n");
        // Copia os bytes do UID deste nível para a estrutura uid final
        uint8_t bytesParaCopiarEsteNivel = (usarTagCascataEsteNivel ? 3 : 4);
        for (uint8_t i = 0; i < bytesParaCopiarEsteNivel; ++i) {
            // O UID já foi preenchido durante o SELECT ou ANTICOLLISION
            // uid->uidByte[indiceUidAtual + i] = bufferComando[2 + (usarTagCascataEsteNivel ? 1 : 0) + i];
        }
        uid->size += bytesParaCopiarEsteNivel;
        if (usarTagCascataEsteNivel) uid->size++; // Conta o CT


        if (uid->sak & 0x04) { // Bit Cascade set no SAK
            nivelCascata++;
            if (nivelCascata > 3) return STATUS_INTERNAL_ERROR; // Max 3 níveis
            // bitsValidosConhecidos já inclui os bits deste nível.
            // Para o próximo nível, só precisamos adicionar os bits do CT, se houver.
        } else {
            printf("17\n");
            uidCompleto = true; // SAK não tem cascade bit, UID está completo
        }
    } // Fim do while (!uidCompleto) -> Loop de níveis de cascata

    // Ajusta o tamanho final do UID
    if (nivelCascata == 1) uid->size = 4;
    else if (nivelCascata == 2) uid->size = 7;
    else if (nivelCascata == 3) uid->size = 10;
    printf("18\n");
    return STATUS_OK;
}


uint8_t PCD_CalcularCRC(spi_device_handle_t spi, uint8_t *dados, uint8_t comprimento, uint8_t *resultado) {
    PCD_EscreverRegistrador(spi, CommandReg, PCD_Idle);
    PCD_DefinirMascaraBitsRegistrador(spi, DivIrqReg, DivIrqReg_CRCIRq); // Limpa CRCIRq
    PCD_DefinirMascaraBitsRegistrador(spi, FIFOLevelReg, 0x80); // FlushBuffer

    PCD_EscreverRegistradorMultiplos(spi, FIFODataReg, comprimento, dados);
    PCD_EscreverRegistrador(spi, CommandReg, PCD_CalcCRC);

    TickType_t ticks_inicio = xTaskGetTickCount();
    // Timeout aumentado, pois em alguns casos pode demorar um pouco mais
    // especialmente se o MFRC522 estiver ocupado ou com clock SPI lento.
    TickType_t ticks_timeout = pdMS_TO_TICKS(100); // 100ms

    uint8_t n_irq;
    while(1) {
        n_irq = PCD_LerRegistrador(spi, DivIrqReg);
        if (n_irq & DivIrqReg_CRCIRq) {
            PCD_EscreverRegistrador(spi, CommandReg, PCD_Idle); // Para o comando CalcCRC
            resultado[0] = PCD_LerRegistrador(spi, CRCResultRegL);
            resultado[1] = PCD_LerRegistrador(spi, CRCResultRegH);
            return STATUS_OK;
        }
        if ((xTaskGetTickCount() - ticks_inicio) > ticks_timeout) {
            // printf("E: Timeout PCD_CalcularCRC\n");
            printf("B\n");
            return STATUS_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Pequena pausa para não sobrecarregar
    }
}

void PICC_ImprimirNoSerial(spi_device_handle_t spi, Uid *uid) {
    MIFARE_Key chave;
    for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {
        chave.keyByte[i] = 0xFF; // Chave padrão
    }

    PICC_ImprimirDetalhesNoSerial(uid);
    PICC_Type tipoPicc = PICC_ObterTipo(uid->sak);

    switch (tipoPicc) {
        case PICC_TYPE_MIFARE_MINI:
        case PICC_TYPE_MIFARE_1K:
        case PICC_TYPE_MIFARE_4K:
            PICC_ImprimirMifareClassicNoSerial(spi, uid, tipoPicc, &chave);
            break;
        case PICC_TYPE_MIFARE_UL:
            PICC_ImprimirMifareUltralightNoSerial(spi);
            break;
        default:
            printf("Impressao do conteudo da memoria nao implementada para este tipo de PICC.\n");
            break;
    }
}

void PICC_ImprimirDetalhesNoSerial(Uid *uid) {
    printf("UID do Cartao:");
    for (uint8_t i = 0; i < uid->size; i++) {
        printf(" %02X", uid->uidByte[i]);
    }
    printf("\r\nSAK do Cartao: 0x%02X (", uid->sak);
    PICC_ObterNomeTipo(PICC_ObterTipo(uid->sak));
    printf(")\n");
}

void PICC_ImprimirMifareClassicNoSerial(spi_device_handle_t spi, Uid *uid, PICC_Type tipoPicc, MIFARE_Key *chave) {
    uint8_t num_setores = 0;
    switch (tipoPicc) {
        case PICC_TYPE_MIFARE_MINI: num_setores = 5; break;
        case PICC_TYPE_MIFARE_1K:   num_setores = 16; break;
        case PICC_TYPE_MIFARE_4K:   num_setores = 40; break;
        default: printf("Tipo MIFARE Classic nao suportado para impressao.\n"); return;
    }

    if (num_setores) {
        printf("Setor Bloco   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  Bits Acesso\r\n");
        for (int8_t i = num_setores - 1; i >= 0; i--) { // Imprime do último setor para o primeiro
            PICC_ImprimirSetorMifareClassicNoSerial(spi, uid, chave, i);
        }
    }
    PICC_PararA(spi); // Coloca o cartão em HALT
    PCD_PararCripto1(spi); // Para a criptografia MIFARE no leitor
}

void PICC_ImprimirSetorMifareClassicNoSerial(spi_device_handle_t spi, Uid *uid, MIFARE_Key *chave, uint8_t setor) {
    uint8_t status_auth;
    uint8_t primeiroBlocoSector;
    uint8_t numBlocosNoSector;

    if (setor < 32) { // Setores 0-31 têm 4 blocos cada
        numBlocosNoSector = 4;
        primeiroBlocoSector = setor * numBlocosNoSector;
    } else if (setor < 40) { // Setores 32-39 (para MIFARE 4K) têm 16 blocos cada
        numBlocosNoSector = 16;
        primeiroBlocoSector = 128 + (setor - 32) * numBlocosNoSector;
    } else {
        printf("Setor invalido: %d\n", setor);
        return;
    }

    // Autentica o trailer do setor (último bloco do setor)
    uint8_t trailerBloco = primeiroBlocoSector + numBlocosNoSector - 1;
    status_auth = PCD_Autenticar(spi, PICC_CMD_MF_AUTH_KEY_A, trailerBloco, chave, uid);

    if (status_auth != STATUS_OK) {
        printf("Falha ao autenticar setor %d com Chave A. Status: ", setor);
        ObterNomeCodigoStatus(status_auth);
        printf("\n");
        // Tentar com Chave B (se disponível/conhecida) poderia ser uma opção aqui
        return;
    }
    // printf("Setor %d autenticado.\n", setor);

    uint8_t buffer[18]; // MIFARE_Ler espera buffer de pelo menos 18 (16 dados + 2 para CRC interno)
    bool ehTrailerSetorAtual = false;

    for (int8_t offsetBloco = numBlocosNoSector - 1; offsetBloco >= 0; offsetBloco--) {
        uint8_t enderecoBlocoAtual = primeiroBlocoSector + offsetBloco;
        ehTrailerSetorAtual = (offsetBloco == numBlocosNoSector - 1);

        if (ehTrailerSetorAtual) { // Só imprime o número do setor uma vez, para o trailer
            printf("%s%2d%s", (setor < 10 ? "  " : " "), setor, "   ");
        } else {
            printf("       "); // Espaçamento para alinhar com o número do setor
        }
        printf("%s%3d%s", (enderecoBlocoAtual < 10 ? "  " : (enderecoBlocoAtual < 100 ? " " : "")), enderecoBlocoAtual, "  ");

        uint8_t tamanhoBufferLeitura = sizeof(buffer);
        uint8_t status_leitura = MIFARE_Ler(spi, enderecoBlocoAtual, buffer, &tamanhoBufferLeitura);

        if (status_leitura == STATUS_OK && tamanhoBufferLeitura >= 16) {
            for (uint8_t i = 0; i < 16; i++) {
                printf(" %02X", buffer[i]);
                if ((i % 4) == 3 && i < 15) { // Espaço extra a cada 4 bytes
                    printf(" ");
                }
            }
            if (ehTrailerSetorAtual) { // Se for o trailer, decodifica e imprime os bits de acesso
                uint8_t c1 = buffer[7] >> 4;
                uint8_t c2 = buffer[8] & 0xF;
                uint8_t c3 = buffer[8] >> 4;
                // uint8_t c1_ = buffer[6] & 0xF; // Invertidos, para verificação de consistência
                // uint8_t c2_ = buffer[6] >> 4;
                // uint8_t c3_ = buffer[7] & 0xF;
                // bool erroInvertido = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));

                uint8_t g[4]; // Bits de acesso para os 4 grupos (bloco 0, 1, 2, trailer)
                g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0); // C11 C21 C31 -> Bloco 0
                g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1); // C12 C22 C32 -> Bloco 1
                g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2); // C13 C23 C33 -> Bloco 2
                g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3); // C14 C24 C34 -> Trailer

                // Imprime bits de acesso do trailer (g[3])
                 printf(" [ %d %d %d ] (Trailer)", (g[3] >> 2) & 1, (g[3] >> 1) & 1, (g[3] >> 0) & 1);
                // if (erroInvertido) printf(" !INV!");
            } else { // Se for bloco de dados, imprime seus bits de acesso
                 uint8_t grupoBloco = offsetBloco % 4; // 0, 1, 2 (para blocos de dados)
                 if (numBlocosNoSector == 16) { // Mapeamento para MIFARE 4K
                     if(offsetBloco < 5) grupoBloco = 0; // Bloco 0-4
                     else if (offsetBloco < 10) grupoBloco = 1; // Bloco 5-9
                     else if (offsetBloco < 15) grupoBloco = 2; // Bloco 10-14
                     // Trailer (grupo 3) já tratado
                 }
                 if (grupoBloco < 3) { // Só para blocos de dados
                    // Recalcula c1,c2,c3 do trailer para obter os 'g' corretos se não for o trailer
                    uint8_t trailer_data[16];
                    uint8_t trailer_len = 16;
                    if (MIFARE_Ler(spi, trailerBloco, trailer_data, &trailer_len) == STATUS_OK) {
                        uint8_t c1_tr = trailer_data[7] >> 4;
                        uint8_t c2_tr = trailer_data[8] & 0xF;
                        uint8_t c3_tr = trailer_data[8] >> 4;
                        uint8_t g_temp[4];
                        g_temp[0] = ((c1_tr & 1) << 2) | ((c2_tr & 1) << 1) | ((c3_tr & 1) << 0);
                        g_temp[1] = ((c1_tr & 2) << 1) | ((c2_tr & 2) << 0) | ((c3_tr & 2) >> 1);
                        g_temp[2] = ((c1_tr & 4) << 0) | ((c2_tr & 4) >> 1) | ((c3_tr & 4) >> 2);

                        printf(" [ %d %d %d ]", (g_temp[grupoBloco] >> 2) & 1, (g_temp[grupoBloco] >> 1) & 1, (g_temp[grupoBloco] >> 0) & 1);
                    }
                 }
            }
        } else {
            printf("Falha ao ler bloco %d. Status: ", enderecoBlocoAtual);
            ObterNomeCodigoStatus(status_leitura);
        }
        printf("\n");
    }
}


uint8_t PCD_Autenticar(spi_device_handle_t spi, uint8_t comandoPICC, uint8_t enderecoBloco, MIFARE_Key *chave, Uid *uid) {
    uint8_t esperarIRq = ComIrqReg_IdleIRq;
    uint8_t dadosEnviar[12];

    dadosEnviar[0] = comandoPICC; // PICC_CMD_MF_AUTH_KEY_A ou PICC_CMD_MF_AUTH_KEY_B
    dadosEnviar[1] = enderecoBloco;
    for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {
        dadosEnviar[2 + i] = chave->keyByte[i];
    }
    // UID: Os primeiros 4 bytes do UID do cartão são usados na autenticação MIFARE Classic
    for (uint8_t i = 0; i < 4; i++) {
        if (i < uid->size) {
            dadosEnviar[8 + i] = uid->uidByte[i];
        } else {
            dadosEnviar[8 + i] = 0x00; // Preenche com 0 se UID for menor que 4 bytes
        }
    }
    // A autenticação não retorna dados do PICC diretamente, apenas o status do leitor.
    // O sucesso é indicado por Status2Reg.MFCrypto1On ser setado.
    uint8_t status_comm = PCD_ComunicarComPICC(spi, PCD_MFAuthent, esperarIRq, dadosEnviar, sizeof(dadosEnviar), NULL, 0, NULL, 0, false);
    
    if (status_comm != STATUS_OK) return status_comm;

    // Verifica se o bit MFCrypto1On está setado no Status2Reg
    if (!(PCD_LerRegistrador(spi, Status2Reg) & Status2Reg_MFCrypto1On)) {
        return STATUS_ERROR; // Autenticação falhou (não ativou criptografia)
    }
    return STATUS_OK;
}

uint8_t MIFARE_Ler(spi_device_handle_t spi, uint8_t enderecoBloco, uint8_t *buffer, uint8_t *tamanhoBuffer) {
    if (buffer == NULL || *tamanhoBuffer < 18) { // Precisa de espaço para 16 dados + 2 CRC que Transceive pode esperar
        // printf("E: MIFARE_Ler STATUS_NO_ROOM. TamanhoBuffer: %d\n", *tamanhoBuffer);
        return STATUS_NO_ROOM;
    }

    uint8_t bufferComando[4];
    bufferComando[0] = PICC_CMD_MF_READ;
    bufferComando[1] = enderecoBloco;
    uint8_t status_crc = PCD_CalcularCRC(spi, bufferComando, 2, &bufferComando[2]);
    if (status_crc != STATUS_OK) {
        return status_crc;
    }

    // MIFARE Classic responde com 16 bytes de dados + 2 bytes de CRC_A
    uint8_t comprimentoRetornoEsperado = 18; // 16 dados + 2 CRC
    uint8_t status_transc = PCD_TransceptarDados(spi, bufferComando, 4, buffer, &comprimentoRetornoEsperado, NULL, 0, true);
    
    if (status_transc == STATUS_OK) {
        if (comprimentoRetornoEsperado == 18) { // Sucesso e tamanho correto
             *tamanhoBuffer = 16; // Retorna ao chamador apenas os 16 bytes de dados
        } else if (comprimentoRetornoEsperado == 1 && (buffer[0] & 0x0F) != MF_ACK) { // NAK (4 bits)
            *tamanhoBuffer = 0;
            return STATUS_MIFARE_NACK;
        } else { // Tamanho inesperado
            *tamanhoBuffer = comprimentoRetornoEsperado;
            return STATUS_ERROR;
        }
    } else {
        *tamanhoBuffer = 0; // Nenhum dado válido
    }
    return status_transc;
}

uint8_t MIFARE_Escrever(spi_device_handle_t spi, uint8_t enderecoBloco, uint8_t *bufferDados16Bytes, uint8_t tamanhoBuffer) {
    if (bufferDados16Bytes == NULL || tamanhoBuffer < 16) {
        return STATUS_INVALID; // Precisa de exatamente 16 bytes para escrever
    }

    // Passo 1: Enviar comando de escrita e endereço do bloco
    uint8_t bufferComando[2];
    bufferComando[0] = PICC_CMD_MF_WRITE;
    bufferComando[1] = enderecoBloco;
    uint8_t resultado = PCD_TransceptarMIFARE(spi, bufferComando, 2, false);
    if (resultado != STATUS_OK) {
        return resultado; // Falhou em obter ACK para o comando de escrita
    }

    // Passo 2: Enviar os 16 bytes de dados
    resultado = PCD_TransceptarMIFARE(spi, bufferDados16Bytes, 16, false);
    return resultado; // Retorna status do envio dos dados (espera ACK)
}

// Função interna, não precisa estar no .h como pública
static uint8_t PCD_TransceptarMIFARE(spi_device_handle_t spi, uint8_t *dadosEnviar, uint8_t comprimentoEnviar, bool aceitarTimeout) {
    uint8_t resultado;
    uint8_t bufferComandoComCRC[18]; // Máximo 16 dados + 2 CRC

    if (dadosEnviar == NULL || comprimentoEnviar > 16) {
        return STATUS_INVALID;
    }

    memcpy(bufferComandoComCRC, dadosEnviar, comprimentoEnviar);
    resultado = PCD_CalcularCRC(spi, bufferComandoComCRC, comprimentoEnviar, &bufferComandoComCRC[comprimentoEnviar]);
    if (resultado != STATUS_OK) {
        return resultado;
    }
    uint8_t comprimentoTotalEnviar = comprimentoEnviar + 2;

    // Para comandos MIFARE que esperam ACK/NAK, a resposta é curta (4 bits)
    uint8_t bufferResposta[1]; // Buffer para receber ACK/NAK
    uint8_t tamanhoBufferResposta = sizeof(bufferResposta);
    uint8_t bitsValidosResposta = 0;

    // Usar PCD_ComunicarComPICC diretamente para ter controle sobre a verificação do ACK/NAK
    uint8_t esperarIRq = ComIrqReg_RxIRq | ComIrqReg_IdleIRq;
    resultado = PCD_ComunicarComPICC(spi, PCD_Transceive, esperarIRq,
                                     bufferComandoComCRC, comprimentoTotalEnviar,
                                     bufferResposta, &tamanhoBufferResposta, &bitsValidosResposta,
                                     0, false); // CRC da resposta não é verificado aqui, mas o valor do ACK/NAK

    if (aceitarTimeout && resultado == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (resultado != STATUS_OK) {
        return resultado;
    }

    // Verifica se a resposta é um ACK (0xA) de 4 bits
    if (tamanhoBufferResposta != 1 || bitsValidosResposta != 4) {
        return STATUS_ERROR; // Resposta inesperada
    }
    if ((bufferResposta[0] & 0x0F) != MF_ACK) { // Verifica os 4 bits LSB
        return STATUS_MIFARE_NACK;
    }
    return STATUS_OK;
}

void PICC_ImprimirMifareUltralightNoSerial(spi_device_handle_t spi) {
    uint8_t status_leitura;
    uint8_t buffer[18]; // MIFARE_Ler espera buffer para 16 dados + CRC interno

    printf("Pagina  Hex: 0  1  2  3\r\n");
    // MIFARE Ultralight tem tipicamente 16 páginas de 4 bytes (0-15)
    // MIFARE_Ler para Ultralight lê 4 páginas (16 bytes) de uma vez.
    for (uint8_t paginaBase = 0; paginaBase < 16; paginaBase += 4) {
        uint8_t tamanhoBufferLeitura = sizeof(buffer);
        status_leitura = MIFARE_Ler(spi, paginaBase, buffer, &tamanhoBufferLeitura);

        if (status_leitura == STATUS_OK && tamanhoBufferLeitura >= 16) {
            for (uint8_t offsetPagina = 0; offsetPagina < 4; offsetPagina++) {
                uint8_t paginaAtual = paginaBase + offsetPagina;
                printf("%s%2d%s:", (paginaAtual < 10 ? "  " : " "), paginaAtual, "    ");
                for (uint8_t byteNaPagina = 0; byteNaPagina < 4; byteNaPagina++) {
                    printf(" %02X", buffer[offsetPagina * 4 + byteNaPagina]);
                }
                printf("\r\n");
            }
        } else {
            printf("Falha ao ler paginas a partir de %d. Status: ", paginaBase);
            ObterNomeCodigoStatus(status_leitura);
            printf(" (Recebidos: %d bytes)\n", tamanhoBufferLeitura);
            break; // Interrompe se houver erro
        }
    }
}
uint8_t Ultralight_LerPaginas(spi_device_handle_t spi, uint8_t pagina_inicio, uint8_t *dados_saida) {
    if (!dados_saida) return STATUS_INVALID;

    uint8_t comando[2] = { 0x30, pagina_inicio }; // Comando READ para Ultralight
    uint8_t tamanho = 16; // 4 páginas de 4 bytes
    return PCD_TransceptarDados(spi, comando, 2, dados_saida, &tamanho, NULL, 0x00, true);
}

void ObterNomeCodigoStatus(uint8_t codigo) {
    switch (codigo) {
        case STATUS_OK:             printf("Sucesso."); break;
        case STATUS_ERROR:          printf("Erro na comunicacao."); break;
        case STATUS_COLLISION:      printf("Colisao detectada."); break;
        case STATUS_TIMEOUT:        printf("Timeout na comunicacao."); break;
        case STATUS_NO_ROOM:        printf("Buffer insuficiente."); break;
        case STATUS_INTERNAL_ERROR: printf("Erro interno no codigo."); break;
        case STATUS_INVALID:        printf("Argumento invalido."); break;
        case STATUS_CRC_WRONG:      printf("CRC_A nao corresponde."); break;
        case STATUS_MIFARE_NACK:    printf("PICC MIFARE respondeu com NAK."); break;
        default:                    printf("Erro desconhecido (0x%02X)", codigo);
    }
    // printf("\n"); // Removido para ser adicionado pelo chamador se necessário
}

// Definições para bits de registradores usados em PCD_ComunicarComPICC
// (para evitar "magic numbers" e melhorar a leitura)
// ComIrqReg bits
#define ComIrqReg_TimerIRq  (1 << 0) // Bit 0: TimerIrq - Timer set to 0
#define ComIrqReg_ErrIRq    (1 << 1) // Bit 1: ErrIrq - Error bit is set
#define ComIrqReg_LoAlertIRq (1 << 2) // Bit 2: LoAlertIRq - LoAlert condition is fulfilled
#define ComIrqReg_HiAlertIRq (1 << 3) // Bit 3: HiAlertIRq - HiAlert condition is fulfilled
#define ComIrqReg_IdleIRq   (1 << 4) // Bit 4: IdleIRq - Command terminates or PCD enters Idle state
#define ComIrqReg_RxIRq     (1 << 5) // Bit 5: RxIRq - Receiver has detected end of data stream
#define ComIrqReg_TxIRq     (1 << 6) // Bit 6: TxIRq - End of data transmission
// Bit 7: Set1 / unused

// ErrorReg bits
#define ErrorReg_ProtocolErr (1 << 0) // Bit 0: ProtocolErr - SOF, EOF, parity etc.
#define ErrorReg_ParityErr   (1 << 1) // Bit 1: ParityErr - Parity error
#define ErrorReg_CRCErr      (1 << 2) // Bit 2: CRCErr - CRC error
#define ErrorReg_CollErr     (1 << 3) // Bit 3: CollErr - Collision detected
#define ErrorReg_BufferOvfl  (1 << 4) // Bit 4: BufferOvfl - FIFO buffer overflow
// Bit 5: reserved
#define ErrorReg_TempErr     (1 << 6) // Bit 6: TempErr - Internal temperature sensor detects overheating
#define ErrorReg_WrErr       (1 << 7) // Bit 7: WrErr - Error during write operation in MFRC522 EEPROM (not FIFO)

// Status2Reg bits
#define Status2Reg_MFCrypto1On (1 << 3) // Bit 3: MFCrypto1On - MIFARE Crypto1 unit is switched on

// CollReg bits
#define CollReg_CollPosNotValid (1 << 5) // Bit 5: CollPosNotValid - Collision position not valid

// DivIrqReg bits
#define DivIrqReg_CRCIRq    (1 << 2) // Bit 2: CRCIRq - CRC command is active and all data is processed