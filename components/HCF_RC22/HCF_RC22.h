#ifndef HCF_RC22_H // Alterado para corresponder ao seu nome de componente/arquivo
#define HCF_RC22_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "esp_err.h" // Para esp_err_t

// --- Definições de Pinos ---
// Substitua pelos seus pinos GPIO reais
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22
#define PIN_NUM_RST  14

// Descomente a linha abaixo se você quiser que a biblioteca controle o CS manualmente.
// Se descomentado, o spics_io_num na configuração do dispositivo SPI no main.c deve ser -1.
// #define MFRC522_MANUAL_CS

// --- Constantes MIFARE ---
enum MIFARE_Misc {
    MF_ACK                  = 0xA,
    MF_KEY_SIZE             = 6
};

// --- Estrutura para UID ---
typedef struct {
    uint8_t     size;
    uint8_t     uidByte[10];
    uint8_t     sak;
} Uid;

// --- Estrutura para Chave MIFARE ---
typedef struct {
    uint8_t     keyByte[MF_KEY_SIZE];
} MIFARE_Key;

// --- Registradores MFRC522 (com shift left) ---
enum PCD_Registrador {
    // Page 0
    CommandReg              = 0x01 << 1,
    ComIEnReg               = 0x02 << 1,
    DivIEnReg               = 0x03 << 1,
    ComIrqReg               = 0x04 << 1,
    DivIrqReg               = 0x05 << 1,
    ErrorReg                = 0x06 << 1,
    Status1Reg              = 0x07 << 1,
    Status2Reg              = 0x08 << 1,
    FIFODataReg             = 0x09 << 1,
    FIFOLevelReg            = 0x0A << 1,
    WaterLevelReg           = 0x0B << 1,
    ControlReg              = 0x0C << 1,
    BitFramingReg           = 0x0D << 1,
    CollReg                 = 0x0E << 1,
    // Page 1
    ModeReg                 = 0x11 << 1,
    TxModeReg               = 0x12 << 1,
    RxModeReg               = 0x13 << 1,
    TxControlReg            = 0x14 << 1,
    TxASKReg                = 0x15 << 1,
    TxSelReg                = 0x16 << 1,
    RxSelReg                = 0x17 << 1,
    RxThresholdReg          = 0x18 << 1,
    DemodReg                = 0x19 << 1,
    MfTxReg                 = 0x1C << 1,
    MfRxReg                 = 0x1D << 1,
    // Page 2
    CRCResultRegH           = 0x21 << 1,
    CRCResultRegL           = 0x22 << 1,
    ModWidthReg             = 0x24 << 1,
    RFCfgReg                = 0x26 << 1,
    GsNReg                  = 0x27 << 1,
    CWGsPReg                = 0x28 << 1,
    ModGsPReg               = 0x29 << 1,
    TModeReg                = 0x2A << 1,
    TPrescalerReg           = 0x2B << 1,
    TReloadRegH             = 0x2C << 1,
    TReloadRegL             = 0x2D << 1,
    // Page 3
    VersionReg              = 0x37 << 1
};

// --- Comandos MFRC522 (PCD) ---
enum PCD_Comando {
    PCD_Idle                = 0x00,
    PCD_Mem                 = 0x01,
    PCD_GenerateRandomID    = 0x02,
    PCD_CalcCRC             = 0x03,
    PCD_Transmit            = 0x04,
    PCD_NoCmdChange         = 0x07,
    PCD_Receive             = 0x08,
    PCD_Transceive          = 0x0C,
    PCD_MFAuthent           = 0x0E,
    PCD_SoftReset           = 0x0F
};

// --- Comandos PICC ---
enum PICC_Comando {
    PICC_CMD_REQA           = 0x26,
    PICC_CMD_WUPA           = 0x52,
    PICC_CMD_CT             = 0x88,
    PICC_CMD_SEL_CL1        = 0x93,
    PICC_CMD_SEL_CL2        = 0x95,
    PICC_CMD_SEL_CL3        = 0x97,
    PICC_CMD_HLTA           = 0x50,
    PICC_CMD_MF_AUTH_KEY_A  = 0x60,
    PICC_CMD_MF_AUTH_KEY_B  = 0x61,
    PICC_CMD_MF_READ        = 0x30,
    PICC_CMD_MF_WRITE       = 0xA0
};

// --- Tipos de PICC ---
typedef enum {
    PICC_TYPE_UNKNOWN       ,
    PICC_TYPE_ISO_14443_4   ,
    PICC_TYPE_ISO_18092     ,
    PICC_TYPE_MIFARE_MINI   ,
    PICC_TYPE_MIFARE_1K     ,
    PICC_TYPE_MIFARE_4K     ,
    PICC_TYPE_MIFARE_UL     ,
    PICC_TYPE_MIFARE_PLUS   ,
    PICC_TYPE_MIFARE_DESFIRE,
    PICC_TYPE_TNP3XXX       ,
    PICC_TYPE_NOT_COMPLETE  = 0xff
} PICC_Type;

// --- Códigos de Status ---
enum StatusCode {
    STATUS_OK               ,
    STATUS_ERROR            ,
    STATUS_COLLISION        ,
    STATUS_TIMEOUT          ,
    STATUS_NO_ROOM          ,
    STATUS_INTERNAL_ERROR   ,
    STATUS_INVALID          ,
    STATUS_CRC_WRONG        ,
    STATUS_MIFARE_NACK      = 0xff
};

// --- Definições de Bits de Registradores --- <<<<<<<<<<< ADICIONADO AQUI
// ComIrqReg bits (RegistradorRequisicaoInterrupcaoComunicacao)
#define ComIrqReg_TimerIRq  (1 << 0) // Bit 0: TimerIrq - Temporizador chegou a 0
#define ComIrqReg_ErrIRq    (1 << 1) // Bit 1: ErrIrq - Bit de erro está setado
#define ComIrqReg_LoAlertIRq (1 << 2) // Bit 2: LoAlertIRq - Condição de alerta baixo atendida
#define ComIrqReg_HiAlertIRq (1 << 3) // Bit 3: HiAlertIRq - Condição de alerta alto atendida
#define ComIrqReg_IdleIRq   (1 << 4) // Bit 4: IdleIRq - Comando termina ou PCD entra em estado Ocioso
#define ComIrqReg_RxIRq     (1 << 5) // Bit 5: RxIRq - Receptor detectou fim do fluxo de dados
#define ComIrqReg_TxIRq     (1 << 6) // Bit 6: TxIRq - Fim da transmissão de dados
// Bit 7: Set1 / não usado

// ErrorReg bits (RegistradorDeErro)
#define ErrorReg_ProtocolErr (1 << 0) // Bit 0: ProtocolErr - Erro de protocolo (SOF, EOF, paridade, etc.)
#define ErrorReg_ParityErr   (1 << 1) // Bit 1: ParityErr - Erro de paridade
#define ErrorReg_CRCErr      (1 << 2) // Bit 2: CRCErr - Erro de CRC
#define ErrorReg_CollErr     (1 << 3) // Bit 3: CollErr - Colisão detectada
#define ErrorReg_BufferOvfl  (1 << 4) // Bit 4: BufferOvfl - Overflow do buffer FIFO
// Bit 5: reservado
#define ErrorReg_TempErr     (1 << 6) // Bit 6: TempErr - Sensor de temperatura interno detecta superaquecimento
#define ErrorReg_WrErr       (1 << 7) // Bit 7: WrErr - Erro durante operação de escrita na EEPROM do MFRC522 (não FIFO)

// Status2Reg bits (RegistradorDeStatus2)
#define Status2Reg_MFCrypto1On (1 << 3) // Bit 3: MFCrypto1On - Unidade MIFARE Crypto1 está ligada

// CollReg bits (RegistradorDeColisao)
#define CollReg_CollPosNotValid (1 << 5) // Bit 5: CollPosNotValid - Posição de colisão não é válida

// DivIrqReg bits (RegistradorRequisicaoInterrupcaoDiversa)
#define DivIrqReg_CRCIRq    (1 << 2) // Bit 2: CRCIRq - Comando CRC está ativo e todos os dados foram processados
#define DivIrqReg_MfinActIRq (1 << 4) // Bit 4: MfinActIRq - ??? (Verificar datasheet, geralmente relacionado ao MFIN)
// Bit 7: Set2 / não usado


// --- Protótipos de Funções Públicas (Traduzidos) ---
void PCD_EscreverRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t valor);
void PCD_EscreverRegistradorMultiplos(spi_device_handle_t spi, uint8_t registrador, uint8_t contagem, uint8_t *valores);
uint8_t PCD_LerRegistrador(spi_device_handle_t spi, uint8_t registrador);
void PCD_LerRegistradorMultiplos(spi_device_handle_t spi, uint8_t registrador, uint8_t contagem, uint8_t *valores, uint8_t alinhamentoRx);
void PCD_LimparMascaraBitsRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t mascara);
void PCD_DefinirMascaraBitsRegistrador(spi_device_handle_t spi, uint8_t registrador, uint8_t mascara);

void PCD_Inicializar(spi_device_handle_t spi);
void PCD_LigarAntena(spi_device_handle_t spi);
esp_err_t PCD_ObterVersao(spi_device_handle_t spi);

bool PICC_NovoCartaoPresente(spi_device_handle_t spi);
uint8_t PICC_LerSerialCartao(spi_device_handle_t spi, Uid *uidCartao);
uint8_t PICC_RequisitarA(spi_device_handle_t spi, uint8_t *bufferATQA, uint8_t *tamanhoBuffer);
uint8_t PCD_TransceptarDados(spi_device_handle_t spi, uint8_t *dadosEnviar, uint8_t comprimentoEnviar, uint8_t *dadosRecebidos, uint8_t *comprimentoRecebido, uint8_t *bitsValidos, uint8_t alinhamentoRx, bool verificarCRC);
uint8_t PICC_Selecionar(spi_device_handle_t spi, Uid *uid, uint8_t bitsValidos);
uint8_t PICC_PararA(spi_device_handle_t spi);
void PCD_PararCripto1(spi_device_handle_t spi);

uint8_t PCD_CalcularCRC(spi_device_handle_t spi, uint8_t *dados, uint8_t comprimento, uint8_t *resultado);

uint8_t MIFARE_Ler(spi_device_handle_t spi, uint8_t enderecoBloco, uint8_t *buffer, uint8_t *tamanhoBuffer);
uint8_t MIFARE_Escrever(spi_device_handle_t spi, uint8_t enderecoBloco, uint8_t *buffer, uint8_t tamanhoBuffer);
uint8_t PCD_Autenticar(spi_device_handle_t spi, uint8_t comandoPICC, uint8_t enderecoBloco, MIFARE_Key *chave, Uid *uid);

void PICC_ObterNomeTipo(PICC_Type tipoPicc);
PICC_Type PICC_ObterTipo(uint8_t sak);
void PICC_ImprimirNoSerial(spi_device_handle_t spi, Uid *uid);
void PICC_ImprimirDetalhesNoSerial(Uid *uid);
void PICC_ImprimirMifareClassicNoSerial(spi_device_handle_t spi, Uid *uid, PICC_Type tipoPicc, MIFARE_Key *chave);
void PICC_ImprimirSetorMifareClassicNoSerial(spi_device_handle_t spi, Uid *uid, MIFARE_Key *chave, uint8_t setor);
void PICC_ImprimirMifareUltralightNoSerial(spi_device_handle_t spi);
void ObterNomeCodigoStatus(uint8_t codigo);
uint8_t Ultralight_LerPaginas(spi_device_handle_t spi, uint8_t pagina_inicio, uint8_t *dados_saida);

#endif // HCF_RC522_H