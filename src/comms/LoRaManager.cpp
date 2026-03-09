#include "LoRaManager.h"

bool LoRaManager::init() {
    // Configura os pinos do barramento SPI para o LoRa
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    // No Brasil (Salvador), as bandas ISM permitidas para LoRa 
    // geralmente são 915MHz ou 433MHz (se você comprou o RA-02 433MHz).
    // Mude para 915E6 se o seu módulo for 915MHz.
    if (!LoRa.begin(433E6)) { 
        Serial.println("FALHA: Módulo LoRa SX1278 não encontrado!");
        return false;
    }

    // --- SINTONIA FÍSICA DO LORA (Conforme sua engenharia) ---
    
    // Spreading Factor (SF): 7 a 12. 
    // SF 7 = Muito rápido (bom para 10Hz), menor alcance. 
    // SF 10 = Muito lento, alcance insano (>15km). Vamos usar 8 para um bom balanço.
    LoRa.setSpreadingFactor(8); 
    
    // Bandwidth (Largura de Banda). Maior = mais rápido. Menor = mais alcance.
    LoRa.setSignalBandwidth(125E3); 
    
    // Coding Rate (Correção de erro no ar). 4/5 é o padrão equilibrado.
    LoRa.setCodingRate4(5);
    
    // Potência de Saída: O RA-02 suporta até 20dBm (Pino PA_BOOST).
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

    Serial.println("LoRa SX1278 Inicializado (SF8, 125kHz, 20dBm)");
    return true;
}

void LoRaManager::sendTelemetry(PacketTelemetryLoRa_t &packet) {
    // 1. Calcula o CRC8 do pacote (ignorando o próprio byte de checksum que é o último)
    packet.checksum_crc8 = CRC::calculateCRC8((uint8_t*)&packet, sizeof(PacketTelemetryLoRa_t) - 1);

    // 2. Inicia o pacote em MODO IMPLÍCITO (Implicit Header)
    // Ao passar "true" para o beginPacket, economizamos os 4 bytes do cabeçalho de rádio padrão.
    // Isso é possível porque o tamanho (18 bytes) é estático e cravado!
    LoRa.beginPacket(true); 

    // 3. Escreve a struct C de uma só vez na FIFO do SX1278 (Cópia direta de memória)
    LoRa.write((uint8_t*)&packet, sizeof(PacketTelemetryLoRa_t));

    // 4. Dispara a transmissão (Bloqueante, mas como são 18 bytes a SF8, leva ~20ms, 
    // o que não afeta a nossa Task de 100ms no Core 0)
    LoRa.endPacket(); 
}

bool LoRaManager::receiveUplink(PacketUplinkLoRa_t &uplinkData, int8_t &rssi) {
    // Verifica se chegou algo no buffer do SX1278
    int packetSize = LoRa.parsePacket();
    
    // Se o tamanho bater exatamente com a nossa struct compactada (4 bytes)
    if (packetSize == sizeof(PacketUplinkLoRa_t)) {
        
        // Copia os dados do rádio direto para a memória da struct
        LoRa.readBytes((uint8_t*)&uplinkData, sizeof(PacketUplinkLoRa_t));
        
        // Validação de Segurança (Ignora ruído ou pacotes corrompidos)
        uint8_t calc_crc = CRC::calculateCRC8((uint8_t*)&uplinkData, sizeof(PacketUplinkLoRa_t) - 1);
        
        if (calc_crc == uplinkData.checksum_crc8 && uplinkData.sync_header == 0xBB) {
            // Pacote perfeito! Lemos o RSSI (Qualidade do sinal de quem enviou)
            rssi = LoRa.packetRssi(); 
            return true; 
        }
    }
    return false; // Não chegou nada ou pacote era lixo eletromagnético
}